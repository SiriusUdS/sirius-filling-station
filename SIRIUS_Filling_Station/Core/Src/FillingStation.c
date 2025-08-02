#include "../Inc/FillingStation.h"

static volatile FillingStation fillStation;

//static volatile FillingStationADCBuffer adcBuffer = {0};

uint32_t adcHalfFullTimestamp_ms = 0;
uint32_t adcFullTimestamp_ms = 0;

uint32_t timeSinceLastCommand_ms = 0;
uint32_t communicationRestartTimer_ms = 0;
uint32_t lastCommandTimestamp_ms = 0;

uint8_t activateStorageFlag = 0;

uint16_t filteredTelemetryValues[16] = {0};

uint8_t uart_rx_buffer[132] = {0};

static void executeInit(uint32_t timestamp_ms);
static void executeSafe(uint32_t timestamp_ms);
static void executeAbort(uint32_t timestamp_ms);
static void executeUnsafe(uint32_t timestamp_ms);

static void executeAbortCommand(uint32_t timestamp_ms);

static void initPWMs();
static void initADC();
static void initGPIOs();
static void initUART();

static void initValves();
static void initHeaters();
static void initTemperatureSensors();
static void initTelecom();
static void initIgniter();
static void initEmergencyButton();
static void initStorageDevices();

static void tickValves(uint32_t timestamp_ms);
static void tickTemperatureSensors();

static void handleTelecommunication(uint32_t timestamp_ms);
static void handleDataStorage(uint32_t timestamp_ms);

static void handleCurrentCommand();
static void handleCurrentCommandSafe();
static void handleCurrentCommandUnsafe();
static void handleCurrentCommandAbort();
static void handleCommandAcknowledge();

static void filterTelemetryValues(uint8_t index);

static void sendTelemetryPacket(uint32_t timestamp_ms);
static void sendStatusPacket(uint32_t timestamp_ms);

static void getReceivedCommand();
static uint8_t checkCommandCrc();

BoardCommand currentCommand = {0};

FillingStationStatusPacket statusPacket = {
  .fields = {
    .header = {
      .bits = {
        .type = STATUS_TYPE_CODE,
        .boardId = FILLING_STATION_BOARD_ID
      }
    },
    .timestamp_ms = 0,
    .errorStatus = {0},
    .status = {0},
    .valveStatus = {0},
    .padding = {0},
    .crc = 0
  }
};

FillingStationTelemetryPacket telemetryPacket = {
  .fields = {
    .header = {
      .bits = {
        .type = TELEMETRY_TYPE_CODE,
        .boardId = FILLING_STATION_BOARD_ID
      }
    },
    .timestamp_ms = 0,
    .adcValues = {0},
    .crc = 0
  }
};

void FillingStation_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, Valve* valves, Heater* heaters, TemperatureSensor* temperatureSensors, Telecommunication* telecom, Igniter* igniter, Button* emergencyButton,Storage* storageDevices, volatile EngineSDCardBuffer* sdCardBuffer,  CRC_HandleTypeDef* hcrc) {
  fillStation.errorStatus.value  = 0;
  fillStation.status.value       = 0;
  fillStation.currentState       = FILLING_STATION_STATE_INIT;

  fillStation.pwms   = pwms;
  fillStation.adc    = adc;
  fillStation.gpios  = gpios;
  fillStation.uart   = uart;

  fillStation.hcrc   = hcrc;

  fillStation.valves = valves;
  fillStation.heaters = heaters;
  fillStation.temperatureSensors = temperatureSensors;
  fillStation.telecom = telecom;
  fillStation.igniter = igniter;
  fillStation.emergencyButton = emergencyButton;

  fillStation.storageDevices = storageDevices;

  lastCommandTimestamp_ms = 0;
  timeSinceLastCommand_ms = 0;

  fillStation.sdCardBuffer = sdCardBuffer;

  // FUCKING REMOVE THIS
  activateStorageFlag = 1;
  fillStation.isStoringData = 1;

  initHeaters();
  initValves();
  initTemperatureSensors();
  initTelecom();
  initIgniter();
  initEmergencyButton();
  initStorageDevices();

  initADC();
  initPWMs();
  initGPIOs();
  initUART();
}

void FillingStation_tick(uint32_t timestamp_ms) {
  timeSinceLastCommand_ms = timestamp_ms - lastCommandTimestamp_ms;
  /*if (fillStation.currentState != FILLING_STATION_STATE_ABORT && timeSinceLastCommand_ms > 60000) {
    executeAbortCommand(HAL_GetTick());
  }*/

  if (timeSinceLastCommand_ms > communicationRestartTimer_ms + 3000) {
    communicationRestartTimer_ms = timeSinceLastCommand_ms;
    if (__HAL_UART_GET_FLAG((UART_HandleTypeDef*)fillStation.uart->externalHandle, UART_FLAG_ORE)) {
      __HAL_UART_CLEAR_OREFLAG((UART_HandleTypeDef*)fillStation.uart->externalHandle);
    }
    HAL_UART_DMAStop(fillStation.uart->externalHandle);
    HAL_UART_Receive_DMA(fillStation.uart->externalHandle, uart_rx_buffer, sizeof(uart_rx_buffer));
  }

  tickTemperatureSensors(timestamp_ms);
  tickValves(timestamp_ms);
  fillStation.status.bits.state = fillStation.currentState;
  handleTelecommunication(timestamp_ms);
  handleDataStorage(timestamp_ms);
  fillStation.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].tick((struct Storage*)&fillStation.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], timestamp_ms);
  fillStation.emergencyButton->tick((struct Button*)fillStation.emergencyButton, timestamp_ms);
  fillStation.igniter->tick((struct Igniter*)fillStation.igniter, timestamp_ms);

  FillingStation_execute(timestamp_ms);
}

void FillingStation_execute(uint32_t timestamp_ms) {
  switch (fillStation.currentState) {
    case FILLING_STATION_STATE_INIT:
      executeInit(timestamp_ms);
      break;
    case FILLING_STATION_STATE_SAFE:
      executeSafe(timestamp_ms);
      break;
    case FILLING_STATION_STATE_UNSAFE:
      executeUnsafe(timestamp_ms);
      break;
    case FILLING_STATION_STATE_ABORT:
      executeAbort(timestamp_ms);
      break;
    default:
    fillStation.errorStatus.bits.invalidState = 1;
      executeSafe(timestamp_ms);
      break;
  }
}

void executeInit(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < FILLING_STATION_VALVE_AMOUNT; i++) {
    fillStation.valves[i].close((struct Valve*)&fillStation.valves[i], timestamp_ms);
  }
  fillStation.currentState = FILLING_STATION_STATE_SAFE;

  fillStation.telecom->init((struct Telecommunication*) fillStation.telecom);
}

void executeSafe(uint32_t timestamp_ms) {
  if (fillStation.emergencyButton->status.bits.isPressed) {
    fillStation.currentState = FILLING_STATION_STATE_ABORT;
    executeAbortCommand(timestamp_ms);
    return;
  }
}

void executeUnsafe(uint32_t timestamp_ms) {
  if (fillStation.emergencyButton->status.bits.isPressed) {
    fillStation.currentState = FILLING_STATION_STATE_ABORT;
    executeAbortCommand(timestamp_ms);
    return;
  }
}

void executeAbort(uint32_t timestamp_ms) {
  
}

void initPWMs() {
  for (uint8_t i = 0; i < FILLING_STATION_PWM_AMOUNT; i++) {
    if (fillStation.pwms[i].init == FUNCTION_NULL_POINTER) {
      fillStation.pwms[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    fillStation.pwms[i].init((struct PWM*)&fillStation.pwms[i]);
  }
}

void initADC() {
  if (fillStation.adc->init == FUNCTION_NULL_POINTER) {
    fillStation.adc->errorStatus.bits.nullFunctionPointer = 1;
  }
  else {
    fillStation.adc->init((struct ADC12*)fillStation.adc, fillStation.sdCardBuffer->values, sizeof(EngineSDCardBuffer), FILLING_STATION_ADC_CHANNEL_AMOUNT);
  }

  for (uint8_t i = 0; i < FILLING_STATION_ADC_CHANNEL_AMOUNT; i++) {
    if (fillStation.adc->channels[i].init == FUNCTION_NULL_POINTER) {
      fillStation.adc->channels[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    fillStation.adc->channels[i].init((struct ADC12Channel*)&fillStation.adc->channels[i]);
  }
}

void initGPIOs() {
  for (uint8_t i = 0; i < FILLING_STATION_GPIO_AMOUNT; i++) {
    if (fillStation.gpios[i].init == FUNCTION_NULL_POINTER) {
      fillStation.gpios[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    fillStation.gpios[i].init((struct GPIO*)&fillStation.gpios[i]);
  }
}

void initUART() {
  if (fillStation.uart->init == FUNCTION_NULL_POINTER) {
    fillStation.uart->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  fillStation.uart->init((struct UART*)fillStation.uart);
  HAL_UART_Receive_DMA(fillStation.uart->externalHandle, uart_rx_buffer, sizeof(uart_rx_buffer));
}

void initValves() {
  fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].pwm = &fillStation.pwms[FILLING_STATION_FILL_VALVE_PWM_INDEX];
  fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &fillStation.gpios[FILLING_STATION_FILL_VALVE_OPENED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &fillStation.gpios[FILLING_STATION_FILL_VALVE_CLOSED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].heatpad = &fillStation.heaters[FILLING_STATION_FILL_VALVE_HEATPAD_INDEX];
  fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].openDutyCycle_pct = FILLING_STATION_FILL_VALVE_OPEN_DUTY_CYCLE_PCT;
  fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].closeDutyCycle_pct = FILLING_STATION_FILL_VALVE_CLOSED_DUTY_CYCLE_PCT;

  fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].pwm = &fillStation.pwms[FILLING_STATION_DUMP_VALVE_PWM_INDEX];
  fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &fillStation.gpios[FILLING_STATION_DUMP_VALVE_OPENED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &fillStation.gpios[FILLING_STATION_DUMP_VALVE_CLOSED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].heatpad = &fillStation.heaters[FILLING_STATION_DUMP_VALVE_HEATPAD_INDEX];
  fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].openDutyCycle_pct = FILLING_STATION_DUMP_VALVE_OPEN_DUTY_CYCLE_PCT;
  fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].closeDutyCycle_pct = FILLING_STATION_DUMP_VALVE_CLOSED_DUTY_CYCLE_PCT;

  for (uint8_t i = 0; i < FILLING_STATION_VALVE_AMOUNT; i++) {
    if (fillStation.valves[i].init == FUNCTION_NULL_POINTER) {
      fillStation.valves[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    fillStation.valves[i].init((struct Valve*)&fillStation.valves[i]);
  }
}

void initHeaters() {
  fillStation.heaters[FILLING_STATION_FILL_VALVE_HEATPAD_INDEX].gpio = &fillStation.gpios[FILLING_STATION_FILL_HEATPAD_GPIO_INDEX];
  fillStation.heaters[FILLING_STATION_DUMP_VALVE_HEATPAD_INDEX].gpio = &fillStation.gpios[FILLING_STATION_DUMP_HEATPAD_GPIO_INDEX];

  for (uint8_t i = 0; i < FILLING_STATION_HEATPAD_AMOUNT; i++) {
    if (fillStation.heaters[i].init == FUNCTION_NULL_POINTER) {
      fillStation.heaters[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    fillStation.heaters[i].period_s = 10;
    fillStation.heaters[i].currentDutyCycle_pct = 0;
    fillStation.heaters[i].lastSwitchTimestamp_ms = 0;
    fillStation.heaters[i].init((struct Heater*)&fillStation.heaters[i]);
  }
}

void initTemperatureSensors() {
  fillStation.temperatureSensors[FILLING_STATION_FILL_VALVE_THERMISTANCE_INDEX].adcChannel = &fillStation.adc->channels[FILLING_STATION_FILL_VALVE_THERMISTANCE_ADC_CHANNEL_INDEX];
  fillStation.temperatureSensors[FILLING_STATION_QUICK_CONNECT_THERMISTANCE_INDEX].adcChannel = &fillStation.adc->channels[FILLING_STATION_QUICK_CONNECT_THERMISTANCE_ADC_CHANNEL_INDEX];
  fillStation.temperatureSensors[FILLING_STATION_NOS_VALVE_THERMISTANCE_INDEX].adcChannel = &fillStation.adc->channels[FILLING_STATION_NOS_VALVE_THERMISTANCE_ADC_CHANNEL_INDEX];
  fillStation.temperatureSensors[FILLING_STATION_IPA_VALVE_THERMISTANCE_INDEX].adcChannel = &fillStation.adc->channels[FILLING_STATION_IPA_VALVE_THERMISTANCE_ADC_CHANNEL_INDEX];

  for (uint8_t i = 0; i < FILLING_STATION_TEMPERATURE_SENSOR_AMOUNT; i++) {
    if (fillStation.temperatureSensors[i].init == FUNCTION_NULL_POINTER) {
      fillStation.temperatureSensors[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    fillStation.temperatureSensors[i].init((struct TemperatureSensor*)&fillStation.temperatureSensors[i]);
  }
}

void initTelecom(){
  if(fillStation.telecom->init == FUNCTION_NULL_POINTER){
    fillStation.telecom->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  fillStation.telecom->init((struct Telecommunication*)fillStation.telecom);
  fillStation.telecom->uart = fillStation.uart;
}

void initStorageDevices() {
  for (uint8_t i = 0; i < 1; i++) {
    if (fillStation.storageDevices[i].init == FUNCTION_NULL_POINTER) {
      fillStation.storageDevices[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    fillStation.storageDevices[i].init((struct Storage*)&fillStation.storageDevices[i]);
  }
}

void initIgniter() {
  if (fillStation.igniter->init == FUNCTION_NULL_POINTER) {
    fillStation.igniter->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  fillStation.igniter->igniteDuration_ms = 3500;
  fillStation.igniter->gpio = &fillStation.gpios[FILLING_STATION_IGNITER_DUMP_GPIO_INDEX];
  fillStation.igniter->init((struct Igniter*)fillStation.igniter);
}

void initEmergencyButton() {
  if (fillStation.emergencyButton->init == FUNCTION_NULL_POINTER) {
    fillStation.emergencyButton->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  fillStation.emergencyButton->gpio = &fillStation.gpios[FILLING_STATION_BUTTON_EMERGENCY_STOP_GPIO_INDEX];
  fillStation.emergencyButton->init((struct Button*)fillStation.emergencyButton);
  fillStation.emergencyButton->debounceTargetReadCount = 15;
  fillStation.emergencyButton->debounceCurrentReadCount = 0;
  fillStation.emergencyButton->delayBetweenReads_ms = 4;

  fillStation.emergencyButton->status.bits.isPressed = 0;
}

void tickValves(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < FILLING_STATION_VALVE_AMOUNT; i++) {
    fillStation.valves[i].tick((struct Valve*)&fillStation.valves[i], timestamp_ms);
  }
}

void tickTemperatureSensors() {
  for (uint8_t i = 0; i < FILLING_STATION_TEMPERATURE_SENSOR_AMOUNT; i++) {
    fillStation.temperatureSensors[i].tick((struct TemperatureSensor*)&fillStation.temperatureSensors[i]);
  }
}

void handleDataStorage(uint32_t timestamp_ms) {
  if (fillStation.adc->status.bits.dmaFull) {
    if (fillStation.isStoringData) {
      fillStation.sdCardBuffer->sdData[1].footer.timestamp_ms = adcFullTimestamp_ms;
      fillStation.sdCardBuffer->sdData[1].footer.status = fillStation.status.value;
      fillStation.sdCardBuffer->sdData[1].footer.errorStatus = fillStation.errorStatus.value;
      fillStation.sdCardBuffer->sdData[1].footer.valveStatus[0] = fillStation.valves[0].status.value;
      fillStation.sdCardBuffer->sdData[1].footer.valveStatus[1] = fillStation.valves[1].status.value;
      fillStation.sdCardBuffer->sdData[1].footer.valveErrorStatus[0] = fillStation.valves[0].errorStatus.value;
      fillStation.sdCardBuffer->sdData[1].footer.valveErrorStatus[1] = fillStation.valves[1].errorStatus.value;
      fillStation.sdCardBuffer->sdData[1].footer.currentCommand[0] = currentCommand.fields.header.value;
      fillStation.sdCardBuffer->sdData[1].footer.currentCommand[0] = 0;
      fillStation.sdCardBuffer->sdData[1].footer.currentCommand[0] = 0;

      for (uint8_t i = 0; i < 32;i++) {
        fillStation.sdCardBuffer->sdData[1].footer.padding[i] = 0;
      }

      for (uint8_t i = 0; i < 16;i++) {
        fillStation.sdCardBuffer->sdData[1].footer.signature[i] = 0xFFFFFFFF;
      }

      fillStation.sdCardBuffer->sdData[1].footer.crc = HAL_CRC_Calculate(fillStation.hcrc, (uint32_t*)&fillStation.sdCardBuffer->sdData[1], (sizeof(EngineSDFormattedData) / sizeof(uint32_t)) -  sizeof(uint8_t));
      fillStation.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&fillStation.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_DATA_FAST_DESTINATION, (uint8_t*)(fillStation.sdCardBuffer->hex + (sizeof(EngineSDCardBuffer) / 2)), (sizeof(EngineSDCardBuffer) / 2));
    }
    
    fillStation.adc->status.bits.dmaFull = 0;
  }
  
  if (fillStation.adc->status.bits.dmaHalfFull) {
    if (fillStation.isStoringData) {
      fillStation.sdCardBuffer->sdData[0].footer.timestamp_ms = adcFullTimestamp_ms;
      fillStation.sdCardBuffer->sdData[0].footer.status = fillStation.status.value;
      fillStation.sdCardBuffer->sdData[0].footer.errorStatus = fillStation.errorStatus.value;
      fillStation.sdCardBuffer->sdData[0].footer.valveStatus[0] = fillStation.valves[0].status.value;
      fillStation.sdCardBuffer->sdData[0].footer.valveStatus[1] = fillStation.valves[1].status.value;
      fillStation.sdCardBuffer->sdData[0].footer.valveErrorStatus[0] = fillStation.valves[0].errorStatus.value;
      fillStation.sdCardBuffer->sdData[0].footer.valveErrorStatus[1] = fillStation.valves[1].errorStatus.value;
      // Those are the BoardCommand
      fillStation.sdCardBuffer->sdData[0].footer.currentCommand[0] = currentCommand.fields.header.value;
      fillStation.sdCardBuffer->sdData[0].footer.currentCommand[0] = 0;
      fillStation.sdCardBuffer->sdData[0].footer.currentCommand[0] = 0;

      for (uint8_t i = 0; i < 32;i++) {
        fillStation.sdCardBuffer->sdData[0].footer.padding[i] = 0;
      }

      for (uint8_t i = 0; i < 16;i++) {
        fillStation.sdCardBuffer->sdData[0].footer.signature[i] = 0xFFFFFFFF;
      }

      fillStation.sdCardBuffer->sdData[0].footer.crc = HAL_CRC_Calculate(fillStation.hcrc, (uint32_t*)&fillStation.sdCardBuffer->sdData[0], (sizeof(EngineSDFormattedData) / sizeof(uint32_t)) -  sizeof(uint8_t));
      fillStation.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX].store((struct Storage*)&fillStation.storageDevices[ENGINE_STORAGE_SD_CARD_INDEX], STORAGE_DATA_FAST_DESTINATION, (uint8_t*)(fillStation.sdCardBuffer->hex), (sizeof(EngineSDCardBuffer) / 2));
    }
    
    fillStation.adc->status.bits.dmaHalfFull = 0;
  }
}

void handleTelecommunication(uint32_t timestamp_ms) {
  if (fillStation.telecommunicationTimestampTarget_ms <= timestamp_ms) {
    if (fillStation.telecommunicationTelemetryPacketCount > TELEMETRY_PACKETS_BETWEEN_STATUS_PACKETS) {
      fillStation.telecommunicationTelemetryPacketCount = 0;
      sendStatusPacket(timestamp_ms);
    }
    else {
      sendTelemetryPacket(timestamp_ms);
      fillStation.telecommunicationTelemetryPacketCount++;
    }
    fillStation.telecommunicationTimestampTarget_ms = timestamp_ms + TIME_BETWEEN_TELEMETRY_PACKETS_MS;
  }
}

void executeAbortCommand(uint32_t timestamp_ms) {
  fillStation.currentState = FILLING_STATION_STATE_ABORT;
  fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].close((struct Valve*)&fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX], timestamp_ms);
  fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].open((struct Valve*)&fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX], timestamp_ms);
  fillStation.heaters[FILLING_STATION_FILL_VALVE_HEATPAD_INDEX].setDutyCycle_pct((struct Heater*)&fillStation.heaters[FILLING_STATION_FILL_VALVE_HEATPAD_INDEX], 0);
  fillStation.heaters[FILLING_STATION_DUMP_VALVE_HEATPAD_INDEX].setDutyCycle_pct((struct Heater*)&fillStation.heaters[FILLING_STATION_DUMP_VALVE_HEATPAD_INDEX], 0);
  fillStation.igniter->ignite((struct Igniter*)fillStation.igniter, timestamp_ms);
}

void handleCurrentCommand() {
  if (currentCommand.fields.header.bits.commandCode == BOARD_COMMAND_CODE_ACK) {
    handleCommandAcknowledge();
  }

  switch (fillStation.currentState) {
    case FILLING_STATION_STATE_INIT:
      break;
    case FILLING_STATION_STATE_SAFE:
      handleCurrentCommandSafe();
      break;
    case FILLING_STATION_STATE_UNSAFE:
      handleCurrentCommandUnsafe();
      break;
    case FILLING_STATION_STATE_ABORT:
      handleCurrentCommandAbort();
      break;
    default:
      break;
  }
}

void handleCurrentCommandSafe() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case BOARD_COMMAND_CODE_ABORT:
      executeAbortCommand(HAL_GetTick());
      break;
    case FILLING_STATION_COMMAND_CODE_SET_FILL_VALVE_HEATER_POWER_PCT:
      if (currentCommand.fields.value <= 100) {
        fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].heatpad, currentCommand.fields.value);
      }
      break;
    case FILLING_STATION_COMMAND_CODE_SET_DUMP_VALVE_HEATER_POWER_PCT:
      if (currentCommand.fields.value <= 100) {
        fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].heatpad, currentCommand.fields.value);
      }
      break;
    case BOARD_COMMAND_CODE_UNSAFE:
      fillStation.currentState = FILLING_STATION_STATE_UNSAFE;
      break;
  }
}

void handleCurrentCommandUnsafe() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case BOARD_COMMAND_CODE_ABORT:
      executeAbortCommand(HAL_GetTick());
      break;
    case BOARD_COMMAND_CODE_SAFE:
      fillStation.currentState = FILLING_STATION_STATE_SAFE;
      break;
    case FILLING_STATION_COMMAND_CODE_SET_FILL_VALVE_HEATER_POWER_PCT:
      if (currentCommand.fields.value <= 100) {
        fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].heatpad, currentCommand.fields.value);
      }
      break;
    case FILLING_STATION_COMMAND_CODE_SET_DUMP_VALVE_HEATER_POWER_PCT:
      if (currentCommand.fields.value <= 100) {
        fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].heatpad->setDutyCycle_pct((struct Heater*)fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].heatpad, currentCommand.fields.value);
      }
      break;
    case FILLING_STATION_COMMAND_CODE_OPEN_FILL_VALVE_PCT:
      if (currentCommand.fields.value <= 100) {
        fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].setDutyCycle((struct Valve*)&fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX], currentCommand.fields.value, HAL_GetTick());
      }
      break;
    case FILLING_STATION_COMMAND_CODE_OPEN_DUMP_VALVE_PCT:
      if (currentCommand.fields.value <= 100) {
        fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].setDutyCycle((struct Valve*)&fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX], currentCommand.fields.value, HAL_GetTick());
      }
      break;
    default:
      break;
  }
}

void handleCurrentCommandAbort() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case BOARD_COMMAND_CODE_RESET:
      fillStation.currentState = FILLING_STATION_STATE_SAFE;
      break;
    default:
      break;
  }
}

void handleCommandAcknowledge() {
  CommandResponse commandResponse = {
    .fields = {
      .header = {
        .bits = {
          .type = COMMAND_RESPONSE_TYPE_CODE,
          .boardId = GS_CONTROL_BOARD_ID,
          .commandIndex = currentCommand.fields.header.bits.commandIndex,
          .response = RESPONSE_CODE_OK
        }
      },
      .crc = 0
    }
  };

  commandResponse.fields.crc = HAL_CRC_Calculate((CRC_HandleTypeDef*)fillStation.hcrc, commandResponse.data32, (sizeof(CommandResponse) / sizeof(uint32_t)) - sizeof(uint8_t));

  HAL_UART_Transmit_DMA(fillStation.uart->externalHandle, commandResponse.data, sizeof(CommandResponse));
}

void sendTelemetryPacket(uint32_t timestamp_ms) {
  telemetryPacket.fields.timestamp_ms = timestamp_ms;
  for (uint8_t i = 0; i < ENGINE_ADC_CHANNEL_AMOUNT; i++) {
    filterTelemetryValues(i);
    telemetryPacket.fields.adcValues[i] = filteredTelemetryValues[i];
  }
  telemetryPacket.fields.crc = HAL_CRC_Calculate(fillStation.hcrc, telemetryPacket.data32, (sizeof(FillingStationTelemetryPacket) / sizeof(uint32_t)) - sizeof(uint8_t));
  fillStation.telecom->sendData((struct Telecommunication*)fillStation.telecom, telemetryPacket.data, sizeof(FillingStationTelemetryPacket));
}

void sendStatusPacket(uint32_t timestamp_ms) {
  statusPacket.fields.timestamp_ms = timestamp_ms;
  statusPacket.fields.errorStatus = fillStation.errorStatus;
  statusPacket.fields.status = fillStation.status;
  statusPacket.fields.timeSinceLastCommand_ms = timeSinceLastCommand_ms;
  statusPacket.fields.valveStatus[FILLING_STATION_FILL_VALVE_INDEX] = fillStation.valves[FILLING_STATION_FILL_VALVE_INDEX].status;
  statusPacket.fields.valveStatus[FILLING_STATION_DUMP_VALVE_INDEX] = fillStation.valves[FILLING_STATION_DUMP_VALVE_INDEX].status;
  statusPacket.fields.crc = 0;

  statusPacket.fields.crc = HAL_CRC_Calculate(fillStation.hcrc, statusPacket.data32, (sizeof(FillingStationStatusPacket) / sizeof(uint32_t)) - sizeof(uint8_t));
  fillStation.telecom->sendData((struct Telecommunication*)fillStation.telecom, statusPacket.data, sizeof(FillingStationStatusPacket));
}

void getReceivedCommand() {
  for (uint16_t i = 0; i < sizeof(uart_rx_buffer) - sizeof(currentCommand) - 1; i++) {
    currentCommand.data[0] = uart_rx_buffer[i];
    currentCommand.data[1] = uart_rx_buffer[i + 1];
    currentCommand.data[2] = uart_rx_buffer[i + 2];
    currentCommand.data[3] = uart_rx_buffer[i + 3];
    if (currentCommand.fields.header.bits.type == BOARD_COMMAND_BROADCAST_TYPE_CODE || 
        (currentCommand.fields.header.bits.type == BOARD_COMMAND_UNICAST_TYPE_CODE && 
         currentCommand.fields.header.bits.boardId == FILLING_STATION_BOARD_ID)) {
      for (uint16_t j = 4; j < sizeof(BoardCommand); j++) {
        currentCommand.data[j] = uart_rx_buffer[i + j];
        if (checkCommandCrc()) {
          i += sizeof(BoardCommand) - 1;
          statusPacket.fields.lastReceivedCommandCode = currentCommand.fields.header.bits.commandCode;
          lastCommandTimestamp_ms = HAL_GetTick();
          timeSinceLastCommand_ms = 0;
          communicationRestartTimer_ms = 0;
          handleCurrentCommand();
          break;
        }
      }
    }
  }
}

void filterTelemetryValues(uint8_t index) {
  uint32_t filteredValue = 0;
  for (uint16_t i = 0; i < 64; i++) {
    filteredValue += fillStation.sdCardBuffer->values[index + i * FILTER_TELEMETRY_OFFSET];
  }
  filteredTelemetryValues[index] = filteredValue >> 6;
}

uint8_t checkCommandCrc() {
  if (HAL_CRC_Calculate(fillStation.hcrc, currentCommand.data32, (sizeof(BoardCommand) / sizeof(uint32_t)) - sizeof(uint8_t)) != currentCommand.fields.crc) {
    return 0;
  }
  return 1;
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if (fillStation.adc->status.bits.dmaHalfFull == 0) {
    fillStation.adc->status.bits.dmaHalfFull = 1;
    adcHalfFullTimestamp_ms = HAL_GetTick();
    if (activateStorageFlag) {
      fillStation.isStoringData = 1;
    }
    else {
      fillStation.isStoringData = 0;
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (fillStation.adc->status.bits.dmaFull == 0) {
    fillStation.adc->status.bits.dmaFull = 1;
    adcFullTimestamp_ms = HAL_GetTick();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    getReceivedCommand();
    HAL_UART_Receive_DMA(huart, uart_rx_buffer, sizeof(uart_rx_buffer));
  }
}