#include "../Inc/FillingStation.h"

static volatile FillingStation fillStation;

static volatile FillingStationADCBuffer adcBuffer = {0};

uint16_t filteredTelemetryValues[16] = {0};

uint8_t uart_rx_buffer[132] = {0};
uint8_t uart_tx_buffer[44] = {0};

static void executeInit(uint32_t timestamp_ms);
static void executeIdle(uint32_t timestamp_ms);
static void executeAbort(uint32_t timestamp_ms);
static void executeArming(uint32_t timestamp_ms);
static void executeArmed(uint32_t timestamp_ms);
static void executeFilling(uint32_t timestamp_ms);
static void executeFilled(uint32_t timestamp_ms);

static void initPWMs();
static void initADC();
static void initGPIOs();
static void initUART();

static void initValves();
static void initTemperatureSensors();
static void initTelecom();

static void tickValves(uint32_t timestamp_ms);
static void tickTemperatureSensors();

static void handleTelecommunication(uint32_t timestamp_ms);
static void handleCurrentCommand();
static void handleCurrentCommandIdle();
static void handleCurrentCommandAbort();

static void filterTelemetryValues(uint8_t index);

static void sendTelemetryPacket(uint32_t timestamp_ms);
static void sendStatusPacket(uint32_t timestamp_ms);
static uint32_t computeCrc(uint8_t* data, uint16_t size);

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
    //.temperatureSensorErrorStatus = {0},
    //.pressureSensorErrorStatus = {0},
    //.engineErrorStatus = 0,
    //.engineStatus = 0,
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

void FillingStation_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, Valve* valves, TemperatureSensor* temperatureSensors, Telecommunication* telecom) {
  fillStation.errorStatus.value  = 0;
  fillStation.status.value       = 0;
  fillStation.currentState       = FILLING_STATION_STATE_INIT;

  fillStation.pwms   = pwms;
  fillStation.adc    = adc;
  fillStation.gpios  = gpios;
  fillStation.uart   = uart;

  fillStation.valves = valves;
  fillStation.temperatureSensors = temperatureSensors;
  fillStation.telecom = telecom;

  initValves();
  initTemperatureSensors();
  initTelecom();

  initADC();
  initPWMs();
  initGPIOs();
  initUART();
}

void FillingStation_tick(uint32_t timestamp_ms) {
  tickTemperatureSensors(timestamp_ms);
  tickValves(timestamp_ms);
  handleTelecommunication(timestamp_ms);

  FillingStation_execute(timestamp_ms);
}

void FillingStation_execute(uint32_t timestamp_ms) {
  switch (fillStation.currentState) {
    case FILLING_STATION_STATE_INIT:
      executeInit(timestamp_ms);
      break;
    case FILLING_STATION_STATE_IDLE:
      executeIdle(timestamp_ms);
      break;
    case FILLING_STATION_STATE_ARMING:
      executeArming(timestamp_ms);
      break;
    case FILLING_STATION_STATE_FILLING:
      executeFilling(timestamp_ms);
      break;
    case FILLING_STATION_STATE_ABORT:
      executeAbort(timestamp_ms);
      break;
    default:
    fillStation.errorStatus.bits.invalidState = 1;
      executeIdle(timestamp_ms);
      break;
  }
}

void executeInit(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < FILLING_STATION_VALVE_AMOUNT; i++) {
    fillStation.valves[i].close((struct Valve*)&fillStation.valves[i], timestamp_ms);
  }
  fillStation.currentState = FILLING_STATION_STATE_IDLE;

  fillStation.telecom->init((struct Telecommunication*) fillStation.telecom);
}

void executeIdle(uint32_t timestamp_ms) {
  
}

void executeArming(uint32_t timestamp_ms) {
  // Wait for ignition command, completely armed
}

void executeArmed(uint32_t timestamp_ms) {
  
}

void executeFilling(uint32_t timestamp_ms) {
  
}

void executeFilled(uint32_t timestamp_ms) {
  // stay mostly idle
}

void executeAbort(uint32_t timestamp_ms) {
  // Check flowcharts for wtf to do
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
    fillStation.adc->init((struct ADC12*)fillStation.adc, adcBuffer.values, sizeof(adcBuffer), FILLING_STATION_ADC_CHANNEL_AMOUNT);
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
  fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].pwm = &fillStation.pwms[FILLING_STATION_NOS_VALVE_PWM_INDEX];
  fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &fillStation.gpios[FILLING_STATION_NOS_VALVE_OPENED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &fillStation.gpios[FILLING_STATION_NOS_VALVE_OPENED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].openDutyCycle_pct = FILLING_STATION_FILL_VALVE_OPEN_DUTY_CYCLE_PCT;
  fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].closeDutyCycle_pct = FILLING_STATION_FILL_VALVE_CLOSED_DUTY_CYCLE_PCT;

  fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].pwm = &fillStation.pwms[FILLING_STATION_NOS_DUMP_VALVE_PWM_INDEX];
  fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &fillStation.gpios[FILLING_STATION_NOS_DUMP_VALVE_OPENED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &fillStation.gpios[FILLING_STATION_NOS_DUMP_VALVE_CLOSED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].openDutyCycle_pct = FILLING_STATION_DUMP_VALVE_OPEN_DUTY_CYCLE_PCT;
  fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].closeDutyCycle_pct = FILLING_STATION_DUMP_VALVE_CLOSED_DUTY_CYCLE_PCT;

  for (uint8_t i = 0; i < FILLING_STATION_VALVE_AMOUNT; i++) {
    if (fillStation.valves[i].init == FUNCTION_NULL_POINTER) {
      fillStation.valves[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    fillStation.valves[i].init((struct Valve*)&fillStation.valves[i]);
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

void handleCurrentCommand() {
  switch (fillStation.currentState) {
    case FILLING_STATION_STATE_INIT:
      break;
    case FILLING_STATION_STATE_IDLE:
      handleCurrentCommandIdle();
      break;
    case FILLING_STATION_STATE_ABORT:
      //handleCurrentCommandAbort();
      break;
    default:
      //engine.errorStatus.bits.invalidCommand = 1;
      break;
  }
}

void handleCurrentCommandIdle() {
  switch (currentCommand.fields.header.bits.commandCode) {
    case FILLING_STATION_COMMAND_CODE_OPEN_FILL_VALVE_PCT:
      if (currentCommand.fields.value <= 100) {
        fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].setDutyCycle((struct Valve*)&fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX], currentCommand.fields.value);
      } else {
        //fillStation.errorStatus.bits.invalidCommandValue = 1;
      }
      break;
  }
}

void sendTelemetryPacket(uint32_t timestamp_ms) {
  telemetryPacket.fields.timestamp_ms = timestamp_ms;
  for (uint8_t i = 0; i < ENGINE_ADC_CHANNEL_AMOUNT; i++) {
    filterTelemetryValues(i);
    telemetryPacket.fields.adcValues[i] = filteredTelemetryValues[i];
  }
  telemetryPacket.fields.crc = 0;
  fillStation.telecom->sendData((struct Telecommunication*)fillStation.telecom, telemetryPacket.data, sizeof(FillingStationTelemetryPacket));
}

void sendStatusPacket(uint32_t timestamp_ms) {
  statusPacket.fields.timestamp_ms = timestamp_ms;
  statusPacket.fields.errorStatus = fillStation.errorStatus;
  statusPacket.fields.status = fillStation.status;
  statusPacket.fields.valveStatus[FILLING_STATION_NOS_VALVE_INDEX] = fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].status;
  statusPacket.fields.valveStatus[FILLING_STATION_NOS_DUMP_VALVE_INDEX] = fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].status;
  statusPacket.fields.crc = 0;

  fillStation.telecom->sendData((struct Telecommunication*)fillStation.telecom, statusPacket.data, sizeof(FillingStationStatusPacket));
}

void getReceivedCommand() {
  for (uint8_t i = 0; i < sizeof(uart_rx_buffer) - sizeof(currentCommand) - 1; i++) {
    currentCommand.data[0] = uart_rx_buffer[i];
    currentCommand.data[1] = uart_rx_buffer[i + 1];
    currentCommand.data[2] = uart_rx_buffer[i + 2];
    currentCommand.data[3] = uart_rx_buffer[i + 3];
    if (currentCommand.fields.header.bits.type == BOARD_COMMAND_TYPE_CODE) {
      if (currentCommand.fields.header.bits.boardId == FILLING_STATION_BOARD_ID) {
        for (uint8_t j = 4; j < sizeof(BoardCommand); j++) {
          currentCommand.data[j] = uart_rx_buffer[i + j];
          if (checkCommandCrc()) {
            i += sizeof(BoardCommand) - 1;
            handleCurrentCommand();
            break;
          }
        }
      }
    }
  }
}

void filterTelemetryValues(uint8_t index) {
  uint32_t filteredValue = 0;
  for (uint16_t i = 0; i < 64; i++) {
    filteredValue += adcBuffer.values[index + i * FILTER_TELEMETRY_OFFSET];
  }
  filteredTelemetryValues[index] = filteredValue >> 6;
}

uint8_t checkCommandCrc() {
  //HAL_CRC_Calculate
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    getReceivedCommand();
    HAL_UART_Receive_DMA(huart, uart_rx_buffer, sizeof(uart_rx_buffer));
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    // Transmission complete callback
    // Example: Prepare next data to send if needed
  }
}