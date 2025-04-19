#include "../Inc/FillingStation.h"

static volatile FillingStation fillStation;

uint32_t previous;
uint32_t previous2;
uint16_t testValueThermistance = 0;

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
static void initUSB();

static void initValves();
static void initTemperatureSensors();
static void initTelecom();

static void tickValves(uint32_t timestamp_ms);
static void tickTemperatureSensors();

void FillingStation_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, USB* usb, Valve* valves, TemperatureSensor* temperatureSensors, Telecommunication* telecom) {
  fillStation.errorStatus.value  = 0;
  fillStation.status.value       = 0;
  fillStation.currentState       = FILLING_STATION_STATE_INIT;

  fillStation.pwms   = pwms;
  fillStation.adc    = adc;
  fillStation.gpios  = gpios;
  fillStation.uart   = uart;
  fillStation.usb    = usb;

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
  initUSB();
}

void FillingStation_tick(uint32_t timestamp_ms) {
  tickTemperatureSensors(timestamp_ms);
  tickValves(timestamp_ms);

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
    case FILLING_STATION_STATE_ARMED:
      executeArmed(timestamp_ms);
      break;
    case FILLING_STATION_STATE_FILLING:
      executeFilling(timestamp_ms);
      break;
    case FILLING_STATION_STATE_FILLED:
      executeFilled(timestamp_ms);
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

  fillStation.telecom->setupTelecom((struct Telecommunication*) fillStation.telecom);
}

void executeIdle(uint32_t timestamp_ms) {
  TemperatureSensorPacket testPacket = {
    .fields = {
      .header = {
        .values[0] = TEMPERATURE_SENSOR_DATA_HEADER_CODE & 0xFFFFFF00
      },
      .rawData = {
        .members = {
          .data = {
            .rawTemperature = testValueThermistance
          },
          .status = fillStation.temperatureSensors[0].status,
          .errorStatus = fillStation.temperatureSensors[0].errorStatus,
          .timeStamp_ms = timestamp_ms
        }
      }
    }
  };
  uint8_t data[] = "FUCK TRUMP!";

  if (HAL_GetTick() - previous >= 100) {
    previous = HAL_GetTick();
    //CDC_Transmit_FS(data, sizeof(data) - 1);
    testValueThermistance++;
    testPacket.fields.rawData.members.data.rawTemperature = testValueThermistance;
    fillStation.usb->transmit((struct USB*)fillStation.usb, testPacket.data, sizeof(TemperatureSensorPacket));
  }

  if (fillStation.usb->status.bits.rxDataReady == 1) {
    uint8_t* test = fillStation.usb->rxBuffer;
    fillStation.usb->status.bits.rxDataReady = 0;
  }
  // Wait for arming command, collect data
  /*if(HAL_GetTick() - previous2 >= 500){
    previous2 = HAL_GetTick();

    engine.telecom->sendData((struct Telecommunication*)engine.telecom, data, sizeof(data)-1);
  }*/
  /*engine.valves[ENGINE_IPA_VALVE_INDEX].open((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], timestamp_ms);
  HAL_Delay(1000);
  engine.valves[ENGINE_IPA_VALVE_INDEX].setIdle((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX]);
  engine.valves[ENGINE_IPA_VALVE_INDEX].close((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], timestamp_ms);
  HAL_Delay(1000);
  engine.valves[ENGINE_IPA_VALVE_INDEX].setIdle((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX]);*/
  
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
    fillStation.adc->init((struct ADC12*)fillStation.adc, FILLING_STATION_ADC_CHANNEL_AMOUNT);
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
}

void initUSB() {
  if (fillStation.usb->init == FUNCTION_NULL_POINTER) {
    fillStation.usb->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  fillStation.usb->init((struct USB*)fillStation.usb);
}

void initValves() {
  fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].pwm = &fillStation.pwms[FILLING_STATION_NOS_VALVE_PWM_INDEX];
  fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &fillStation.gpios[FILLING_STATION_NOS_VALVE_OPENED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_NOS_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &fillStation.gpios[FILLING_STATION_NOS_VALVE_OPENED_GPIO_INDEX];

  fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].pwm = &fillStation.pwms[FILLING_STATION_NOS_DUMP_VALVE_PWM_INDEX];
  fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &fillStation.gpios[FILLING_STATION_NOS_DUMP_VALVE_OPENED_GPIO_INDEX];
  fillStation.valves[FILLING_STATION_NOS_DUMP_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &fillStation.gpios[FILLING_STATION_NOS_DUMP_VALVE_CLOSED_GPIO_INDEX];

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