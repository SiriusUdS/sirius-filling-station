#include "../Inc/Engine.h"

static volatile Engine engine;

uint32_t previous;
uint32_t previous2;
uint16_t testValueThermistance = 0;

static void executeInit(uint32_t timestamp_ms);
static void executeIdle(uint32_t timestamp_ms);
static void executeArming(uint32_t timestamp_ms);
static void executeIgnition(uint32_t timestamp_ms);
static void executePoweredFlight(uint32_t timestamp_ms);
static void executeUnpoweredFlight(uint32_t timestamp_ms);
static void executeAbort(uint32_t timestamp_ms);

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

void Engine_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, USB* usb, Valve* valves, TemperatureSensor* temperatureSensors, Telecommunication* telecom) {
  engine.errorStatus.value  = 0;
  engine.status.value       = 0;
  engine.currentState       = ENGINE_STATE_INIT;
  

  engine.pwms   = pwms;
  engine.adc    = adc;
  engine.gpios  = gpios;
  engine.uart   = uart;
  engine.usb    = usb;

  engine.valves = valves;
  engine.temperatureSensors = temperatureSensors;
  engine.telecom = telecom;

  initValves();
  initTemperatureSensors();
  initTelecom();

  initADC();
  initPWMs();
  initGPIOs();
  initUART();
  initUSB();
}

void Engine_tick(uint32_t timestamp_ms) {
  tickTemperatureSensors(timestamp_ms);
  tickValves(timestamp_ms);

  Engine_execute(timestamp_ms);
}

void Engine_execute(uint32_t timestamp_ms) {
  switch (engine.currentState) {
    case ENGINE_STATE_INIT:
      executeInit(timestamp_ms);
      break;
    case ENGINE_STATE_IDLE:
      executeIdle(timestamp_ms);
      break;
    case ENGINE_STATE_ARMING:
      executeArming(timestamp_ms);
      break;
    case ENGINE_STATE_IGNITION:
      executeIgnition(timestamp_ms);
      break;
    case ENGINE_STATE_POWERED_FLIGHT:
      executePoweredFlight(timestamp_ms);
      break;
    case ENGINE_STATE_UNPOWERED_FLIGHT:
      executeUnpoweredFlight(timestamp_ms);
      break;
    case ENGINE_STATE_ABORT:
      executeAbort(timestamp_ms);
      break;
    default:
      engine.errorStatus.bits.invalidState = 1;
      executeIdle(timestamp_ms);
      break;
  }
}

void executeInit(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    engine.valves[i].close((struct Valve*)&engine.valves[i], timestamp_ms);
  }
  engine.currentState = ENGINE_STATE_IDLE;

  engine.telecom->setupTelecom((struct Telecommunication*) engine.telecom);
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
          .status = engine.temperatureSensors[0].status,
          .errorStatus = engine.temperatureSensors[0].errorStatus,
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
    engine.usb->transmit((struct USB*)engine.usb, testPacket.data, sizeof(TemperatureSensorPacket));
  }

  if (engine.usb->status.bits.rxDataReady == 1) {
    uint8_t* test = engine.usb->rxBuffer;
    engine.usb->status.bits.rxDataReady = 0;
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

void executeIgnition(uint32_t timestamp_ms) {
  engine.valves[ENGINE_IPA_VALVE_INDEX].open((struct Valve*)&engine.valves[ENGINE_IPA_VALVE_INDEX], timestamp_ms);
}

void executePoweredFlight(uint32_t timestamp_ms) {
  // save as much data as possible
}

void executeUnpoweredFlight(uint32_t timestamp_ms) {
  // stay mostly idle
}

void executeAbort(uint32_t timestamp_ms) {
  // Check flowcharts for wtf to do
}

void initPWMs() {
  for (uint8_t i = 0; i < ENGINE_PWM_AMOUNT; i++) {
    if (engine.pwms[i].init == FUNCTION_NULL_POINTER) {
      engine.pwms[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.pwms[i].init((struct PWM*)&engine.pwms[i]);
  }
}

void initADC() {
  if (engine.adc->init == FUNCTION_NULL_POINTER) {
    engine.adc->errorStatus.bits.nullFunctionPointer = 1;
  }
  else {
    engine.adc->init((struct ADC12*)engine.adc, ENGINE_ADC_CHANNEL_AMOUNT);
  }

  for (uint8_t i = 0; i < ENGINE_ADC_CHANNEL_AMOUNT; i++) {
    if (engine.adc->channels[i].init == FUNCTION_NULL_POINTER) {
      engine.adc->channels[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.adc->channels[i].init((struct ADC12Channel*)&engine.adc->channels[i]);
  }
}

void initGPIOs() {
  for (uint8_t i = 0; i < ENGINE_GPIO_AMOUNT; i++) {
    if (engine.gpios[i].init == FUNCTION_NULL_POINTER) {
      engine.gpios[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.gpios[i].init((struct GPIO*)&engine.gpios[i]);
  }
}

void initUART() {
  if (engine.uart->init == FUNCTION_NULL_POINTER) {
    engine.uart->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  engine.uart->init(engine.uart);
}

void initUSB() {
  if (engine.usb->init == FUNCTION_NULL_POINTER) {
    engine.usb->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  engine.usb->init((struct USB*)engine.usb);
}

void initValves() {
  engine.valves[ENGINE_IPA_VALVE_INDEX].pwm = &engine.pwms[ENGINE_IPA_VALVE_PWM_INDEX];
  engine.valves[ENGINE_IPA_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &engine.gpios[ENGINE_IPA_VALVE_OPENED_GPIO_INDEX];
  engine.valves[ENGINE_IPA_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &engine.gpios[ENGINE_IPA_VALVE_CLOSED_GPIO_INDEX];

  engine.valves[ENGINE_NOS_VALVE_INDEX].pwm = &engine.pwms[ENGINE_NOS_VALVE_PWM_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].gpio[VALVE_GPIO_OPENED_INDEX] = &engine.gpios[ENGINE_NOS_VALVE_OPENED_GPIO_INDEX];
  engine.valves[ENGINE_NOS_VALVE_INDEX].gpio[VALVE_GPIO_CLOSED_INDEX] = &engine.gpios[ENGINE_NOS_VALVE_CLOSED_GPIO_INDEX];

  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    if (engine.valves[i].init == FUNCTION_NULL_POINTER) {
      engine.valves[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.valves[i].init((struct Valve*)&engine.valves[i]);
  }
}

void initTemperatureSensors() {
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_1_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_2_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_3_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_4_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_5_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_6_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_7_THERMISTANCE_ADC_CHANNEL_INDEX];
  engine.temperatureSensors[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_INDEX].adcChannel = &engine.adc->channels[ENGINE_COMBUSTION_CHAMBER_8_THERMISTANCE_ADC_CHANNEL_INDEX];

  for (uint8_t i = 0; i < ENGINE_THERMISTANCE_AMOUNT; i++) {
    if (engine.temperatureSensors[i].init == FUNCTION_NULL_POINTER) {
      engine.temperatureSensors[i].errorStatus.bits.nullFunctionPointer = 1;
      continue;
    }

    engine.temperatureSensors[i].init((struct TemperatureSensor*)&engine.temperatureSensors[i]);
  }
}

void initTelecom(){
  if(engine.telecom->init == FUNCTION_NULL_POINTER){
    engine.telecom->errorStatus.bits.nullFunctionPointer = 1;
    return;
  }

  engine.telecom->init((struct Telecommunication*)engine.telecom);
  engine.telecom->uart = engine.uart;
}

void tickValves(uint32_t timestamp_ms) {
  for (uint8_t i = 0; i < ENGINE_VALVE_AMOUNT; i++) {
    engine.valves[i].tick((struct Valve*)&engine.valves[i], timestamp_ms);
  }
}

void tickTemperatureSensors() {
  for (uint8_t i = 0; i < ENGINE_THERMISTANCE_AMOUNT; i++) {
    engine.temperatureSensors[i].tick((struct TemperatureSensor*)&engine.temperatureSensors[i]);
  }
}