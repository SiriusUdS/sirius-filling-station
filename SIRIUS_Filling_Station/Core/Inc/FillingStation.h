#pragma once

#include "../sirius-embedded-common/Inc/Sensor/PressureSensor/ETM375.h"
#include "../sirius-embedded-common/sirius-headers-common/PressureSensor/PressureSensorData.h"

#include "../sirius-embedded-common/Inc/Sensor/TemperatureSensor/NTC3950.h"
#include "../sirius-embedded-common/sirius-headers-common/TemperatureSensor/TemperatureSensorPacket.h"

#include "../sirius-embedded-common/Inc/Device/Heater/FTVOGUEanpih0ztre.h"

#include "../sirius-embedded-common/Inc/Device/Valve/HBL388.h"

#include "../sirius-embedded-common/Inc/Device/Storage/SDCard.h"

#include "../sirius-embedded-common/Inc/LowLevelDriver/GPIO/GPIOHAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/PWM/PWMHAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/ADC/ADC12HAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/UART/UARTHAL.h"
#include "../sirius-embedded-common/Inc/LowLevelDriver/USB/USBHAL.h"

#include "../sirius-embedded-common/sirius-headers-common/FillingStation/FillingStationStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/FillingStation/FillingStationErrorStatus.h"
#include "../sirius-embedded-common/sirius-headers-common/FillingStation/FillingStationSensors.h"
#include "../sirius-embedded-common/sirius-headers-common/FillingStation/FillingStationState.h"

#include "../sirius-embedded-common/Inc/Device/Telecommunication/Telecommunication.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/TelemetryPacket.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/BoardCommand.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/CommandResponse.h"
#include "../sirius-embedded-common/Inc/Device/Telecommunication/XBEE.h"
#include "../sirius-embedded-common/sirius-headers-common/Telecommunication/PacketHeaderVariable.h"

#include "stm32f4xx_hal.h"

#define FUNCTION_NULL_POINTER 0

#define ADC_BUFFER_SIZE_BYTES 0x10000

#define TIME_BETWEEN_TELEMETRY_PACKETS_MS        (uint8_t)91
#define TELEMETRY_PACKETS_BETWEEN_STATUS_PACKETS (uint8_t)5

#define FILTER_TELEMETRY_OFFSET (((sizeof(FillingStationADCBuffer) / 2)/sizeof(uint16_t)) / 64)

#define FILLING_STATION_FILL_VALVE_OPEN_DUTY_CYCLE_PCT   (uint8_t)26
#define FILLING_STATION_FILL_VALVE_CLOSED_DUTY_CYCLE_PCT (uint8_t)54

#define FILLING_STATION_DUMP_VALVE_OPEN_DUTY_CYCLE_PCT   (uint8_t)20
#define FILLING_STATION_DUMP_VALVE_CLOSED_DUTY_CYCLE_PCT (uint8_t)60

typedef union {
  uint16_t values[ADC_BUFFER_SIZE_BYTES / sizeof(uint16_t)];

  uint8_t hex[ADC_BUFFER_SIZE_BYTES];
}
FillingStationADCBuffer;

typedef struct {
  FillingStationErrorStatus errorStatus;
  FillingStationStatus      status;

  uint8_t currentState;

  ADC12* adc;
  PWM*   pwms;
  GPIO*  gpios;
  UART*  uart;

  CRC_HandleTypeDef* hcrc;

  Valve*             valves;
  Heater*            heaters;
  TemperatureSensor* temperatureSensors;
  PressureSensor*    pressureSensors;
  Telecommunication* telecom;
  uint32_t           telecommunicationTimestampTarget_ms;
  uint8_t            telecommunicationTelemetryPacketCount;
}
FillingStation;

extern void FillingStation_init(PWM* pwms, ADC12* adc, GPIO* gpios, UART* uart, Valve* valves, Heater* heaters, TemperatureSensor* temperatureSensors, Telecommunication* telecom, CRC_HandleTypeDef* hcrc);

extern void FillingStation_tick(uint32_t timestamp_ms);

extern void FillingStation_execute(uint32_t timestamp_ms);