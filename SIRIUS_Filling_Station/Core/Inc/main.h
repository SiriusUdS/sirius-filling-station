/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_INPUT_EMATCH_INDICATOR_2_Pin GPIO_PIN_5
#define GPIO_INPUT_EMATCH_INDICATOR_2_GPIO_Port GPIOE
#define GPIO_INPUT_EMATCH_INDICATOR_1_Pin GPIO_PIN_6
#define GPIO_INPUT_EMATCH_INDICATOR_1_GPIO_Port GPIOE
#define ADC_PRESSURE_SENSOR_TANK_Pin GPIO_PIN_0
#define ADC_PRESSURE_SENSOR_TANK_GPIO_Port GPIOC
#define ADC_PRESSURE_SENSOR_COMBUSTION_CHAMBER_Pin GPIO_PIN_1
#define ADC_PRESSURE_SENSOR_COMBUSTION_CHAMBER_GPIO_Port GPIOC
#define ADC_BATTERY_TENSION_Pin GPIO_PIN_2
#define ADC_BATTERY_TENSION_GPIO_Port GPIOC
#define ADC_UNUSED_3_Pin GPIO_PIN_3
#define ADC_UNUSED_3_GPIO_Port GPIOC
#define ADC_TEMPERATURE_SENSOR_1_Pin GPIO_PIN_0
#define ADC_TEMPERATURE_SENSOR_1_GPIO_Port GPIOA
#define ADC_TEMPERATURE_SENSOR_2_Pin GPIO_PIN_1
#define ADC_TEMPERATURE_SENSOR_2_GPIO_Port GPIOA
#define ADC_TEMPERATURE_SENSOR_3_Pin GPIO_PIN_2
#define ADC_TEMPERATURE_SENSOR_3_GPIO_Port GPIOA
#define ADC_TEMPERATURE_SENSOR_4_Pin GPIO_PIN_3
#define ADC_TEMPERATURE_SENSOR_4_GPIO_Port GPIOA
#define ADC_TEMPERATURE_SENSOR_5_Pin GPIO_PIN_4
#define ADC_TEMPERATURE_SENSOR_5_GPIO_Port GPIOA
#define ADC_TEMPERATURE_SENSOR_6_Pin GPIO_PIN_5
#define ADC_TEMPERATURE_SENSOR_6_GPIO_Port GPIOA
#define ADC_TEMPERATURE_SENSOR_7_Pin GPIO_PIN_6
#define ADC_TEMPERATURE_SENSOR_7_GPIO_Port GPIOA
#define ADC_TEMPERATURE_SENSOR_8_Pin GPIO_PIN_7
#define ADC_TEMPERATURE_SENSOR_8_GPIO_Port GPIOA
#define ADC_UNUSED_LOAD_SENSOR_1_Pin GPIO_PIN_4
#define ADC_UNUSED_LOAD_SENSOR_1_GPIO_Port GPIOC
#define ADC_UNUSED_LOAD_SENSOR_2_Pin GPIO_PIN_5
#define ADC_UNUSED_LOAD_SENSOR_2_GPIO_Port GPIOC
#define ADC_UNUSED_1_Pin GPIO_PIN_0
#define ADC_UNUSED_1_GPIO_Port GPIOB
#define ADC_UNUSED_2_Pin GPIO_PIN_1
#define ADC_UNUSED_2_GPIO_Port GPIOB
#define GPIO_INPUT_NOS_VALVE_SWITCH_CLOSED_Pin GPIO_PIN_7
#define GPIO_INPUT_NOS_VALVE_SWITCH_CLOSED_GPIO_Port GPIOE
#define GPIO_INPUT_NOS_VALVE_SWITCH_OPENED_Pin GPIO_PIN_8
#define GPIO_INPUT_NOS_VALVE_SWITCH_OPENED_GPIO_Port GPIOE
#define GPIO_INPUT_IPA_VALVE_SWITCH_CLOSED_Pin GPIO_PIN_9
#define GPIO_INPUT_IPA_VALVE_SWITCH_CLOSED_GPIO_Port GPIOE
#define GPIO_INPUT_IPA_VALVE_SWITCH_OPENED_Pin GPIO_PIN_10
#define GPIO_INPUT_IPA_VALVE_SWITCH_OPENED_GPIO_Port GPIOE
#define GPIO_OUTPUT_EXT_FLASH_HOLD_Pin GPIO_PIN_8
#define GPIO_OUTPUT_EXT_FLASH_HOLD_GPIO_Port GPIOD
#define GPIO_OUTPUT_EXT_FLASH_WP_Pin GPIO_PIN_9
#define GPIO_OUTPUT_EXT_FLASH_WP_GPIO_Port GPIOD
#define TIM4_PWM_IPA_VALVE_Pin GPIO_PIN_12
#define TIM4_PWM_IPA_VALVE_GPIO_Port GPIOD
#define TIM4_PWM_NOS_VALVE_Pin GPIO_PIN_13
#define TIM4_PWM_NOS_VALVE_GPIO_Port GPIOD
#define TIM4_UNUSED_CHANNEL_Pin GPIO_PIN_14
#define TIM4_UNUSED_CHANNEL_GPIO_Port GPIOD
#define GPIO_OUTPUT_HEATPAD_FILL_Pin GPIO_PIN_10
#define GPIO_OUTPUT_HEATPAD_FILL_GPIO_Port GPIOC
#define GPIO_OUTPUT_HEATPAD_DUMP_Pin GPIO_PIN_11
#define GPIO_OUTPUT_HEATPAD_DUMP_GPIO_Port GPIOC
#define GPIO_OUPUT_EMATCH_2_UNUSED_Pin GPIO_PIN_0
#define GPIO_OUPUT_EMATCH_2_UNUSED_GPIO_Port GPIOE
#define GPIO_OUTPUT_EMATCH_DUMP_Pin GPIO_PIN_1
#define GPIO_OUTPUT_EMATCH_DUMP_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
