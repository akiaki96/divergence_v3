/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR_L_Pin GPIO_PIN_14
#define IR_L_GPIO_Port GPIOC
#define IR_FL_Pin GPIO_PIN_15
#define IR_FL_GPIO_Port GPIOC
#define IR_R_Pin GPIO_PIN_0
#define IR_R_GPIO_Port GPIOH
#define IR_FR_Pin GPIO_PIN_1
#define IR_FR_GPIO_Port GPIOH
#define VOL_CHECK_Pin GPIO_PIN_0
#define VOL_CHECK_GPIO_Port GPIOC
#define PUSH_IN_1_Pin GPIO_PIN_1
#define PUSH_IN_1_GPIO_Port GPIOC
#define SENSOR_FR_Pin GPIO_PIN_0
#define SENSOR_FR_GPIO_Port GPIOA
#define SENSOR_R_Pin GPIO_PIN_1
#define SENSOR_R_GPIO_Port GPIOA
#define SENSOR_FL_Pin GPIO_PIN_2
#define SENSOR_FL_GPIO_Port GPIOA
#define SENSOR_L_Pin GPIO_PIN_3
#define SENSOR_L_GPIO_Port GPIOA
#define SR_G_Pin GPIO_PIN_4
#define SR_G_GPIO_Port GPIOA
#define MOTOR_L_PWM_Pin GPIO_PIN_5
#define MOTOR_L_PWM_GPIO_Port GPIOA
#define LED_4_Pin GPIO_PIN_6
#define LED_4_GPIO_Port GPIOA
#define SR_LATCH_Pin GPIO_PIN_7
#define SR_LATCH_GPIO_Port GPIOA
#define MOTOR_R_CCW_Pin GPIO_PIN_4
#define MOTOR_R_CCW_GPIO_Port GPIOC
#define MOTOR_R_CW_Pin GPIO_PIN_5
#define MOTOR_R_CW_GPIO_Port GPIOC
#define MOTOR_STBY_Pin GPIO_PIN_0
#define MOTOR_STBY_GPIO_Port GPIOB
#define MOTOR_L_CW_Pin GPIO_PIN_1
#define MOTOR_L_CW_GPIO_Port GPIOB
#define MOTOR_L_CCW_Pin GPIO_PIN_10
#define MOTOR_L_CCW_GPIO_Port GPIOB
#define MOTOR_R_PWM_Pin GPIO_PIN_11
#define MOTOR_R_PWM_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_12
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_13
#define LED_2_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_15
#define LED_3_GPIO_Port GPIOB
#define EC_R_A_Pin GPIO_PIN_6
#define EC_R_A_GPIO_Port GPIOC
#define EC_R_B_Pin GPIO_PIN_7
#define EC_R_B_GPIO_Port GPIOC
#define SR_CLK_Pin GPIO_PIN_8
#define SR_CLK_GPIO_Port GPIOA
#define LED_6_Pin GPIO_PIN_11
#define LED_6_GPIO_Port GPIOA
#define LED_5_Pin GPIO_PIN_12
#define LED_5_GPIO_Port GPIOA
#define SR_SER_Pin GPIO_PIN_15
#define SR_SER_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_2
#define SPI3_CS_GPIO_Port GPIOD
#define FAN_Pin GPIO_PIN_4
#define FAN_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_5
#define BUZZER_GPIO_Port GPIOB
#define EC_L_A_Pin GPIO_PIN_6
#define EC_L_A_GPIO_Port GPIOB
#define EC_L_B_Pin GPIO_PIN_7
#define EC_L_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
