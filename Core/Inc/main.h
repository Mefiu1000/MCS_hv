/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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
void AIR1_AIR2_Current_Measurment(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_RED_Pin GPIO_PIN_0
#define LED_RED_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_1
#define LED_GREEN_GPIO_Port GPIOA
#define AIR1_CURRENT_SENSOR_Pin GPIO_PIN_6
#define AIR1_CURRENT_SENSOR_GPIO_Port GPIOA
#define AIR2_CURRENT_SENSOR_Pin GPIO_PIN_7
#define AIR2_CURRENT_SENSOR_GPIO_Port GPIOA
#define AIR2_ON_uC_Pin GPIO_PIN_12
#define AIR2_ON_uC_GPIO_Port GPIOB
#define AIR1_ON_uC_Pin GPIO_PIN_13
#define AIR1_ON_uC_GPIO_Port GPIOB
#define AIR2_STATUS_uC_Pin GPIO_PIN_14
#define AIR2_STATUS_uC_GPIO_Port GPIOB
#define AIR1_STATUS_uC_Pin GPIO_PIN_15
#define AIR1_STATUS_uC_GPIO_Port GPIOB
#define IMD_M_LS_uC_Pin GPIO_PIN_8
#define IMD_M_LS_uC_GPIO_Port GPIOA
#define Precharge_ON_Pin GPIO_PIN_3
#define Precharge_ON_GPIO_Port GPIOB
#define IMD_STATUS_uC_Pin GPIO_PIN_7
#define IMD_STATUS_uC_GPIO_Port GPIOB
#define BMS_STATUS_uC_Pin GPIO_PIN_11
#define BMS_STATUS_uC_GPIO_Port GPIOF
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
