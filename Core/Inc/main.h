/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define CAM1_PWDWN_Pin GPIO_PIN_13
#define CAM1_PWDWN_GPIO_Port GPIOC
#define CAM2_RST_Pin GPIO_PIN_14
#define CAM2_RST_GPIO_Port GPIOC
#define CAM2_PWDWN_Pin GPIO_PIN_15
#define CAM2_PWDWN_GPIO_Port GPIOC
#define BUTTON_1_Pin GPIO_PIN_11
#define BUTTON_1_GPIO_Port GPIOE
#define BUTTON_2_Pin GPIO_PIN_15
#define BUTTON_2_GPIO_Port GPIOE
#define LED_4_Pin GPIO_PIN_8
#define LED_4_GPIO_Port GPIOD
#define PIR_1_Pin GPIO_PIN_10
#define PIR_1_GPIO_Port GPIOD
#define PIR_1_EXTI_IRQn EXTI15_10_IRQn
#define PIR_2_Pin GPIO_PIN_11
#define PIR_2_GPIO_Port GPIOD
#define PIR_2_EXTI_IRQn EXTI15_10_IRQn
#define PIR_3_Pin GPIO_PIN_14
#define PIR_3_GPIO_Port GPIOD
#define PIR_3_EXTI_IRQn EXTI15_10_IRQn
#define PIR_4_Pin GPIO_PIN_15
#define PIR_4_GPIO_Port GPIOD
#define PIR_4_EXTI_IRQn EXTI15_10_IRQn
#define CAM1_RST_Pin GPIO_PIN_12
#define CAM1_RST_GPIO_Port GPIOC
#define MSIZI_TRIGGER_Pin GPIO_PIN_0
#define MSIZI_TRIGGER_GPIO_Port GPIOD
#define MILO_TRIGGER_Pin GPIO_PIN_1
#define MILO_TRIGGER_GPIO_Port GPIOD
#define LED_1_Pin GPIO_PIN_5
#define LED_1_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_6
#define LED_2_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_7
#define LED_3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
