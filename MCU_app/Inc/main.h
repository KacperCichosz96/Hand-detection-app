/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define PIN_sleep_Pin GPIO_PIN_0
#define PIN_sleep_GPIO_Port GPIOA
#define PIN_sleep_EXTI_IRQn EXTI0_IRQn
#define LED_green_Pin GPIO_PIN_12
#define LED_green_GPIO_Port GPIOD
#define LED_orange_Pin GPIO_PIN_13
#define LED_orange_GPIO_Port GPIOD
#define LED_red_Pin GPIO_PIN_14
#define LED_red_GPIO_Port GPIOD
#define LED_blue_Pin GPIO_PIN_15
#define LED_blue_GPIO_Port GPIOD
#define Motor_STEP_Pin GPIO_PIN_2
#define Motor_STEP_GPIO_Port GPIOD
#define Motor_DIR_Pin GPIO_PIN_4
#define Motor_DIR_GPIO_Port GPIOD
#define Motor_SLEEP_Pin GPIO_PIN_6
#define Motor_SLEEP_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
