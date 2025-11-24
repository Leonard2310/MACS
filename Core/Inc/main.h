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
#include "stm32f3xx_hal.h"

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
#define PWM_Servo_Pin GPIO_PIN_0
#define PWM_Servo_GPIO_Port GPIOB
#define PIR2_Signal_Pin GPIO_PIN_2
#define PIR2_Signal_GPIO_Port GPIOB
#define PIR2_Signal_EXTI_IRQn EXTI2_TSC_IRQn
#define Blue_LED_Pin GPIO_PIN_8
#define Blue_LED_GPIO_Port GPIOE
#define Red_LED_Pin GPIO_PIN_9
#define Red_LED_GPIO_Port GPIOE
#define PWM_Servo2_Pin GPIO_PIN_10
#define PWM_Servo2_GPIO_Port GPIOB
#define PIR3_LED2_Pin GPIO_PIN_11
#define PIR3_LED2_GPIO_Port GPIOB
#define PIR4_signal_Pin GPIO_PIN_15
#define PIR4_signal_GPIO_Port GPIOB
#define PIR4_signal_EXTI_IRQn EXTI15_10_IRQn
#define PIR3_LED_Pin GPIO_PIN_3
#define PIR3_LED_GPIO_Port GPIOB
#define PIR_Signal_Pin GPIO_PIN_4
#define PIR_Signal_GPIO_Port GPIOB
#define PIR_Signal_EXTI_IRQn EXTI4_IRQn
#define Button_Pin GPIO_PIN_8
#define Button_GPIO_Port GPIOB
#define PIR3_Signal_Pin GPIO_PIN_9
#define PIR3_Signal_GPIO_Port GPIOB
#define PIR3_Signal_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
