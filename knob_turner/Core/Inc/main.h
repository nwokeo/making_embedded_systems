/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define lcd_rw_Pin GPIO_PIN_2
#define lcd_rw_GPIO_Port GPIOE
#define lcd_e_Pin GPIO_PIN_3
#define lcd_e_GPIO_Port GPIOE
#define lcd_d4_Pin GPIO_PIN_4
#define lcd_d4_GPIO_Port GPIOE
#define lcd_d5_Pin GPIO_PIN_5
#define lcd_d5_GPIO_Port GPIOE
#define lcd_d6_Pin GPIO_PIN_6
#define lcd_d6_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define stp1_Pin GPIO_PIN_1
#define stp1_GPIO_Port GPIOA
#define stp2_Pin GPIO_PIN_2
#define stp2_GPIO_Port GPIOA
#define stp3_Pin GPIO_PIN_3
#define stp3_GPIO_Port GPIOA
#define stp4_Pin GPIO_PIN_4
#define stp4_GPIO_Port GPIOF
#define button2_Pin GPIO_PIN_1
#define button2_GPIO_Port GPIOB
#define button2_EXTI_IRQn EXTI1_IRQn
#define lcd_d7_Pin GPIO_PIN_7
#define lcd_d7_GPIO_Port GPIOE
#define DM_Pin GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define button1_Pin GPIO_PIN_0
#define button1_GPIO_Port GPIOD
#define button1_EXTI_IRQn EXTI0_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define lcd_rs_Pin GPIO_PIN_1
#define lcd_rs_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
