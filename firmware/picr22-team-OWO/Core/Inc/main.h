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
#include "stm32g4xx_hal.h"

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
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOF
#define INFR_Pin GPIO_PIN_1
#define INFR_GPIO_Port GPIOF
#define TMPWM_Pin GPIO_PIN_0
#define TMPWM_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define S1PWM_Pin GPIO_PIN_2
#define S1PWM_GPIO_Port GPIOA
#define S2PWM_Pin GPIO_PIN_3
#define S2PWM_GPIO_Port GPIOA
#define ENC31_Pin GPIO_PIN_4
#define ENC31_GPIO_Port GPIOA
#define M2D_Pin GPIO_PIN_5
#define M2D_GPIO_Port GPIOA
#define ENC32_Pin GPIO_PIN_6
#define ENC32_GPIO_Port GPIOA
#define M1D_Pin GPIO_PIN_7
#define M1D_GPIO_Port GPIOA
#define M1PWM_Pin GPIO_PIN_0
#define M1PWM_GPIO_Port GPIOB
#define ENC21_Pin GPIO_PIN_8
#define ENC21_GPIO_Port GPIOA
#define ENC22_Pin GPIO_PIN_9
#define ENC22_GPIO_Port GPIOA
#define M2PWM_Pin GPIO_PIN_15
#define M2PWM_GPIO_Port GPIOA
#define MSLEEP_Pin GPIO_PIN_3
#define MSLEEP_GPIO_Port GPIOB
#define M3D_Pin GPIO_PIN_4
#define M3D_GPIO_Port GPIOB
#define M3PWM_Pin GPIO_PIN_5
#define M3PWM_GPIO_Port GPIOB
#define ENC11_Pin GPIO_PIN_6
#define ENC11_GPIO_Port GPIOB
#define ENC12_Pin GPIO_PIN_7
#define ENC12_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
