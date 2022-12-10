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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Dead_man_SW_Pin GPIO_PIN_0
#define Dead_man_SW_GPIO_Port GPIOC
#define RC_PWM2_Pin GPIO_PIN_0
#define RC_PWM2_GPIO_Port GPIOA
#define IMotor_Pin GPIO_PIN_1
#define IMotor_GPIO_Port GPIOA
#define UBatt_Pin GPIO_PIN_4
#define UBatt_GPIO_Port GPIOA
#define RC_PWM_Pin GPIO_PIN_6
#define RC_PWM_GPIO_Port GPIOA
#define DRV_PWR_FB_Pin GPIO_PIN_4
#define DRV_PWR_FB_GPIO_Port GPIOC
#define I2C_INT_Pin GPIO_PIN_5
#define I2C_INT_GPIO_Port GPIOC
#define ToF_XSDN_36_Pin GPIO_PIN_0
#define ToF_XSDN_36_GPIO_Port GPIOB
#define ToF_XSDN_25_Pin GPIO_PIN_1
#define ToF_XSDN_25_GPIO_Port GPIOB
#define ToF_XSDN_14_Pin GPIO_PIN_2
#define ToF_XSDN_14_GPIO_Port GPIOB
#define AD_CS4_Pin GPIO_PIN_12
#define AD_CS4_GPIO_Port GPIOB
#define AD_CS3_Pin GPIO_PIN_14
#define AD_CS3_GPIO_Port GPIOB
#define AD_CS2_Pin GPIO_PIN_15
#define AD_CS2_GPIO_Port GPIOB
#define RADIO_RX_Pin GPIO_PIN_7
#define RADIO_RX_GPIO_Port GPIOC
#define AD_CS1_Pin GPIO_PIN_8
#define AD_CS1_GPIO_Port GPIOC
#define INF_LE_Pin GPIO_PIN_9
#define INF_LE_GPIO_Port GPIOA
#define INF_OE_Pin GPIO_PIN_10
#define INF_OE_GPIO_Port GPIOA
#define DRV_EN_FB_Pin GPIO_PIN_11
#define DRV_EN_FB_GPIO_Port GPIOA
#define DRV_EN_Pin GPIO_PIN_12
#define DRV_EN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LED_LE_Pin GPIO_PIN_11
#define LED_LE_GPIO_Port GPIOC
#define LED_OE_Pin GPIO_PIN_2
#define LED_OE_GPIO_Port GPIOD
#define DRV_PMW1_Pin GPIO_PIN_5
#define DRV_PMW1_GPIO_Port GPIOB
#define DRV_PWM2_Pin GPIO_PIN_6
#define DRV_PWM2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
