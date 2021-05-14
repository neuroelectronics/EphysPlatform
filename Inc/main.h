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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f7xx_hal.h"

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
#define HJ_RST_Pin GPIO_PIN_3
#define HJ_RST_GPIO_Port GPIOE
#define HJ_WAKE_Pin GPIO_PIN_4
#define HJ_WAKE_GPIO_Port GPIOE
#define HJ_DRDY_Pin GPIO_PIN_5
#define HJ_DRDY_GPIO_Port GPIOE
#define HJ_CONFIG_Pin GPIO_PIN_13
#define HJ_CONFIG_GPIO_Port GPIOC
#define HJ_ROLE_Pin GPIO_PIN_14
#define HJ_ROLE_GPIO_Port GPIOC
#define HJ_STATE_Pin GPIO_PIN_15
#define HJ_STATE_GPIO_Port GPIOC
#define BLE_LED_Pin GPIO_PIN_6
#define BLE_LED_GPIO_Port GPIOF
#define LCD_TX_Pin GPIO_PIN_0
#define LCD_TX_GPIO_Port GPIOA
#define LCD_RX_Pin GPIO_PIN_1
#define LCD_RX_GPIO_Port GPIOA
#define IC2_TX_Pin GPIO_PIN_2
#define IC2_TX_GPIO_Port GPIOA
#define IC2_RX_Pin GPIO_PIN_3
#define IC2_RX_GPIO_Port GPIOA
#define LED_BUF_100_Pin GPIO_PIN_4
#define LED_BUF_100_GPIO_Port GPIOA
#define LED_BUF_75_Pin GPIO_PIN_4
#define LED_BUF_75_GPIO_Port GPIOC
#define LED_BUF_50_Pin GPIO_PIN_5
#define LED_BUF_50_GPIO_Port GPIOC
#define LED_BUF_25_Pin GPIO_PIN_0
#define LED_BUF_25_GPIO_Port GPIOB
#define NBL_0_Pin GPIO_PIN_1
#define NBL_0_GPIO_Port GPIOB
#define VEE_IC_EN_Pin GPIO_PIN_11
#define VEE_IC_EN_GPIO_Port GPIOB
#define VEE_EN_Pin GPIO_PIN_12
#define VEE_EN_GPIO_Port GPIOB
#define VCC_EN_Pin GPIO_PIN_13
#define VCC_EN_GPIO_Port GPIOB
#define IC3_TX_Pin GPIO_PIN_8
#define IC3_TX_GPIO_Port GPIOD
#define IC3_RX_Pin GPIO_PIN_9
#define IC3_RX_GPIO_Port GPIOD
#define MODE4_LED_Pin GPIO_PIN_10
#define MODE4_LED_GPIO_Port GPIOD
#define MODE3_LED_Pin GPIO_PIN_11
#define MODE3_LED_GPIO_Port GPIOD
#define MODE2_LED_Pin GPIO_PIN_12
#define MODE2_LED_GPIO_Port GPIOD
#define MODE1_LED_Pin GPIO_PIN_13
#define MODE1_LED_GPIO_Port GPIOD
#define IC4_TX_Pin GPIO_PIN_6
#define IC4_TX_GPIO_Port GPIOC
#define IC4_RX_Pin GPIO_PIN_7
#define IC4_RX_GPIO_Port GPIOC
#define IC1_TX_Pin GPIO_PIN_9
#define IC1_TX_GPIO_Port GPIOA
#define IC1_RX_Pin GPIO_PIN_10
#define IC1_RX_GPIO_Port GPIOA
#define SD_LED_Pin GPIO_PIN_6
#define SD_LED_GPIO_Port GPIOD
#define BLE_RX_Pin GPIO_PIN_0
#define BLE_RX_GPIO_Port GPIOE
#define BLE_TX_Pin GPIO_PIN_1
#define BLE_TX_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
