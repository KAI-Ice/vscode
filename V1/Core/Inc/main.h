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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CTP_INT_Pin GPIO_PIN_2
#define CTP_INT_GPIO_Port GPIOE
#define LED_D4_Pin GPIO_PIN_3
#define LED_D4_GPIO_Port GPIOC
#define Car_E1B_Pin GPIO_PIN_1
#define Car_E1B_GPIO_Port GPIOA
#define Car_Battery_Pin GPIO_PIN_3
#define Car_Battery_GPIO_Port GPIOA
#define Car_E1A_Pin GPIO_PIN_5
#define Car_E1A_GPIO_Port GPIOA
#define Car_E2B_Pin GPIO_PIN_6
#define Car_E2B_GPIO_Port GPIOA
#define Car_E2A_Pin GPIO_PIN_7
#define Car_E2A_GPIO_Port GPIOA
#define LCD_RST_Pin GPIO_PIN_8
#define LCD_RST_GPIO_Port GPIOE
#define LCD_LED_Pin GPIO_PIN_9
#define LCD_LED_GPIO_Port GPIOE
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_DC_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_11
#define LCD_CS_GPIO_Port GPIOE
#define Car_AIN1_Pin GPIO_PIN_12
#define Car_AIN1_GPIO_Port GPIOE
#define Car_AIN2_Pin GPIO_PIN_13
#define Car_AIN2_GPIO_Port GPIOE
#define Car_BIN1_Pin GPIO_PIN_14
#define Car_BIN1_GPIO_Port GPIOE
#define Car_BIN2_Pin GPIO_PIN_15
#define Car_BIN2_GPIO_Port GPIOE
#define CTP_SCL_Pin GPIO_PIN_10
#define CTP_SCL_GPIO_Port GPIOB
#define CTP_SDA_Pin GPIO_PIN_11
#define CTP_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOC
#define CTP_RST_Pin GPIO_PIN_15
#define CTP_RST_GPIO_Port GPIOG
#define LCD_SCK_Pin GPIO_PIN_3
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_MISO_Pin GPIO_PIN_4
#define LCD_MISO_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_5
#define LCD_MOSI_GPIO_Port GPIOB
#define Car_STBY_Pin GPIO_PIN_6
#define Car_STBY_GPIO_Port GPIOB
#define Car_PWMA_Pin GPIO_PIN_8
#define Car_PWMA_GPIO_Port GPIOB
#define Car_PWMB_Pin GPIO_PIN_9
#define Car_PWMB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
