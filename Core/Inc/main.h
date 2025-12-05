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
#include "stm32h7xx_ll_adc.h"
#include "stm32h7xx_ll_bdma.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_i2c.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

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
#define GPIO_nLED_GREEN_Pin LL_GPIO_PIN_6
#define GPIO_nLED_GREEN_GPIO_Port GPIOI
#define GPIO_nLED_RED_Pin LL_GPIO_PIN_5
#define GPIO_nLED_RED_GPIO_Port GPIOI
#define SPI4_CS3_MS5611_CS_Pin LL_GPIO_PIN_10
#define SPI4_CS3_MS5611_CS_GPIO_Port GPIOG
#define GPIO_nLED_BLUE_Pin LL_GPIO_PIN_7
#define GPIO_nLED_BLUE_GPIO_Port GPIOI
#define SPI6_CS2_MS5611_Pin LL_GPIO_PIN_8
#define SPI6_CS2_MS5611_GPIO_Port GPIOI
#define HEATER_Pin LL_GPIO_PIN_8
#define HEATER_GPIO_Port GPIOA
#define SPI2_CS2_RM3100_Pin LL_GPIO_PIN_2
#define SPI2_CS2_RM3100_GPIO_Port GPIOF
#define SPI1_CS3_ICM20689_Pin LL_GPIO_PIN_6
#define SPI1_CS3_ICM20689_GPIO_Port GPIOG
#define SPI6_CS1_ICM20649_Pin LL_GPIO_PIN_12
#define SPI6_CS1_ICM20649_GPIO_Port GPIOI
#define SPI4_CS1_BMI088_A_Pin LL_GPIO_PIN_3
#define SPI4_CS1_BMI088_A_GPIO_Port GPIOF
#define SPI2_CS1_FRAM_Pin LL_GPIO_PIN_5
#define SPI2_CS1_FRAM_GPIO_Port GPIOF
#define SPI4_CS2_BMI088_G_Pin LL_GPIO_PIN_4
#define SPI4_CS2_BMI088_G_GPIO_Port GPIOF
#define SPI1_CS1_ADI16470_Pin LL_GPIO_PIN_10
#define SPI1_CS1_ADI16470_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
