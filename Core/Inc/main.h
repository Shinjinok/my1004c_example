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
#include "stm32l0xx_ll_crc.h"
#include "stm32l0xx_ll_i2c.h"
#include "stm32l0xx_ll_crs.h"
#include "stm32l0xx_ll_rcc.h"
#include "stm32l0xx_ll_bus.h"
#include "stm32l0xx_ll_system.h"
#include "stm32l0xx_ll_exti.h"
#include "stm32l0xx_ll_cortex.h"
#include "stm32l0xx_ll_utils.h"
#include "stm32l0xx_ll_pwr.h"
#include "stm32l0xx_ll_dma.h"
#include "stm32l0xx_ll_rtc.h"
#include "stm32l0xx_ll_spi.h"
#include "stm32l0xx_ll_usart.h"
#include "stm32l0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
#define WakeUpBtn_Pin LL_GPIO_PIN_0
#define WakeUpBtn_GPIO_Port GPIOA
#define WakeUpBtn_EXTI_IRQn EXTI0_1_IRQn
#define LED_Blue_Pin LL_GPIO_PIN_1
#define LED_Blue_GPIO_Port GPIOA
#define LIS3DH_IRQ_Pin LL_GPIO_PIN_2
#define LIS3DH_IRQ_GPIO_Port GPIOA
#define LIS3DH_IRQ_EXTI_IRQn EXTI2_3_IRQn
#define DW_IRQ_Pin LL_GPIO_PIN_3
#define DW_IRQ_GPIO_Port GPIOA
#define DW_IRQ_EXTI_IRQn EXTI2_3_IRQn
#define LIS3DH_PWR_Pin LL_GPIO_PIN_4
#define LIS3DH_PWR_GPIO_Port GPIOA
#define DW_CS_Pin LL_GPIO_PIN_0
#define DW_CS_GPIO_Port GPIOB
#define DW_RST_Pin LL_GPIO_PIN_1
#define DW_RST_GPIO_Port GPIOB
#define LED_Red_Pin LL_GPIO_PIN_8
#define LED_Red_GPIO_Port GPIOA
#define LIS3DH_CS_Pin LL_GPIO_PIN_15
#define LIS3DH_CS_GPIO_Port GPIOA
#define LED_Green_Pin LL_GPIO_PIN_3
#define LED_Green_GPIO_Port GPIOB
#define SyncGPIO_Pin LL_GPIO_PIN_4
#define SyncGPIO_GPIO_Port GPIOB
#define Button_Pin LL_GPIO_PIN_5
#define Button_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */
//extern I2C_HandleTypeDef hi2c1;
//extern SPI_HandleTypeDef hspi1;
//extern UART_HandleTypeDef huart2;

void _Error_Handler(char *file, int line);
void SystemClock_Config(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
