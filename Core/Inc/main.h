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
#include "stm32l4xx_hal.h"

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
void set_reconfigure();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define INT_Pin GPIO_PIN_14
#define INT_GPIO_Port GPIOC
#define GPIO_1_Pin GPIO_PIN_13
#define GPIO_1_GPIO_Port GPIOC
#define TR1_EN_Pin GPIO_PIN_9
#define TR1_EN_GPIO_Port GPIOB
#define PDN_Pin GPIO_PIN_4
#define PDN_GPIO_Port GPIOB
#define REFSEL_Pin GPIO_PIN_3
#define REFSEL_GPIO_Port GPIOB
#define TX1_CS_Pin GPIO_PIN_15
#define TX1_CS_GPIO_Port GPIOA
#define TR4_EN_Pin GPIO_PIN_15
#define TR4_EN_GPIO_Port GPIOC
#define TR3_EN_Pin GPIO_PIN_8
#define TR3_EN_GPIO_Port GPIOB
#define SYSTEM_RDY_Pin GPIO_PIN_2
#define SYSTEM_RDY_GPIO_Port GPIOD
#define TX1_SHUTZ_Pin GPIO_PIN_11
#define TX1_SHUTZ_GPIO_Port GPIOC
#define CALL_OUT_Pin GPIO_PIN_10
#define CALL_OUT_GPIO_Port GPIOC
#define GLOBAL_SDA_Pin GPIO_PIN_7
#define GLOBAL_SDA_GPIO_Port GPIOB
#define HW_SW_CTRL_Pin GPIO_PIN_5
#define HW_SW_CTRL_GPIO_Port GPIOB
#define LD_HB_Pin GPIO_PIN_12
#define LD_HB_GPIO_Port GPIOC
#define GLOBAL_SCL_Pin GPIO_PIN_6
#define GLOBAL_SCL_GPIO_Port GPIOB
#define REF_CLK_Pin GPIO_PIN_8
#define REF_CLK_GPIO_Port GPIOA
#define RX_I2C_SDA_Pin GPIO_PIN_9
#define RX_I2C_SDA_GPIO_Port GPIOC
#define RX_I2C_SCL_Pin GPIO_PIN_0
#define RX_I2C_SCL_GPIO_Port GPIOC
#define RX_RDY_Pin GPIO_PIN_7
#define RX_RDY_GPIO_Port GPIOC
#define TX_RESET_L_Pin GPIO_PIN_8
#define TX_RESET_L_GPIO_Port GPIOC
#define THERMISTOR_Pin GPIO_PIN_2
#define THERMISTOR_GPIO_Port GPIOC
#define CALL_IN_Pin GPIO_PIN_2
#define CALL_IN_GPIO_Port GPIOA
#define SPI_SCK_Pin GPIO_PIN_5
#define SPI_SCK_GPIO_Port GPIOA
#define TX2_SHUTZ_Pin GPIO_PIN_0
#define TX2_SHUTZ_GPIO_Port GPIOB
#define TX_CW_EN_Pin GPIO_PIN_6
#define TX_CW_EN_GPIO_Port GPIOC
#define TRIGGER_Pin GPIO_PIN_15
#define TRIGGER_GPIO_Port GPIOB
#define POWER_GOOD_Pin GPIO_PIN_14
#define POWER_GOOD_GPIO_Port GPIOB
#define TX2_CS_Pin GPIO_PIN_0
#define TX2_CS_GPIO_Port GPIOA
#define TR8_EN_Pin GPIO_PIN_3
#define TR8_EN_GPIO_Port GPIOA
#define SPI_MISO_Pin GPIO_PIN_6
#define SPI_MISO_GPIO_Port GPIOA
#define TR2_EN_Pin GPIO_PIN_1
#define TR2_EN_GPIO_Port GPIOB
#define TR7_EN_Pin GPIO_PIN_2
#define TR7_EN_GPIO_Port GPIOB
#define LOCAL_SCL_Pin GPIO_PIN_10
#define LOCAL_SCL_GPIO_Port GPIOB
#define TR6_EN_Pin GPIO_PIN_13
#define TR6_EN_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_1
#define RST_GPIO_Port GPIOA
#define TX_STDBY_Pin GPIO_PIN_4
#define TX_STDBY_GPIO_Port GPIOA
#define SPI_MOSI_Pin GPIO_PIN_7
#define SPI_MOSI_GPIO_Port GPIOA
#define TR5_EN_Pin GPIO_PIN_4
#define TR5_EN_GPIO_Port GPIOC
#define RDY_Pin GPIO_PIN_5
#define RDY_GPIO_Port GPIOC
#define LOCAL_SDA_Pin GPIO_PIN_11
#define LOCAL_SDA_GPIO_Port GPIOB
#define EXT_Pin GPIO_PIN_12
#define EXT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MAX_FOUND_ADDRESSES 10 // Maximum number of found addresses

extern uint8_t found_addresses[]; // Global array to store found addresses
extern uint8_t found_address_count; // Counter for found addresses

extern CRC_HandleTypeDef hcrc;
extern SPI_HandleTypeDef hspi1;
// extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim1;
extern LPTIM_HandleTypeDef hlptim1;

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define CALL_OUT_UART huart3
#define CALL_IN_UART huart2
#define REF_CLK_TIMER htim1
#define REF_CLK_CHANNEL TIM_CHANNEL_1
#define CDC_TIMER htim7
#define TRIGGER_TIMER htim15
#define HIRES_TIMER htim2
#define LORES_TIMER htim1
#define RESET_TIMER hlptim1
// #define TEMPERATURE_TIMER htim7

extern I2C_HandleTypeDef* GLOBAL_I2C_DEVICE;
extern I2C_HandleTypeDef* LOCAL_I2C_DEVICE;

#define MAX31875_ADDRESS 0x48
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
