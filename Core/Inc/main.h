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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define WRIST_PLUS_BTN_Pin GPIO_PIN_5
#define WRIST_PLUS_BTN_GPIO_Port GPIOC
#define PCA9685_I2C_SCL_Pin GPIO_PIN_10
#define PCA9685_I2C_SCL_GPIO_Port GPIOB
#define SHOULDER_MINUS_BTN_Pin GPIO_PIN_12
#define SHOULDER_MINUS_BTN_GPIO_Port GPIOB
#define SHOULDER_PLUS_BTN_Pin GPIO_PIN_15
#define SHOULDER_PLUS_BTN_GPIO_Port GPIOB
#define WRIST_ROTATE_MINUS_BTN_Pin GPIO_PIN_6
#define WRIST_ROTATE_MINUS_BTN_GPIO_Port GPIOC
#define WRIST_ROTATE_PLUS_BTN_Pin GPIO_PIN_8
#define WRIST_ROTATE_PLUS_BTN_GPIO_Port GPIOC
#define GRIPPER_BTN_Pin GPIO_PIN_9
#define GRIPPER_BTN_GPIO_Port GPIOC
#define ELBOW_MINUS_BTN_Pin GPIO_PIN_11
#define ELBOW_MINUS_BTN_GPIO_Port GPIOA
#define ELBOW_PLUS_BTN_Pin GPIO_PIN_12
#define ELBOW_PLUS_BTN_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define PCA9685_I2C_SDA_Pin GPIO_PIN_12
#define PCA9685_I2C_SDA_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define WRIST_MINUS_BTN_Pin GPIO_PIN_9
#define WRIST_MINUS_BTN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
