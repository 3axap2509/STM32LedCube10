/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "time.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef int8_t byte;
typedef uint8_t ubyte;
typedef uint16_t ushort;

struct Point2Struct
{
	byte x;
	byte y;
};
struct Point2dStruct
{
	double x;
	double y;
};
struct Point3Struct
{
	byte x;
	byte y;
	byte z;
};
typedef struct Point3Struct Point3;
typedef struct Point2Struct Point2;
typedef struct Point2dStruct Point2d;

typedef enum
{
	figure2dOrientationXY = 0,
	figure2dOrientationYZ = 1,
	figure2dOrientationXZ = 2
}figure2dOrientation;
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
void Ping_Latch();
void Render_Square();
void Flying_Square();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Controll_Button_Pin GPIO_PIN_6
#define Controll_Button_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
