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
#define BL_GET_VER					0x51
#define BL_GET_HELP					0x52
#define BL_GET_CID					0x53
#define BL_GET_RDP_STATUS			0x54
#define BL_GO_TO_ADDR				0x55
#define BL_FLASH_ERASE				0x56
#define BL_MEM_WRITE				0x57
#define BL_EN_R_W_PROTECT			0x58
#define BL_MEM_READ					0x59
#define BL_READ_SECTOR_STATUS		0x5A
#define BL_OTP_READ					0x5B
#define BL_DIS_R_W_PROTECT			0x5C



#define BL_ACK				0xA5
#define BL_NACK				0x7F

#define VERIFY_CRC_SUCCESS			0
#define VERIFY_CRC_FAILED			1

#define ADDR_VALID					0
#define ADDR_INVALID				1


#define BL_VERSION		0x20
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
