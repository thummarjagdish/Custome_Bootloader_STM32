/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USER_APPLICATION_ADDRESS		0x800A000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define BL_RCV_BUF_LENGTH		300
uint8_t bl_rcv_buffer[BL_RCV_BUF_LENGTH];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_UART4_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void bootloader_uart_readdata(void);
void bootloader_uart_write_data(uint8_t *pTxBuff,uint16_t txlen);

void booloader_to_jump_application(void);
void bootloader_handle_getver_cmd(uint8_t *buff);
void bootloader_handle_gethelp_cmd(uint8_t *buff);
void bootloader_handle_getcid_cmd(uint8_t *buff);
void bootloader_handle_getrdpstatus_cmd(uint8_t *buff);
void bootloader_handle_gotoaddress_cmd(uint8_t *buff);
void bootloader_handle_flasherase_cmd(uint8_t *buff);
void bootloader_handle_memwrite_cmd(uint8_t *buff);
void bootloader_handle_enableRWprotect_cmd(uint8_t *buff);
void bootloader_handle_memread_cmd(uint8_t *buff);
void bootloader_handle_sectorstatus_cmd(uint8_t *buff);
void bootloader_handle_otpread_cmd(uint8_t *buff);
void bootloader_handle_disableRWprotect_cmd(uint8_t *buff);

void bootloader_send_ack(uint8_t command,uint8_t follow_len);
void bootloader_send_nack(void);
uint8_t bootloader_crc_verify(volatile uint8_t *pBuff,uint32_t bufflength,uint32_t crc_host);


uint8_t get_bootloader_version(void);
uint16_t get_mcu_chip_id(void);
uint8_t get_rdp_status_of_flash(void);
uint8_t execute_flash_erase(uint8_t,uint8_t);
uint8_t executed_mem_write(volatile uint8_t*,volatile uint32_t,volatile uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char  somedata[] = "Hello from bootloader\r\n";
	uint8_t pinStatus = 0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CRC_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart2, (uint8_t *)somedata,sizeof(somedata), HAL_MAX_DELAY);
  pinStatus = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);
  HAL_Delay(500);
  //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
  if(pinStatus == GPIO_PIN_SET)
  {
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_SET);
	  bootloader_uart_readdata();
  }
  else
  {
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_2,GPIO_PIN_RESET);
	  booloader_to_jump_application();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_UART4;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_WORDS;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void bootloader_uart_readdata(void)
{
	uint16_t received_length = 0;

	while(1)
	{
		memset(bl_rcv_buffer,0,BL_RCV_BUF_LENGTH);
		HAL_UART_Receive(&huart2,(uint8_t *)bl_rcv_buffer,1,HAL_MAX_DELAY);
		received_length = bl_rcv_buffer[0];
		HAL_UART_Receive(&huart2,(uint8_t *)&bl_rcv_buffer[1],received_length,HAL_MAX_DELAY);

		switch(bl_rcv_buffer[1])
		{
		case BL_GET_VER:
			bootloader_handle_getver_cmd(bl_rcv_buffer);
			break;
		case BL_GET_HELP:
			bootloader_handle_gethelp_cmd(bl_rcv_buffer);
			break;
		case BL_GET_CID:
			bootloader_handle_getcid_cmd(bl_rcv_buffer);
			break;
		case BL_GET_RDP_STATUS:
			bootloader_handle_getrdpstatus_cmd(bl_rcv_buffer);
			break;
		case BL_GO_TO_ADDR:
			bootloader_handle_gotoaddress_cmd(bl_rcv_buffer);
			break;
		case BL_FLASH_ERASE:
			bootloader_handle_flasherase_cmd(bl_rcv_buffer);
			break;
		case BL_MEM_WRITE:
			bootloader_handle_memwrite_cmd(bl_rcv_buffer);
			break;
		case BL_EN_R_W_PROTECT:
			bootloader_handle_enableRWprotect_cmd(bl_rcv_buffer);
			break;
		case BL_MEM_READ:
			bootloader_handle_memread_cmd(bl_rcv_buffer);
			break;
		case BL_READ_SECTOR_STATUS:
			bootloader_handle_sectorstatus_cmd(bl_rcv_buffer);
			break;
		case BL_OTP_READ:
			bootloader_handle_otpread_cmd(bl_rcv_buffer);
			break;
		case BL_DIS_R_W_PROTECT:
			bootloader_handle_disableRWprotect_cmd(bl_rcv_buffer);
			break;

		default:
			Error_Handler();
			break;

		}
	}


}

void bootloader_uart_write_data(uint8_t *pTxBuff,uint16_t txlen)
{
	HAL_UART_Transmit(&huart2,pTxBuff,txlen,HAL_MAX_DELAY);
}

void bootloader_send_ack(uint8_t command,uint8_t follow_len)
{
	uint8_t ack_buff[2];
	ack_buff[0] = BL_ACK;
	ack_buff[1] = follow_len;

	HAL_UART_Transmit(&huart2,ack_buff,2,HAL_MAX_DELAY);

}

void bootloader_send_nack(void)
{
	uint8_t send_nack = BL_NACK;

	HAL_UART_Transmit(&huart2,&send_nack,1,HAL_MAX_DELAY);
}

uint8_t bootloader_crc_verify(volatile uint8_t *pBuff,uint32_t bufflength,uint32_t crc_host)
{
	volatile uint32_t uwCRCValue = 0xFF;
	volatile uint32_t BuffData = 0;
	volatile uint8_t crc_status = VERIFY_CRC_FAILED;
	volatile uint32_t len = bufflength;
	volatile uint32_t rcv_crc = crc_host;

	for(uint32_t i=0;i<len;i++)
	{
		BuffData = pBuff[i];
		uwCRCValue = HAL_CRC_Accumulate(&hcrc,&BuffData,1);
	}
	__HAL_CRC_DR_RESET(&hcrc);
	if(uwCRCValue == rcv_crc)
	{
		crc_status = VERIFY_CRC_SUCCESS;
	}

	return crc_status;
}

void bootloader_handle_getver_cmd(uint8_t *bl_rcv_buff)
{
	uint8_t bl_version = 0;

	uint32_t command_packet_length = bl_rcv_buff[0] + 1;

	uint32_t crc_host = *((uint32_t *)(bl_rcv_buff + command_packet_length -4));

	if(!bootloader_crc_verify(bl_rcv_buff,command_packet_length -4,crc_host))
	{
		bootloader_send_ack(bl_rcv_buff[1],1);
		bl_version = get_bootloader_version();

		bootloader_uart_write_data(&bl_version,1);



	}
	else
	{
		bootloader_send_nack();
	}


}
void bootloader_handle_gethelp_cmd(uint8_t *bl_rcv_buff)
{
	uint8_t bootaloader_commands[] = {BL_GET_VER,
			BL_GET_HELP		    ,
			BL_GET_CID		    ,
			BL_GET_RDP_STATUS   ,
			BL_GO_TO_ADDR	    ,
			BL_FLASH_ERASE	    ,
			BL_MEM_WRITE	    ,
			BL_EN_R_W_PROTECT   ,
			BL_MEM_READ		    ,
			BL_READ_SECTOR_STATUS,
			BL_OTP_READ		    ,
			BL_DIS_R_W_PROTECT };

	uint32_t command_packet_length = bl_rcv_buff[0] + 1;

	uint32_t crc_host = *((uint32_t *)(bl_rcv_buff + command_packet_length -4));

	if(!bootloader_crc_verify(bl_rcv_buff,command_packet_length -4,crc_host))
	{
		bootloader_send_ack(bl_rcv_buff[1],sizeof(bootaloader_commands));
		bootloader_uart_write_data(bootaloader_commands,sizeof(bootaloader_commands));

	}
	else
	{
		bootloader_send_nack();
	}
}
void bootloader_handle_getcid_cmd(uint8_t *bl_rcv_buff)
{

	uint16_t chip_identification = 0;
	uint8_t cid_buf[2];

	uint32_t command_packet_length = bl_rcv_buff[0] + 1;

	uint32_t crc_host = *((uint32_t *)(bl_rcv_buff + command_packet_length -4));

	if(!bootloader_crc_verify(bl_rcv_buff,command_packet_length -4,crc_host))
	{
		bootloader_send_ack(bl_rcv_buff[1],sizeof(chip_identification));
		chip_identification = get_mcu_chip_id();
		cid_buf[0] = chip_identification & 0xFF;
		cid_buf[1] = (chip_identification >> 8) & 0xFF;
		bootloader_uart_write_data(cid_buf,sizeof(cid_buf));

	}
	else
	{
		bootloader_send_nack();
	}

}
void bootloader_handle_getrdpstatus_cmd(uint8_t *bl_rcv_buff)
{
	uint8_t rdp_status = 0;

	uint32_t command_packet_length = bl_rcv_buff[0] + 1;

	uint32_t crc_host = *((uint32_t *)(bl_rcv_buff + command_packet_length -4));

	if(!bootloader_crc_verify(bl_rcv_buff,command_packet_length -4,crc_host))
	{
		bootloader_send_ack(bl_rcv_buff[1],sizeof(rdp_status));
		rdp_status = get_rdp_status_of_flash();
		bootloader_uart_write_data(&rdp_status,sizeof(rdp_status));

	}
	else
	{
		bootloader_send_nack();
	}
}
void bootloader_handle_gotoaddress_cmd(uint8_t *bl_rcv_buff)
{
	uint8_t address_status = ADDR_INVALID;
	void (*jump_to_app)(void);

	uint32_t goto_address;

	uint32_t command_packet_length = bl_rcv_buff[0] + 1;

	goto_address = *(uint32_t *)(bl_rcv_buff + 2);

	uint32_t crc_host = *((uint32_t *)(bl_rcv_buff + command_packet_length -4));

	if(!bootloader_crc_verify(bl_rcv_buff,command_packet_length -4,crc_host))
	{
		bootloader_send_ack(bl_rcv_buff[1],1);
		if(goto_address == 0x0800c265)
		{
			address_status = ADDR_VALID;
			bootloader_uart_write_data(&address_status,sizeof(address_status));

			jump_to_app = (void*)(goto_address);

			jump_to_app();


		}
		else
		{
			address_status = ADDR_INVALID;
			bootloader_uart_write_data(&address_status,sizeof(address_status));
		}
	}
	else
	{
		bootloader_send_nack();
	}
}
void bootloader_handle_flasherase_cmd(uint8_t *bl_rcv_buff)
{

	uint8_t erase_status;

	uint32_t command_packet_length = bl_rcv_buff[0] + 1;

	uint32_t crc_host = *((uint32_t *)(bl_rcv_buff + command_packet_length -4));

	if(!bootloader_crc_verify(bl_rcv_buff,command_packet_length -4,crc_host))
	{
		erase_status = execute_flash_erase(bl_rcv_buff[2],bl_rcv_buff[3]);
		bootloader_uart_write_data(&erase_status,sizeof(erase_status));

	}
	else
	{
		bootloader_send_nack();
	}
}
void bootloader_handle_memwrite_cmd(uint8_t *bl_rcv_buff)
{
	uint8_t valid_address = ADDR_VALID;
	uint8_t len = 0,checksum = 0;
	uint32_t mem_address;
	uint8_t write_status = 0;
	uint32_t command_packet_length = bl_rcv_buff[0] + 1;
	uint8_t payload_len = bl_rcv_buff[6];

	mem_address = *(uint32_t *)(bl_rcv_buff + 2);
	len = bl_rcv_buff[0];
	checksum = bl_rcv_buff[len];
	uint32_t crc_host = *((uint32_t *)(bl_rcv_buff + command_packet_length -4));

	if(!bootloader_crc_verify(bl_rcv_buff,command_packet_length -4,crc_host))
	{
		bootloader_send_ack(bl_rcv_buff[1],1);
		if(1)
		{
			valid_address = ADDR_VALID;
			write_status = executed_mem_write(&bl_rcv_buff[7],mem_address,payload_len);
			bootloader_uart_write_data(&valid_address,sizeof(valid_address));
		}
	}
	else
	{
		bootloader_send_nack();
	}
}
void bootloader_handle_enableRWprotect_cmd(uint8_t *bl_rcv_buff)
{

}
void bootloader_handle_memread_cmd(uint8_t *bl_rcv_buff)
{

}
void bootloader_handle_sectorstatus_cmd(uint8_t *bl_rcv_buff)
{

}
void bootloader_handle_otpread_cmd(uint8_t *bl_rcv_buff)
{

}
void bootloader_handle_disableRWprotect_cmd(uint8_t *bl_rcv_buff)
{

}

void booloader_to_jump_application(void)
{

	void (*jump_to_app)(void);
	uint32_t MSPvalue = 0;
	uint32_t resethandler_address = 0;

	MSPvalue = *(volatile uint32_t *)(0x08000000UL + 0xA000);
	__set_MSP(MSPvalue);
	resethandler_address = *((volatile uint32_t *)(0x08000000UL + 0xA000 + 0x04));
	jump_to_app = (void*)resethandler_address;

	jump_to_app();

}

uint16_t get_mcu_chip_id(void)
{
	uint16_t dev_id = 0;

	dev_id = (DBGMCU->IDCODE & 0xFFF);

	return dev_id;
}


uint8_t get_rdp_status_of_flash(void)
{
#if 0
	volatile uint32_t *OBaddr = (volatile uint32_t *)0x1FFF7800;
	uint8_t rdp_status = *OBaddr & 0xFF;

	return rdp_status;
#else if
	uint32_t rdp_status = 0;
	FLASH_OBProgramInitTypeDef Obstatus;
	HAL_FLASHEx_OBGetConfig(&Obstatus);
	return (uint8_t)Obstatus.RDPLevel;
#endif
}

uint8_t get_bootloader_version(void)
{
	return BL_VERSION;
}

uint8_t execute_flash_erase(uint8_t pagenumber,uint8_t numberofpages)
{
	FLASH_EraseInitTypeDef flashhandle;

	uint32_t error = 0;
	uint8_t status = 0;

	memset(&flashhandle,0,sizeof(flashhandle));
	if(pagenumber == 0xFF)
	{
		flashhandle.Banks = FLASH_BANK_1;
		flashhandle.TypeErase = FLASH_TYPEERASE_MASSERASE;
		flashhandle.Page = pagenumber;
		flashhandle.NbPages = numberofpages;
	}
	else
	{
		flashhandle.Banks = FLASH_BANK_1;
		flashhandle.TypeErase = FLASH_TYPEERASE_PAGES;
		flashhandle.Page = pagenumber;
		flashhandle.NbPages = numberofpages;
	}
	HAL_FLASH_Unlock();
	status = HAL_FLASHEx_Erase(&flashhandle,&error);
	HAL_FLASH_Lock();
	return status;
}


uint8_t executed_mem_write(volatile uint8_t* buff, uint32_t address, uint8_t len)
{
	uint8_t write_status = 0;
	uint32_t add = 0;
	uint64_t data=0;
	HAL_FLASH_Unlock();

	for(uint32_t i=0;i<len;i=i+8)
	{
/*		data |= buff[i];
		data |= (buff[i+1] << 8);
		data |= (buff[i+2] << 16);
		data |= (buff[i+3] << 24);
		data |= (buff[i+4] << 32);
		data |= (buff[i+5] << 40);
		data |= (buff[i+6] << 48);
		data |= (buff[i+7] << 56);*/

		data = (*((uint32_t *)(buff+i)));
		data |= (uint64_t)(*((uint32_t *)(buff+i+4))) << 32;
		add = address+i;
		write_status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,add,data);
	}

	HAL_FLASH_Lock();
	return write_status;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
