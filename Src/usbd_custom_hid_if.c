/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v2.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
	0x06, 0x00, 0xFF, // USAGE_PAGE (custom)
	0x09, 0x01, // USAGE (custom)
	
	0xA1, 0x01, // COLLECTION (Application)
	// The Input report 
	0x09,0x03, // USAGE ID - Vendor defined 
	0x15,0x00, // LOGICAL_MINIMUM (0) 
	0x26,0x00, 0xFF, // LOGICAL_MAXIMUM (255) 
	0x75,0x08, // REPORT_SIZE (8bit) 
	0x95,0x40, // REPORT_COUNT (64Byte) 
	0x81,0x02, // INPUT (Data,Var,Abs) 

	// The Output report 
	0x09,0x04, // USAGE ID - Vendor defined 
	0x15,0x00, // LOGICAL_MINIMUM (0) 
	0x26,0x00,0xFF, // LOGICAL_MAXIMUM (255) 
	0x75,0x08, // REPORT_SIZE (8bit) 
	0x95,0x40, // REPORT_COUNT (64Byte) 
	0x91,0x02, // OUTPUT (Data,Var,Abs) 
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */
static uint8_t ProcessBuffer[64];
static uint8_t USB_Receive_Flag;
static uint32_t RedLEDDelay = 0;
static uint32_t YellowDelay = 0;
/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len);


/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  USB_Receive_Flag = 0;
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
  USBD_CUSTOM_HID_HandleTypeDef     *hhid;
  hhid = (USBD_CUSTOM_HID_HandleTypeDef*) hUsbDeviceFS.pClassData;
  memcpy(ProcessBuffer, hhid->Report_buf, 64);
  USB_Receive_Flag = 1;
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
#define IIC_SPEED_400K 0

#define IIC_SCL_PORT			GPIOA
#define IIC_SCL_PIN				GPIO_PIN_3
#define IIC_SDA_PORT			GPIOA
#define IIC_SDA_PIN				GPIO_PIN_2

static void _SCL_Delay_Short(void)
{
#if IIC_SPEED_400K
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
#else
	volatile uint32_t delay;
	for (delay = 0; delay < 5;delay++)
	{
		__NOP();
	}
#endif
}

static void _SCL_Delay(void)
{
#if IIC_SPEED_400K
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
	__NOP();
#else
	volatile uint32_t delay;
	for (delay = 0; delay < 30;delay++)
	{
		__NOP();
	}
#endif
}

static inline void _SetSDAInput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = IIC_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);
}
static inline void _SetSDAOutput(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = IIC_SDA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; 
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(IIC_SDA_PORT, &GPIO_InitStruct);
}

static void _SWI2C_Start(void)
{
	HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_RESET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);
	_SCL_Delay();
}

static void _SWI2C_Stop(void)
{
	_SetSDAOutput();
	HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_RESET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
}

static HAL_StatusTypeDef _SWI2C_WriteByte(uint8_t Value)
{
	uint8_t count;
	
	for (count = 0;count < 8;count++)
	{
		_SCL_Delay_Short();
		if (Value&0x80)
		{
			HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_RESET);
		}
		_SCL_Delay_Short();
		HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
		_SCL_Delay();
		HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);
		Value = Value<<1;
	}
	HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET);
	_SetSDAInput();
	HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
	for (count = 0;count < 20;count++)
	{
		if (HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN) == 0)
		{
			HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);
			_SetSDAOutput();
			return HAL_OK;
		}
	}
	HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);	
	_SetSDAOutput();
	return HAL_ERROR;
}

static uint8_t _SWI2C_ReadByte(uint8_t SendAck)
{
	uint8_t count, read, value = 0;

	_SetSDAInput();
	for (count = 0;count < 8;count++)
	{
		_SCL_Delay_Short();
		HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
		_SCL_Delay();
		if (HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN))
		{
			read = 1;
		}
		else
		{
			read = 0;
		}
		value = (value<<1)|read;
		HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);
		_SCL_Delay_Short();
	}	
	if (SendAck)
	{
		_SetSDAOutput();
		HAL_GPIO_WritePin(IIC_SDA_PORT,IIC_SDA_PIN, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET);
	_SCL_Delay();
	HAL_GPIO_WritePin(IIC_SCL_PORT,IIC_SCL_PIN, GPIO_PIN_RESET);

	return value;
}

static HAL_StatusTypeDef SWI2C_Read(uint8_t Addr, uint8_t SubAddr, uint8_t * pData, uint8_t Length)
{
	uint8_t i;
	HAL_StatusTypeDef result;
	
	_SWI2C_Start();

	result = _SWI2C_WriteByte(Addr);
	if (result != HAL_OK)
	{
		_SWI2C_Stop();
		return result;
	}

	result = _SWI2C_WriteByte(SubAddr);
	if (result != HAL_OK)
	{
		_SWI2C_Stop();
		return result;
	}

	_SWI2C_Start(); // restart
	
	result = _SWI2C_WriteByte(Addr|0x01);
	if (result != HAL_OK)
	{
		_SWI2C_Stop();
		return result;
	}
	
	for (i = 0; i < Length; i++)
	{
		if (i == (Length - 1))
			pData[i] = _SWI2C_ReadByte(0);
		else
			pData[i] = _SWI2C_ReadByte(1);
	}
	
	_SWI2C_Stop();
	return result;
}

static HAL_StatusTypeDef SWI2C_Write(uint8_t Addr, uint8_t SubAddr, uint8_t * pData, uint8_t Length)
{		
	uint8_t i;
	HAL_StatusTypeDef result;
	
	_SWI2C_Start();
	
	result = _SWI2C_WriteByte(Addr);
	if (result != HAL_OK)
	{
		_SWI2C_Stop();
		return result;
	}
	
	result = _SWI2C_WriteByte(SubAddr);
	if (result != HAL_OK)
	{
		_SWI2C_Stop();
		return result;
	}

	for (i = 0; i < Length; i++)
	{
		result = _SWI2C_WriteByte(pData[i]);
		if (result != HAL_OK)
		{
			_SWI2C_Stop();
			return result;
		}
	}
	_SWI2C_Stop();
	return result;
}

#define DP_COMMAND              0 
#define DP_DEVICE_ADDR          1 
#define DP_SUB_ADDR             2
#define DP_SUB_ADDR_2           3
#define DP_DATA_NUMBER          4
#define DP_DATA                 5

#define COMMAND_IIC_READ        0
#define COMMAND_IIC_WRITE       1

void CustomHID_ProcessOut(void)
{
	if (USB_Receive_Flag)
	{
		HAL_StatusTypeDef ret;
		uint8_t read_buffer[64];
		uint8_t num = ProcessBuffer[DP_DATA_NUMBER];
		if (num > 59)
		{
			num = 59;
		}
		if (ProcessBuffer[DP_COMMAND] == COMMAND_IIC_READ)
		{
			ret = SWI2C_Read(ProcessBuffer[DP_DEVICE_ADDR], ProcessBuffer[DP_SUB_ADDR], read_buffer, num);
			if (ret == HAL_OK)
			{
				memcpy(&ProcessBuffer[DP_DATA], read_buffer, num);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				YellowDelay = 200;
			}
			else
			{
				ProcessBuffer[DP_COMMAND] = 0xFF;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
				RedLEDDelay = 200;
			}
		}
		else
		{
			ret = SWI2C_Write(ProcessBuffer[DP_DEVICE_ADDR], ProcessBuffer[DP_SUB_ADDR], &ProcessBuffer[DP_DATA], num);
			if (ret == HAL_OK)
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
				YellowDelay = 200;
			}
			else
			{
				ProcessBuffer[DP_COMMAND] = 0xFF;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
				RedLEDDelay = 200;
			}
		}
		USBD_CUSTOM_HID_SendReport_FS(ProcessBuffer, 64);
		USB_Receive_Flag = 0;
	}
	if (YellowDelay == 1)
	{
		YellowDelay = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	}
	if (RedLEDDelay == 1)
	{
		RedLEDDelay = 0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	}
}

void CDC_UpdateTimer(void)
{
	if (YellowDelay > 1)
		YellowDelay--;
	if (RedLEDDelay > 1)
		RedLEDDelay--;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

