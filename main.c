/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2021 STMicroelectronics International N.V. 
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
#include "main.h"
#include "stm32f1xx_hal.h"
#include "fatfs.h"
#include "ov7670.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SDIO_SD_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

#define Width 160
#define Length 120
#define CAM_read_add 0x43
#define CAM_write_add 0x42
#define Num_Regs1 23

unsigned char i2c1_buf[2];
uint32_t Counter = 0x00000000;
uint8_t i2c1_buf2 = 0x00;
char flag=0x00;
/* USER CODE BEGIN 0 */
unsigned char pixel_data[2] = 0x00; 
/* USER CODE END 0 */
int BC = 0;
//char Header1[54]=0; // bitmapSignatureBytes[0]

const uint32_t los=Width*Length*2;
unsigned char sample[los]=0x00;

void wait_fun(uint32_t lim){
	uint32_t itr = 0; 
	while(itr<lim){itr++;}
}
int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, 1);
	HAL_Delay(10);
  /* USER CODE END 2 */
	if ( HAL_I2C_IsDeviceReady(&hi2c1, CAM_read_add, 5,10) == HAL_OK ){
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, 0);
			HAL_Delay(100);
		}
	  /* Infinite loop */
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);	//	|OE = 0
	
	// Reset
	i2c1_buf[0]= 0x12; // COM7
	i2c1_buf[1]= 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, CAM_write_add, i2c1_buf,2,100);
	HAL_Delay(2000);
	
//	for (uint32_t reg_num =0;reg_num < CHANGE_REG;reg_num++ ){
//		
//		i2c1_buf[0] = Camera_REG[reg_num][0];
//		i2c1_buf[1] = Camera_REG[reg_num][1];
//		HAL_I2C_Master_Transmit(&hi2c1, CAM_write_add, i2c1_buf,2,100);
//		HAL_Delay(500);
//	}
		
//	for (uint32_t reg_num =0;reg_num < Debug_Register_Num;reg_num++ ){
//		
//		i2c1_buf[0] = InitBuffer2[reg_num][0];
//		i2c1_buf[1] = InitBuffer2[reg_num][1];
//		HAL_I2C_Master_Transmit(&hi2c1, CAM_write_add, i2c1_buf,2,100);
//		HAL_Delay(500);
//	}

//	// Camera_REG
//	for (uint32_t reg_num =0;reg_num < CHANGE_REG;reg_num++ ){
//		
//		i2c1_buf[0] = Camera_REG[reg_num][0];
//		i2c1_buf[1] = Camera_REG[reg_num][1];
//		HAL_I2C_Master_Transmit(&hi2c1, CAM_write_add, i2c1_buf,2,100);
//		HAL_Delay(100);
//	}	

	// QQVGA_RGB565
	for (uint32_t reg_num =0;reg_num < Num_Regs1;reg_num++ ){
		
		i2c1_buf[0] = QQVGA_RGB565[reg_num][0];
		i2c1_buf[1] = QQVGA_RGB565[reg_num][1];
		HAL_I2C_Master_Transmit(&hi2c1, CAM_write_add, i2c1_buf,2,100);
		HAL_Delay(100);
	}
	
	// COLOR_SETTING
	for (uint32_t reg_num =0;reg_num < 22;reg_num++ ){
		
		i2c1_buf[0] = COLOR_SETTING[reg_num][0];
		i2c1_buf[1] = COLOR_SETTING[reg_num][1];
		HAL_I2C_Master_Transmit(&hi2c1, CAM_write_add, i2c1_buf,2,100);
		HAL_Delay(100);
	}	
	

	///////////////////
	FATFS myFATFS;
	FIL wFILE;
	char myPath [] = "IMG5.bmp\0";
//	char myPath [] = "IMG5.TXT\0";
	UINT testByte;
	///////////////////
	if ( f_mount(&myFATFS,SD_Path,1)==FR_OK  ){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 0);
	}
	// Write Diasble
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);	//	|WR
	wait_fun(500);
	// Write Reset
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);  //	WRST
	HAL_Delay(5);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);  //	WRST
	wait_fun(500);
	// Read reset
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 0);	//	RRST
	wait_fun(500);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);	//	RCK = 0
	wait_fun(500);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);	//	RCK = 1
	wait_fun(500);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 1);	//	RRST
	wait_fun(500);
	
	// Write a frame
	uint8_t value1 = 0x00;
	uint8_t value2 = 0x00;
	while(!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11));
	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11));
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 0);  //	WRST
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, 1);  //	WRST
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);	//	|WR
	while(!HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11)){
		value1 = value2;
		if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == 1){
			value2 = 0x01;
		}else{value2 = 0x00;}
		
		if ( (value1 == 0x00) & (value2 == 0x01) ){Counter++;}
	}
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);	//	|WR
	wait_fun(500);
	
	// Read a frame
	uint32_t address = 0x00;

	for(uint32_t address = 0; address<Length*Width*2;address++){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1);	//	RCK = 1

		sample[address] = (GPIOE->IDR&GPIO_PIN_7)	>>7
							| ((GPIOE->IDR)&GPIO_PIN_8) >>7 
							| ((GPIOD->IDR)&GPIO_PIN_0)	<<2 
							| ((GPIOD->IDR)&GPIO_PIN_1)	<<2
							| ((GPIOD->IDR)&GPIO_PIN_14)>>10
							| ((GPIOD->IDR)&GPIO_PIN_15)>>10 
							| ((GPIOD->IDR)&GPIO_PIN_5) <<1 
							| ((GPIOD->IDR)&GPIO_PIN_4) <<3 ;

//		sample[address] = (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_7)*0x01)
//							| (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8)*0x02 )
//							| (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_0)*0x04)
//							| (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1)*0x08)
//							| (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_14)*0x10)
//							| (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15)*0x20 )
//							| (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_5)*0x40 )
//							| (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4)*0x80);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0);	//	RCK = 0
	}

	char myPath1 [] = "IMG5.bmp\0";
	f_open(&wFILE, myPath1, FA_WRITE | FA_CREATE_ALWAYS);
	f_write(&wFILE, Header1, sizeof(Header1), &testByte);
	f_write(&wFILE, &sample, sizeof(sample), &testByte);

	f_close(&wFILE);
	
//	char myPath2 [] = "IMG5.TXT\0";
//	f_open(&wFILE, myPath2, FA_WRITE | FA_CREATE_ALWAYS);
//	f_write(&wFILE, Header1, sizeof(Header1), &testByte);
//	f_write(&wFILE, &sample, sizeof(sample), &testByte);
//	f_close(&wFILE);

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, 1);

  while (1)
  {

  }
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE9 PE10 PE11 
                           PE12 PE13 PE14 PE15 
                           PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD14 PD15 PD0 
                           PD1 PD4 PD5 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0 
                          |GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
