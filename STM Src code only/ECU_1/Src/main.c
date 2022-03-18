/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define BMP180_1_16     ((float) 0.0625)
#define BMP180_1_256    ((float) 0.00390625)
#define BMP180_1_2048   ((float) 0.00048828125)
#define BMP180_1_4096   ((float) 0.000244140625)
#define BMP180_1_8192   ((float) 0.0001220703125)
#define BMP180_1_32768  ((float) 0.000030517578125)
#define BMP180_1_65536  ((float) 0.0000152587890625)
#define BMP180_1_101325 ((float) 0.00000986923266726)

char st[20] = "Hello!\n\r";
char str[20] = "Welcome CDAC\n\r";
char str1[30] = "Knowledge park Banglore\n\r";

char buffer_my[500];

	uint8_t array[2];
	uint8_t array1[3];
	uint8_t buffer4[]={0x00<<6};
	uint8_t buffer2[]={0x2E};
	uint8_t buffer3[]={0x34};
	uint16_t addr=0xF6F7;
	uint32_t addr1=0xF6F7F8;
	uint8_t data[2];
	uint8_t data1[3];

uint16_t count;
uint32_t startTick;

uint32_t start, stop;

uint8_t MyData[8];

ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

CanTxMsgTypeDef            TxM;  
CanRxMsgTypeDef            RxM;
CAN_FilterConfTypeDef sFilterConfig;
ADC_ChannelConfTypeDef sConfig;

uint16_t x, y, z;
uint8_t xl,xh,yl,yh,zl,zh;  // my variables

float x_axis, y_axis, z_axis, Temprature;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);

void my_adc(void); // function of adc
void BMP_Call_Data(void); // engine RPM func


int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  
	
	hcan.pTxMsg = &TxM;
  hcan.pRxMsg = &RxM;

	
sFilterConfig.FilterNumber = 0;
sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
sFilterConfig.FilterIdHigh = 0x245 << 5;
sFilterConfig.FilterIdLow = 0;
sFilterConfig.FilterMaskIdHigh = 0;
sFilterConfig.FilterMaskIdLow = 0;
sFilterConfig.FilterFIFOAssignment = 0;
sFilterConfig.FilterActivation = ENABLE;
sFilterConfig.BankNumber = 14;
HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

HAL_UART_Transmit(&huart3, (uint8_t *)&st, sizeof(st), 20);
	
	
	
  while (1)
  {
		
		HAL_Delay(20000);
		my_adc();
		BMP_Call_Data();
		

  }

}


void BMP_Call_Data(void)
{
	int16_t AC1=408, AC2=-72, AC3=-14383, B1=6190, B2=4, MC=-8711, MD=2868;
	uint16_t AC4=32741, AC5=32757, AC6=23153, UT;
	int32_t X1, X2, X3, B3, B5, B6;
	uint32_t B4, B7, UP;
	float T,p;

		hcan.pTxMsg -> StdId = 0x259;
		hcan.pTxMsg -> RTR = CAN_RTR_DATA;
		hcan.pTxMsg -> IDE = CAN_ID_STD;
		hcan.pTxMsg -> DLC = 8;	
	
	for(uint8_t i=0; i<10; i++)
	{
		HAL_I2C_Mem_Write(&hi2c1, 0xEE,0xF4,1,buffer2, 1, 50);
		HAL_Delay(20);
		HAL_I2C_Mem_Read(&hi2c1,0xEE, addr, 2, data, 2, 100);

		UT=((data[0]<<8)|data[1]);
		sprintf(buffer_my,"\n\nUncomphensated Temperature=%d\n\r",UT);
		HAL_UART_Transmit(&huart3,(uint8_t *)&buffer_my,sizeof(buffer_my), 100);
		X1 = (UT - AC6) * AC5 * BMP180_1_32768;
		X2 = MC * 2048 / (X1 + MD);
		B5 = X1 + X2;
		T=(B5 + 8) / ((float)16.0);
		T=(T/6)-8;
		sprintf(buffer_my,"True Temperature=%lf\n\r",T);
		HAL_UART_Transmit(&huart3,(uint8_t *)&buffer_my,sizeof(buffer_my), 100);
			
			HAL_I2C_Mem_Write(&hi2c1, 0xEE,0xF4,1,buffer4, 1, 100);
			HAL_I2C_Mem_Write(&hi2c1, 0xEE,0xF4,1,buffer3, 1, 100);
			HAL_Delay(200);
			HAL_I2C_Mem_Read(&hi2c1,0xEE, addr1,3 , data1, 2, 100);
			UP=(data1[0]<<16|data1[1]<<8|data1[2])>>8;
			//AC2=data1[1];
			//AC3=data1[2];
			sprintf(buffer_my,"Uncomphensated Pressure=%d\r\n",UP);
			HAL_UART_Transmit(&huart3,(uint8_t *)buffer_my,sizeof(buffer_my), 100);
			B6 = B5 - 4000;
			X1 = (B2 * (B6 * B6 * BMP180_1_4096)) * BMP180_1_2048;
			X2 = AC2 * B6 * BMP180_1_2048;
			X3 = X1 + X2;
			B3 = (((AC1 * 4 + X3) <<0)+2)*0.25;
			X1 = AC3 * B6 * BMP180_1_8192;
			X2 = (B1 * (B6 * B6 * BMP180_1_4096)) * BMP180_1_65536;
			X3 = ((X1 + X2) + 2) * 0.25;
			B4 = AC4 * (uint32_t)(X3 + 32768) * BMP180_1_32768;
			B7 = ((uint32_t)UP - B3) * (50000 >> 0);
			if (B7 < 0x80000000)
				{
					p = (B7 * 2) / B4;
				} 
			else {
				p = (B7 / B4) * 2;
				}
			X1 = ((float)p * BMP180_1_256) * ((float)p * BMP180_1_256);
			X1 = (X1 * 3038) * BMP180_1_65536;
			X2 = (-7357 * p) * BMP180_1_65536;
			p = p + (X1 + X2 + 3791) * BMP180_1_16;
				p=p+25000;
			sprintf(buffer_my,"Comphensated Pressure =%lfPa\n\r",p);

			HAL_UART_Transmit(&huart3,(uint8_t *)&buffer_my,sizeof(buffer_my), 100);
				uint32_t Temprature = T;
				MyData[0] = Temprature;
				MyData[1] = Temprature >> 8;
				MyData[2] = Temprature >> 16;
				MyData[3] = Temprature >> 24;
				
				uint32_t Presure = p;
				MyData[4] = Presure;
				MyData[5] = Presure >> 8;
				MyData[6] = Presure >> 16;
				MyData[7] = Presure >> 24;
				
				hcan.pTxMsg -> Data[0] = MyData[0];	
				hcan.pTxMsg -> Data[1] = MyData[1];	
				hcan.pTxMsg -> Data[2] = MyData[2];	
				hcan.pTxMsg -> Data[3] = MyData[3];	
				hcan.pTxMsg -> Data[4] = MyData[4];	
				hcan.pTxMsg -> Data[5] = MyData[5];	
				hcan.pTxMsg -> Data[6] = MyData[6];	
				hcan.pTxMsg -> Data[7] = MyData[7];	
				HAL_CAN_Transmit(&hcan, 1);
				
				HAL_Delay(150);
			}
}

void my_adc(void)
{
	uint32_t mytick=0;
	uint32_t mytickstop=0;
	uint16_t g = 0;
	hcan.pTxMsg -> StdId = 0x250;
	hcan.pTxMsg -> RTR = CAN_RTR_DATA;
	hcan.pTxMsg -> IDE = CAN_ID_STD;
	hcan.pTxMsg -> DLC = 8;	
for(g=0; g<100; g++)
	{
	mytick = HAL_GetTick();
	
	    sConfig.Channel = ADC_CHANNEL_0;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			x = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
		
			sConfig.Channel = ADC_CHANNEL_1;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			y = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
		
			sConfig.Channel = ADC_CHANNEL_2;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			z = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);
			

			x_axis = (x*3.3)/4034.00;
			y_axis = (y*3.3)/4034.00;
			z_axis = (z*3.3)/4034.00;

			
			xl= x | xl;
			xh= x>>8;
			
			yl= y | yl;
			yh= y>>8;
			
			zl= z | zl;
			zh= z>>8;
			
			MyData[0]= xl;
			MyData[1]= xh;
			
			MyData[2]= yl;
			MyData[3]= yh;
			
			MyData[4]= zl;
			MyData[5]= zh;
			
			MyData[6]= (uint8_t)(Temprature);
			
	hcan.pTxMsg -> Data[0] = MyData[0];	
	hcan.pTxMsg -> Data[1] = MyData[1];	
	hcan.pTxMsg -> Data[2] = MyData[2];	
	hcan.pTxMsg -> Data[3] = MyData[3];	
	hcan.pTxMsg -> Data[4] = MyData[4];	
	hcan.pTxMsg -> Data[5] = MyData[5];	
	hcan.pTxMsg -> Data[6] = MyData[6];	
	hcan.pTxMsg -> Data[7] = MyData[7];	
	HAL_CAN_Transmit(&hcan, 1);
	HAL_Delay(120);
	mytickstop = HAL_GetTick();
	sprintf(buffer_my,"delay is %d\t %d\t %d\n\r", (mytickstop - mytick), mytick, mytickstop);
	HAL_UART_Transmit(&huart3, (uint8_t *)&buffer_my, sizeof(buffer_my), 20);
			
			//sprintf(buffer, "ADC Value of Channel_0 is %d\t X_axis value is %lf\t Y_axis value is %lf\t Z_axis value is %lf\t Temp value is %lf\t\n\r", x, x_axis, y_axis, z_axis , Temprature);
			//HAL_UART_Transmit(&huart3, (uint8_t *)buffer, sizeof(buffer), 100);
	}
}


void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

 

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_12TQ;
  hcan.Init.BS2 = CAN_BS2_5TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
