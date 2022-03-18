#include "main.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

char st[20] = "Hello!\n\r";
char str[20] = "Welcome CDAC\n\r";
char str1[30] = "Knowledge park Banglore\n\r";

uint16_t count;
uint32_t startTick;
char buffer[500];
uint32_t start, stop;
double RPM_Engine;

ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

ADC_ChannelConfTypeDef sConfig;

UART_HandleTypeDef huart3;


CanTxMsgTypeDef            TxM;  
CanRxMsgTypeDef            RxM;
CAN_FilterConfTypeDef sFilterConfig;

uint16_t x, y, z, Temp;
uint8_t xl,xh,yl,yh,zl,zh;  // my variables
uint8_t MyData[8];
float x_axis, y_axis, z_axis, Temprature;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);

void my_adc(void); // function of adc
void Engine_RPM(void); // engine RPM func

/* generation of delay in microseconds */
	
extern uint32_t SystemCoreClock;
 
void DWT_Init(void) 
{
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) 
  {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}
 
uint32_t DWT_Get(void)
{
  return DWT->CYCCNT;
}
 
__inline
uint8_t DWT_Compare(int32_t tp)
{
  return (((int32_t)DWT_Get() - tp) < 0);
}
 
void DWT_Delay(uint32_t us) // microseconds
{
  int32_t tp = DWT_Get() + us * (SystemCoreClock/1000000);
  while (DWT_Compare(tp));
}
	
	
/* user code end */


int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_CAN_Init();

	DWT_Init(); //user init
	
	hcan.pTxMsg = &TxM;
  hcan.pRxMsg = &RxM;

	
sFilterConfig.FilterNumber = 0;
sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
sFilterConfig.FilterIdHigh = 0x244 << 5;
sFilterConfig.FilterIdLow = 0;
sFilterConfig.FilterMaskIdHigh = 0;
sFilterConfig.FilterMaskIdLow = 0;
sFilterConfig.FilterFIFOAssignment = 0;
sFilterConfig.FilterActivation = ENABLE;
sFilterConfig.BankNumber = 14;
HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);
hcan.pTxMsg -> StdId = 0x245;
hcan.pTxMsg -> RTR = CAN_RTR_DATA;
hcan.pTxMsg -> IDE = CAN_ID_STD;
hcan.pTxMsg -> DLC = 8;
HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);

HAL_UART_Transmit(&huart3, (uint8_t *)&st, sizeof(st), 20);

  while (1)
  {
						
			my_adc();
			Engine_RPM();
		
				hcan.pTxMsg -> Data[0] = MyData[0];	
				hcan.pTxMsg -> Data[1] = MyData[1];	
				hcan.pTxMsg -> Data[2] = MyData[2];	
				hcan.pTxMsg -> Data[3] = MyData[3];	
				hcan.pTxMsg -> Data[4] = MyData[4];	
				hcan.pTxMsg -> Data[5] = MyData[5];	
				hcan.pTxMsg -> Data[6] = MyData[6];	
				hcan.pTxMsg -> Data[7] = MyData[7];	
				HAL_CAN_Transmit(&hcan, 1);
		  	HAL_UART_Transmit(&huart3, (uint8_t *)&hcan.pTxMsg -> Data[0], sizeof(hcan.pTxMsg -> Data[0]), 20);


  }

}

void my_adc(void)
{
	
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
			
			sConfig.Channel = ADC_CHANNEL_4;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
			HAL_ADC_ConfigChannel(&hadc1, &sConfig);
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 100);
			Temp = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			x_axis = (x*3.3)/4034.00;
			y_axis = (y*3.3)/4034.00;
			z_axis = (z*3.3)/4034.00;
			Temprature = (Temp/10.0);
			
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
			
			//sprintf(buffer, "ADC Value of Channel_0 is %d\t X_axis value is %lf\t Y_axis value is %lf\t Z_axis value is %lf\t Temp value is %lf\t\n\r", x, x_axis, y_axis, z_axis , Temprature);
			//HAL_UART_Transmit(&huart3, (uint8_t *)buffer, sizeof(buffer), 100);
  
}

void Engine_RPM(void)
{
	double RPM_Engine2 = 0;
	for(uint8_t i=0; i<3; i++)
		{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);		
			start = DWT_Get();
				while(count<10)
				{
					count++;
					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					sprintf(buffer, "No. of counts in 1 sec is %d\n\r", count);
					HAL_UART_Transmit(&huart3, (uint8_t *)&buffer, sizeof(buffer), 100);
					while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8));
				}
				stop = DWT_Get();
			count = 0;
			RPM_Engine = 10000000.00/(stop - start);
			RPM_Engine = (RPM_Engine*4320.00);
			
				RPM_Engine2 += RPM_Engine ; 
				//sprintf(buffer, "RPM is %lf\n\r", RPM_Engine);
		//	HAL_UART_Transmit(&huart3, (uint8_t *)&buffer, sizeof(buffer), 100);
		}
		MyData[7] = (uint8_t)((RPM_Engine2 / 3.00)/9);  // for sending data of 16 bit to 8 bit
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

  ADC_ChannelConfTypeDef sConfig;

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

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
