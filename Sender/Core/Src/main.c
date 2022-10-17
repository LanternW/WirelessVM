/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "NRF24l01.h"
#define TX_MODE
//#define RX_MODE
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

double  voltages[7] = {0};
uint8_t msg[11] = {0};
uint8_t tmp_buf[11];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t getADCValue(uint32_t channel)
{
	 ADC_ChannelConfTypeDef ADC_ChanConf;

	 ADC_ChanConf.Channel      = channel;                      //通道
	 ADC_ChanConf.Rank         = ADC_REGULAR_RANK_1;           //�?1个序列，序列1
	 ADC_ChanConf.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;      //采样时间
	 if (HAL_ADC_ConfigChannel(&hadc1, &ADC_ChanConf) != HAL_OK)
	 {
	    Error_Handler();
	 }
	 HAL_ADC_Start(&hadc1);                                 //�?启ADC
	 HAL_ADC_PollForConversion(&hadc1,10);                  //轮询转换
	 return (uint16_t)HAL_ADC_GetValue(&hadc1);	        	//返回�?近一次ADC1规则组的转换结果
}

uint16_t Get_Adc_Average(uint32_t channel, uint8_t times)
{
	uint32_t temp_val = 0;
	uint8_t t;
	for(t = 0; t < times; t++)
	{
		temp_val += getADCValue(channel);
		HAL_Delay(5);
	}
	return temp_val / times;
}

float v_ad    = 0.7; 		 //二极管压�?
float k1      = 0.0048340;   // (20 + x)/6206.06
float k2      = 0.0032538;   // (20 + x)/6206.06
float k3      = 0.0145020;   // (20 + x)/6206.06
float k4      = 0.0193359;   // (20 + x)/6206.06
float k5      = 0.0225586;   // (20 + x)/6206.06
float k6      = 0.0273926;   // (20 + x)/6206.06

void pubMessage()
{
	uint8_t cells     = 6;
	double  total_vol = 0.0;
	msg[0] = 0x07;
	msg[1] = 0x09;
	voltages[0] = Get_Adc_Average(ADC_CHANNEL_1, 20);
	voltages[1] = Get_Adc_Average(ADC_CHANNEL_2, 20);
	voltages[2] = Get_Adc_Average(ADC_CHANNEL_3, 20);
	voltages[3] = Get_Adc_Average(ADC_CHANNEL_4, 20);
	voltages[4] = Get_Adc_Average(ADC_CHANNEL_5, 20);
	voltages[5] = Get_Adc_Average(ADC_CHANNEL_6, 20);

	voltages[0] = voltages[0] * k1 + v_ad;
	voltages[1] = voltages[1] * k2 + v_ad;
	voltages[2] = voltages[2] * k3 + v_ad;
	voltages[3] = voltages[3] * k4 + v_ad;
	voltages[4] = voltages[4] * k5 + v_ad;
	voltages[5] = voltages[5] * k6 + v_ad;

	if(voltages[5] < 3.0){ cells = 5;}
	if(voltages[4] < 3.0){ cells = 4;}
	if(voltages[3] < 3.0){ cells = 3;}
	if(voltages[2] < 3.0){ cells = 2;}
	if(voltages[1] < 3.0){ cells = 1;}

	for( uint8_t i = 0 ; i < cells ; i++){
		total_vol += voltages[i];
	}

	msg[2] = cells;
	msg[3] = (uint8_t)( (voltages[0] - 3.0) * 100 );
	msg[4] = (uint8_t)( (voltages[1] - 3.0) * 100 ) - msg[3];
	msg[5] = (uint8_t)( (voltages[2] - 3.0) * 100 ) - msg[4];
	msg[6] = (uint8_t)( (voltages[3] - 3.0) * 100 ) - msg[5];
	msg[7] = (uint8_t)( (voltages[4] - 3.0) * 100 ) - msg[6];
	msg[8] = (uint8_t)( (voltages[5] - 3.0) * 100 ) - msg[7];

	total_vol *= 100;
	msg[9]  = (uint8_t)(total_vol / 255);
	msg[10] = (uint8_t)(total_vol) % 255;

	msg[2] = 3;
	msg[3] = (uint8_t)( (3.2 - 3.0) * 100 );
	msg[4] = (uint8_t)( (3.4 - 3.0) * 100 ) - msg[3];
	msg[5] = (uint8_t)( (3.6 - 3.0) * 100 ) - msg[4];


	if(NRF24L01_TxPacket(msg) != TX_OK){
		NRF24L01_TxPacket(msg); //发两�?
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  void blink2()
  {
	  int t = 3;
	   while( t-- ){
	  	  HAL_GPIO_WritePin(UserLED2_GPIO_Port, UserLED2_Pin, GPIO_PIN_RESET);
//	  	  HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_RESET);
	  	  PAout(12) = 0;
	  	  HAL_Delay(100);
	  	  HAL_GPIO_WritePin(UserLED2_GPIO_Port, UserLED2_Pin, GPIO_PIN_SET);
//	  	  HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_SET);
	  	  PAout(12) = 1;
	  	  HAL_Delay(100);
	   }
  }

  void blink_fast()
  {
	  int t = 8;
	   while( t-- ){
	  	  HAL_GPIO_WritePin(UserLED2_GPIO_Port, UserLED2_Pin, GPIO_PIN_RESET);
//	  	  HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_RESET);
	  	  PAout(12) = 0;
	  	  HAL_Delay(40);
	  	  HAL_GPIO_WritePin(UserLED2_GPIO_Port, UserLED2_Pin, GPIO_PIN_SET);
//	  	  HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_SET);
	  	  PAout(12) = 1;
	  	  HAL_Delay(40);
	   }
  }

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  NRF24L01_Init(&hspi1);

  blink2();
  HAL_Delay(1000);

  while(NRF24L01_Check())
  {
	  blink2();
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
//	  HAL_Delay(100);
//	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
//	  HAL_Delay(100);
  }

  HAL_Delay(200);
  blink2();
#ifdef TX_MODE
  NRF24L01_TX_Mode();
#endif

#ifdef RX_MODE
  NRF24L01_RX_Mode();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
#ifdef TX_MODE
	pubMessage();
	HAL_Delay(1000);
	PAout(12) = 0;
	HAL_Delay(80);
	PAout(12) = 1;
	HAL_Delay(80);
#endif
#ifdef RX_MODE
	if(NRF24L01_RxPacket(tmp_buf) == 0)//一旦接收到信息,则显示出来.
	{
		tmp_buf[10] = 0;//加入字符串结束符
		blink_fast();
	}
#endif
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  __HAL_SPI_ENABLE(&hspi1);

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UserLED2_GPIO_Port, UserLED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(UserLED_GPIO_Port, UserLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : UserLED2_Pin */
  GPIO_InitStruct.Pin = UserLED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : UserLED_Pin */
  GPIO_InitStruct.Pin = UserLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(UserLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  blink_fast();
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
