/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* declare a filter configuration structure */
FMAC_FilterConfigTypeDef sFmacConfig;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FMAC_HandleTypeDef hfmac;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const int16_t aFilterCoeffB_q15[X2_B_COEFF_SIZE] = {0x2000,  0x2000,  0x2000, 0x2000};  // SMA filter with 4 taps with value 0.25
const int16_t aFilterPreloadValues_q15[X1_PRELOAD_SIZE] = {0};
const int16_t aFIRInputX_q15[INPUT_SIZE_N] = {0x7FFF, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Dirac impulse
int16_t aFIROutputY_q15[INPUT_SIZE_N];

uint8_t iFMAC_Interrupt = 2; //interrupt flag, offset 2 for readability in STM32CubeMonitor;
uint8_t iREN = 10;
uint8_t iWEN = 8;
uint32_t iFMAC_START;
uint32_t iX1_FULL;
uint32_t iY_EMPTY;

uint16_t outputSize; // for HAL FMAC
uint16_t inputSize; // for HAL FMAC

uint64_t delayCounter = 0xFFFFF;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMAC_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Read_FMAC_Status(){
	/* Read status flags */
//	iY_EMPTY = FMAC->SR & FMAC_SR_YEMPTY;
	iY_EMPTY = (FMAC->SR & 0x01)  ? 1 : 0;
//	iX1_FULL = (FMAC->SR & FMAC_SR_X1FULL) + 4;
//	iX1_FULL = (FMAC->SR & FMAC_SR_X1FULL >> 1) + 4;
	iX1_FULL = (FMAC->SR & 0x02) ? 1 : 0;
	iX1_FULL = iX1_FULL + 4;
//	iFMAC_START = (FMAC->PARAM  & FMAC_PARAM_START) + 6;
	iFMAC_START = (FMAC->PARAM >> 31) + 6;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FMAC_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	/* Set the coefficient buffer base address */
	sFmacConfig.CoeffBaseAddress = 0;
	/* Set the coefficient buffer size to the number of coeffs */
	sFmacConfig.CoeffBufferSize = X2_B_COEFF_SIZE;
	/* Set the Input buffer base address to the next free address */
	sFmacConfig.InputBaseAddress = X2_B_COEFF_SIZE;
	/* Set the input buffer size greater than the number of coeffs */
	sFmacConfig.InputBufferSize = X1_INPUT_BUFF_SIZE;
	/* Set the input watermark to zero */
	sFmacConfig.InputThreshold = 0;
	/* Set the Output buffer base address to the next free address */
	sFmacConfig.OutputBaseAddress = X2_B_COEFF_SIZE + X1_INPUT_BUFF_SIZE;
	/* Set the output buffer size */
	sFmacConfig.OutputBufferSize = Y_OUTPUT_BUFF_SIZE;
	/* Set the output watermark to zero  */
	sFmacConfig.OutputThreshold = 0;
	/* No A coefficients since FIR */
	sFmacConfig.pCoeffA = NULL;
	sFmacConfig.CoeffASize = 0;
	/* Pointer to the coefficients in memory */
	sFmacConfig.pCoeffB = aFilterCoeffB_q15;
	/* Number of coefficients */
	sFmacConfig.CoeffBSize = X2_B_COEFF_SIZE;
	/* Select FIR filter function */
	sFmacConfig.Filter = FMAC_FUNC_CONVO_FIR;
	/* Enable polling input transfer */
	sFmacConfig.InputAccess = FMAC_BUFFER_ACCESS_IT;
	/* Enable polling output transfer */
	sFmacConfig.OutputAccess = FMAC_BUFFER_ACCESS_IT;
	/* Enable clipping of the output at 0x7FFF and 0x8000 */
	sFmacConfig.Clip = FMAC_CLIP_DISABLED; //FMAC_CLIP_ENABLED; //FMAC_CLIP_DISABLED;
	/* P parameter contains number of coefficients */
	sFmacConfig.P = X2_B_COEFF_SIZE;
	/* Q parameter is not used */
	sFmacConfig.Q = 0;
	/* R parameter contains the post-shift value (none) */
	sFmacConfig.R = 0;

	/* Read status flags */

	Read_FMAC_Status();
	HAL_Delay(DELAY);

	/* Configure the FMAC */
	if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK)
	/* Configuration Error */
	Error_Handler();

	//  FMAC->CR |= FMAC_CR_OVFLIEN;
	//  FMAC->CR |= FMAC_CR_UNFLIEN;

	/* There is no error in the output values: Turn LED2 on */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	Read_FMAC_Status();
	HAL_Delay(DELAY);

	/* Preload  input buffer with 4 samples*/
	if (HAL_FMAC_FilterPreload(&hfmac, aFilterPreloadValues_q15, X1_PRELOAD_SIZE, NULL, 0) != HAL_OK)
	{
	/* Configuration Error */
	Error_Handler();
	}

	Read_FMAC_Status();
	HAL_Delay(DELAY);

	/* Start FIR-Filter */

//		FMAC->PARAM &= ~FMAC_PARAM_START; // clear PARAM_START bit
//		FMAC->PARAM &= ~P_Msk; // Clear the P[7:0] field
//		FMAC->PARAM |= (X2_B_COEFF_SIZE << P_Pos);  // Write the value to the P[7:0] field
//		FMAC->PARAM &= ~FUNC_Msk; // Clear the FUNC field
//		FMAC->PARAM |=  ((8 << FUNC_Pos) | FMAC_PARAM_START);  // function 8 - Convolution
	 __HAL_FMAC_DISABLE_IT(&hfmac, FMAC_IT_OVFLIEN);
	 __HAL_FMAC_DISABLE_IT(&hfmac, FMAC_IT_UNFLIEN);
//	 __HAL_FMAC_DISABLE_IT(&hfmac, FMAC_IT_RIEN);


	outputSize = INPUT_SIZE_N;
	if (HAL_FMAC_FilterStart(&hfmac, aFIROutputY_q15, &outputSize) != HAL_OK)
	{

	Error_Handler();
	}





	  for(int i = 0; i < INPUT_SIZE_N; i++)
	  {
		    if(iFMAC_Interrupt == 3){
//		    	Read_FMAC_Status();
//		    	HAL_Delay(DELAY);

			    if((iWEN == 9)){
			    	if(i>1)
			    		FMAC->WDATA = aFIRInputX_q15[i-2];
			    	iFMAC_Interrupt = 2;
			    	iWEN = 8;
			    	Read_FMAC_Status();


			    	__HAL_FMAC_ENABLE_IT(&hfmac, FMAC_IT_WIEN);
			    }

			    if((iREN == 11) ){
			    	if(i==0)
			    		aFIROutputY_q15[i] = FMAC->RDATA;
			    	iFMAC_Interrupt = 2;
			    	iREN = 10;
			    	Read_FMAC_Status();

					__HAL_FMAC_ENABLE_IT(&hfmac, FMAC_IT_RIEN);
			    }

		    	HAL_Delay(DELAY);


		    }

	  }




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
