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
#include "input_values.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// define bitmasks
#define FUNC_Pos 24U
#define FUNC_Msk (0x7FUL << FUNC_Pos)
#define P_Pos    0U   // P starts at bit 0
#define P_Msk    (0xFFUL << P_Pos) // 8-bit wide mask
/* declare a filter configuration structure */
FMAC_FilterConfigTypeDef sFmacConfig;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FMAC_HandleTypeDef hfmac;
float input;
float output;
float inputSineWave;

/* USER CODE BEGIN PV */
const float aFilterCoeff_b[101] =

{
		  0.0021f, 0.0019f, 0.0016f, 0.0014f, 0.0012f, 0.0009f, 0.0005f, 0.0001f, -0.0005f, -0.0011f,
		  -0.0019f, -0.0028f, -0.0039f, -0.0050f, -0.0063f, -0.0076f, -0.0090f, -0.0104f, -0.0119f,
		  -0.0133f, -0.0146f, -0.0158f, -0.0168f, -0.0176f, -0.0182f, -0.0185f, -0.0185f, -0.0182f, -0.0175f,
		  -0.0164f, -0.0149f, -0.0131f, -0.0109f, -0.0083f, -0.0055f, -0.0024f, 0.0010f, 0.0045f,
		  0.0081f, 0.0118f, 0.0154f, 0.0190f, 0.0224f, 0.0256f, 0.0285f, 0.0311f, 0.0333f, 0.0350f,
		  0.0363f, 0.0371f, 0.0373f, 0.0371f, 0.0363f, 0.0350f, 0.0333f, 0.0311f, 0.0285f,
		  0.0256f, 0.0224f, 0.0190f, 0.0154f, 0.0118f, 0.0081f, 0.0045f, 0.0010f, -0.0024f, -0.0055f,
		  -0.0083f, -0.0109f, -0.0131f, -0.0149f, -0.0164f, -0.0175f, -0.0182f, -0.0185f, -0.0185f,
		  -0.0182f, -0.0176f, -0.0168f, -0.0158f, -0.0146f, -0.0133f, -0.0119f, -0.0104f, -0.0090f,
		  -0.0076f, -0.0063f, -0.0050f, -0.0039f, -0.0028f, -0.0019f, -0.0011f, -0.0005f, 0.0001f, 0.0005f,
		  0.0009f, 0.0012f, 0.0014f, 0.0016f, 0.0019f, 0.0021f
		};

static int16_t  aFilterCoeff_b_q15[101];

static int16_t aFilterPreloadValues_q15[101] = {0};

static int16_t aInputValues_yn_q15[100];

volatile int16_t aOutputValues_q15[2001];
float aOutputValues[2001];

uint32_t uX1BufferFullFlag;
uint32_t uYBufferEmptyFlag;
uint32_t uStartFalg;
uint32_t uSaturationFlag;
uint32_t uUnderflowFlag;
uint32_t uOverflowFlag;
uint32_t uUnderflowEnagbleFlag;
uint32_t uWriteEnableInterruptFlag;
uint32_t uReadEnableInterruptFlag;
uint32_t uFunction;
uint32_t uParamP;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMAC_Init(void);
/* USER CODE BEGIN PFP */
int16_t float_to_q15(float value);
float q15_to_float(int16_t value);
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
  /* USER CODE BEGIN 2 */
  for(int i = 0; i < 101; i++)
  {
	  aFilterCoeff_b_q15[i] = (int16_t)(aFilterCoeff_b[i]*0x8000); // Convert from float to Q1.15 format

  }

  /* Set the coefficient buffer base address */
  sFmacConfig.CoeffBaseAddress = 0;
  /* Set the coefficient buffer size to the number of coeffs */
  sFmacConfig.CoeffBufferSize = 101;
  /* Set the Input buffer base address to the next free address */
  sFmacConfig.InputBaseAddress = 101;
  /* Set the input buffer size greater than the number of coeffs */
  sFmacConfig.InputBufferSize = 101;
  /* Set the input watermark to zero */
  sFmacConfig.InputThreshold = 0;
  /* Set the Output buffer base address to the next free address */
  sFmacConfig.OutputBaseAddress = 202;
  /* Set the output buffer size */
  sFmacConfig.OutputBufferSize = 54;
  /* Set the output watermark to zero  */
  sFmacConfig.OutputThreshold = 0;
  /* No A coefficients since FIR */
  sFmacConfig.pCoeffA = NULL;
  sFmacConfig.CoeffASize = 0;
  /* Pointer to the coefficients in memory */
  sFmacConfig.pCoeffB = aFilterCoeff_b_q15;
  /* Number of coefficients */
  sFmacConfig.CoeffBSize = 101;
  /* Select FIR filter function */
  sFmacConfig.Filter = FMAC_FUNC_CONVO_FIR;
  /* Enable polling input transfer */
  sFmacConfig.InputAccess = FMAC_BUFFER_ACCESS_POLLING;  //FMAC_BUFFER_ACCESS_NONE;
  /* Enable polling output transfer */
  sFmacConfig.OutputAccess = FMAC_BUFFER_ACCESS_POLLING; // FMAC_BUFFER_ACCESS_NONE;
  /* Enable clipping of the output at 0x7FFF and 0x8000 */
  sFmacConfig.Clip = FMAC_CLIP_DISABLED;// FMAC_CLIP_ENABLED;//FMAC_CLIP_DISABLED;
  /* P parameter contains number of coefficients */
  sFmacConfig.P = 101;
  /* Q parameter is not used */
  sFmacConfig.Q = 0;
  /* R parameter contains the post-shift value (none) */
  sFmacConfig.R = 0; // apply gain 2^R
  /* Configure the FMAC */
  if (HAL_FMAC_FilterConfig(&hfmac, &sFmacConfig) != HAL_OK)
  /* Configuration Error */
  Error_Handler();

  /* There is no error in the output values: Turn LED2 on */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

  // check configuration

  uUnderflowEnagbleFlag = FMAC->CR & FMAC_CR_UNFLIEN_Msk;
  uWriteEnableInterruptFlag = FMAC->CR & FMAC_CR_WIEN_Msk;
  uReadEnableInterruptFlag = FMAC->CR & FMAC_CR_RIEN_Msk;
  uFunction = FMAC->PARAM & FMAC_PARAM_FUNC;
  uParamP = FMAC->PARAM & FMAC_PARAM_P;


    /*## Preload the input and output buffers ##################################*/
  if (HAL_FMAC_FilterPreload(&hfmac, aFilterPreloadValues_q15, 101, NULL, 0) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }


// FMAC->PARAM &= ~FMAC_PARAM_START; // clear PARAM_START bit

  // start the filter

  FMAC->PARAM &= ~P_Msk; // Clear the P[7:0] field
  FMAC->PARAM |= (101 << P_Pos);  // Write the value to the P[7:0] field
  FMAC->PARAM &= ~FUNC_Msk; // Clear the FUNC field
  FMAC->PARAM |=  ((8 << FUNC_Pos) | FMAC_PARAM_START);  // function 8 - Convolution

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for(int i = 0; i < 2001; i++)
	  {

		  aInputValues_yn_q15[i] = (int16_t)(aInputValues_yn[i]*0x8000 /5.0); // Convert from float to Q1.15 format
		  FMAC->WDATA = aInputValues_yn_q15[i];
		  aOutputValues_q15[i] = FMAC->RDATA;
		  aOutputValues[i] = (float)aOutputValues_q15[i]/(float)0x8000;  // convert from Q1.15 to float


		  inputSineWave = aSineWave_y[i];
		  //input = aInputValues_yn[i];
		  output = 5.0*aOutputValues[i];
		  HAL_Delay(1);
	  }
	  HAL_Delay(100);
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

// Function to convert a float to Q1.15 format
int16_t float_to_q15(float value) {
        return (int16_t)(value * 0x8000);
}

// Function to convert a Q1.15 format value to float
float q15_to_float(int16_t value) {
    return (float)value / 0x8000;
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
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	  HAL_Delay(500);
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
