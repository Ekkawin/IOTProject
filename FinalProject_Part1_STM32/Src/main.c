/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
# define usTIM TIM4
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
float ultrasonic();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char uartBuf[100];
int c;
int d;
int x;
int y;
int check;
float distance;
int numtick;
const float speedOfSound = 0.0343/2;
float range;
void Start(){

	Turn();
	x = 25;
	sprintf(uartBuf,"x= %.1d\r\n", x);
	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
	y = 25;
	TIM3->CCR1 = TIM3->ARR* x/100;
	TIM3->CCR2 = TIM3->ARR* y/100;
	check = 1;
	while(check == 1){
		check = Stop();
		linetrack(x, y);
		range = ultrasonic();
		checkrange(range, x, y);
		sprintf(uartBuf,"Range= %.1f\r\n", range);
		HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
		HAL_UART_Transmit(&huart6, (uint8_t*)uartBuf, strlen(uartBuf), 1000);

	}
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
}

void linetrack(x, y){
	c = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	sprintf(uartBuf,"Left= %.1d\r\n", c);
	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);

	d = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	sprintf(uartBuf,"Right)= %.1d\r\n", d);
	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
	Stop();
	if (c == 1){
		while(c == 1){
			/*range = ultrasonic();
			checkrange(range, x, y);
			sprintf(uartBuf,"Range= %.1f\r\n", range);

			HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
			HAL_UART_Transmit(&huart6, (uint8_t*)uartBuf, strlen(uartBuf), 1000)*/
			TIM3->CCR2 = 0;
			c = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
			d = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
			if (d ==1){
				break;
			}

			}
		TIM3->CCR2 = TIM3-> ARR* y/100;



		}
	if (d == 1){
		while(d == 1){
			/*range = ultrasonic();
			checkrange(range, x, y);
			sprintf(uartBuf,"Range= %.1f\r\n", range);
			HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
			HAL_UART_Transmit(&huart6, (uint8_t*)uartBuf, strlen(uartBuf), 1000);*/
			TIM3->CCR1 = 0;
			c = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
			d = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
			if (c == 1){
				break;

			}
			}
		TIM3->CCR1 = TIM3-> ARR* x/100;

		}
}
int Stop(){
	c = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
	sprintf(uartBuf,"StopLeft= %.1d\r\n", c);
	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
	d = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	sprintf(uartBuf,"StopRight)= %.1d\r\n", d);
	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
	if (c == 1){
		if (d == 1){
			return 0;
		}else{
			return 1;
		}
	}
	else{
		return 1 ;
	}

}
void Turn(){
	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)== GPIO_PIN_RESET){
		c = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
		sprintf(uartBuf,"PIN0= %.1d\r\n", c);
		HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
		d = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		sprintf(uartBuf,"PIN1)= %.1d\r\n", d);
		HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
	}
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1)== GPIO_PIN_SET){
		TIM3->CCR2 = TIM3->ARR * 50/100;
		TIM3->CCR1 = 0;
		HAL_Delay(1200);
		TIM3->CCR2 =0;
	}else{
		TIM3->CCR1 = TIM3->ARR * 50/100;
		TIM3->CCR2 = 0;
		HAL_Delay(1200);
		TIM3->CCR1 =0;

	}
	TIM3->CCR1 = TIM3->ARR * 20/100;
	TIM3->CCR2 = TIM3->ARR * 20/100;

}
float ultrasonic(){
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	usDelay(3);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin)== GPIO_PIN_RESET);
	numtick = 0;
	while(HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin)== GPIO_PIN_SET)
		{
		 	numtick++;
		 	usDelay(2);
		   };
	distance = (numtick+0.0f)*2.8*speedOfSound ;
	sprintf(uartBuf,"Ultrasonicrange= %.1f\r\n", distance);
	HAL_Delay(200);
	HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);

	return distance;

}
void checkrange(range, x, y){
	range = ultrasonic();
	while (range<10){
		range = ultrasonic();
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		sprintf(uartBuf,"Range= %.1f\r\n", range);
		HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
		HAL_UART_Transmit(&huart6, (uint8_t*)uartBuf, strlen(uartBuf), 1000);

	}
	TIM3->CCR1 = TIM3->ARR* x/100;
	TIM3->CCR2 = TIM3->ARR* y/100;

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t numtick ;
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
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1 );
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2 );
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1 );
  Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1){
	 /* TIM3->CCR1 = TIM3->ARR  * 70 / 100;
	  TIM3->CCR2 = TIM3->ARR  * 30 / 100;
	  TIM4->CCR1 = TIM4->ARR  * 70 / 100;
	  HAL_Delay(1000);
	  c = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
	  sprintf(uartBuf,"Left= %.1d\r\n", c);
	  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);
	  d = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
	  sprintf(uartBuf,"Right)= %.1d\r\n", d);
	  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 1000);*/
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */
  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */
  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */
  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin LD2_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ECHO_Pin PA8 */
  GPIO_InitStruct.Pin = ECHO_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void usDelay(uint32_t usec)
{
	if(usec < 2) usec =2;
	usTIM ->ARR = usec - 1;
	usTIM ->EGR = 1;
	usTIM ->SR &= ~1;
	usTIM ->CR1 |= 1 ;
	while((usTIM ->SR&0x0001)!= 1);
	usTIM -> SR &= ~(0x0001);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
