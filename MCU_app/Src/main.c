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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DIR_LEFT GPIO_PIN_SET
#define DIR_RIGHT GPIO_PIN_RESET
#define SLEEP_MODE 0
#define ACTIVE_MODE 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
const uint8_t  FINGERS_MASK = 0x80;		//MSB = 1 -> fingers locations detection mode;
										//at the same time it is a mask to check the MSB of first
										//got byte, this bit decides about mode type
const uint8_t ROTATION_MASK = 0x00;		//MSB = 0 -> mode: calculating a revolution angle in wrist
const uint8_t END_COMMUNICATION = 0xEE;

const uint8_t ANGLE_MASK = 0x7E;		//mask to get bits of revolution angle value
//
const uint8_t angle_MAX = 100;
volatile uint8_t angle_counter = 0;
volatile uint8_t angle_set = 0;			//got value of revolution angle

volatile uint8_t servo_nr = 1;
//indeks poczatku bajtow danych dla kolejnego serwa
volatile uint8_t finger_ready = 1;		//byte checking if next finger is ready to move
volatile uint8_t move_type;

volatile uint8_t button_pressed = 0;
volatile uint8_t sleep = SLEEP_MODE;

uint8_t PC_data[21] = {0};		//each time it is supposed to get 21 bytes: 1st byte says about
								//mode and the rest of them are data bytes for servos
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM9_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

void activate_motor();
void deactivate_motor();
void move_fingers();
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
  MX_TIM10_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  move_type = ROTATION_MASK;
  deactivate_motor();

  HAL_UART_Receive_IT(&huart1, PC_data, 21);
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 9999;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 9999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 399;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_green_Pin|LED_orange_Pin|LED_red_Pin|LED_blue_Pin 
                          |Motor_STEP_Pin|Motor_DIR_Pin|Motor_SLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PIN_sleep_Pin */
  GPIO_InitStruct.Pin = PIN_sleep_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PIN_sleep_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_green_Pin LED_orange_Pin LED_red_Pin LED_blue_Pin 
                           Motor_STEP_Pin Motor_DIR_Pin Motor_SLEEP_Pin */
  GPIO_InitStruct.Pin = LED_green_Pin|LED_orange_Pin|LED_red_Pin|LED_blue_Pin 
                          |Motor_STEP_Pin|Motor_DIR_Pin|Motor_SLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim10.Instance) {
		if (sleep == ACTIVE_MODE) {
			if (angle_counter == 0)		//first step
				HAL_GPIO_WritePin(Motor_STEP_GPIO_Port, Motor_STEP_Pin,
						GPIO_PIN_RESET);
			else
				HAL_GPIO_TogglePin(Motor_STEP_GPIO_Port, Motor_STEP_Pin);

			angle_counter++;

			if (angle_counter == angle_set)	//100 changes LOW->HIGH are 50 steps which are equal
											//90 degrees angle revolution (it depend on used
											//stepper motor)
			{
				angle_counter = 0;
				HAL_GPIO_TogglePin(LED_blue_GPIO_Port, LED_blue_Pin);
				HAL_TIM_Base_Stop_IT(&htim10);
				deactivate_motor();
			}
		}
	}
	else if (htim->Instance == htim9.Instance)
	{
		if (HAL_GPIO_ReadPin(PIN_sleep_GPIO_Port, PIN_sleep_Pin)
				== GPIO_PIN_RESET)		//if button has been released
		{
			button_pressed = 0;
			HAL_TIM_Base_Stop_IT(&htim9);
			finger_ready = 1;
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_Pin);

	if (GPIO_Pin == PIN_sleep_Pin) {
		if (button_pressed == 0) {
			button_pressed = 1;
			HAL_GPIO_TogglePin(LED_green_GPIO_Port, LED_green_Pin);
			HAL_GPIO_TogglePin(LED_red_GPIO_Port, LED_red_Pin);

			(sleep == ACTIVE_MODE) ? (sleep = SLEEP_MODE) : (sleep = ACTIVE_MODE);
			HAL_GPIO_WritePin(Motor_SLEEP_GPIO_Port, Motor_SLEEP_Pin, sleep);

			HAL_TIM_Base_Start_IT(&htim9);
		}
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (PC_data[0] != END_COMMUNICATION)
			{
		move_type = PC_data[0] & FINGERS_MASK;

		if (move_type == ROTATION_MASK)
				{
			angle_set = PC_data[0] & ANGLE_MASK;
			if (angle_set > angle_MAX)
				angle_set = angle_MAX;

			HAL_GPIO_WritePin(Motor_DIR_GPIO_Port, Motor_DIR_Pin, DIR_RIGHT);	//rotation always counterclockwise
			activate_motor();
			HAL_TIM_Base_Start_IT(&htim10);
			HAL_UART_Receive_IT(&huart1, PC_data, 21);
		} else if (move_type == FINGERS_MASK)
				{
			move_fingers();
			servo_nr = 1;		//set servo_nr to 1 to enable fingers to move back to start
								//position in next step
			HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_SET);
			HAL_UART_Receive_IT(&huart1, PC_data, 21);
		}
	} else if (PC_data[0] == END_COMMUNICATION)
			{
		if (move_type == ROTATION_MASK)
				{
			if (angle_counter != 0)	//if sent END_COMMUNICATION before movement has finished
					{
				angle_set = angle_counter;
				angle_counter = 0;
			}

			HAL_GPIO_WritePin(Motor_DIR_GPIO_Port, Motor_DIR_Pin, DIR_LEFT); //comeback - always clockwise
			activate_motor();
			HAL_TIM_Base_Start_IT(&htim10);
			HAL_UART_Receive_IT(&huart1, PC_data, 21);
		} else if (move_type == FINGERS_MASK)
				{
			move_fingers();
			servo_nr = 1;
			HAL_GPIO_WritePin(LED_blue_GPIO_Port, LED_blue_Pin, GPIO_PIN_RESET);
			HAL_UART_Receive_IT(&huart1, PC_data, 21);
		}
	}
}

void activate_motor()
{
	sleep = ACTIVE_MODE;
	HAL_GPIO_WritePin(Motor_SLEEP_GPIO_Port, Motor_SLEEP_Pin, sleep);
	HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_RESET);
}

void deactivate_motor()
{
	sleep = SLEEP_MODE;
	HAL_GPIO_WritePin(Motor_SLEEP_GPIO_Port, Motor_SLEEP_Pin, sleep);
	HAL_GPIO_WritePin(LED_green_GPIO_Port, LED_green_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_red_GPIO_Port, LED_red_Pin, GPIO_PIN_SET);
}

void move_fingers()
{
	while (servo_nr < 21)
	{
		HAL_UART_Transmit(&huart1, &PC_data[servo_nr], 4,100);
		servo_nr+=4;
	}
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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
