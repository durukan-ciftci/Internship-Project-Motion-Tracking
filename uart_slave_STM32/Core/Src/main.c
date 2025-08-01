/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "STEPPER.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIR_CW            0
#define DIR_CCW           1

#define STEPPER_MOTOR1   0
#define SLAVE_ADRESS1 1
#define SLAVE_ADRESS2 2
#define SLAVE_ADRESS3 3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
uint8_t S_RxData[1];
uint8_t S_TxData[7];
uint8_t S_command_H;
uint8_t S_command_L;
uint8_t S_data_H;
uint8_t S_data_L;

uint8_t S_counter = 0;
uint8_t S_is_start_byte = 0;
uint8_t S_is_adress_me = 0;
uint8_t S_is_error = 0;
uint8_t S_checksum = 0;

uint8_t S_step_rpm_speed;

uint16_t S_step_remaining_steps;

uint8_t S_Step_direction = DIR_CW;

uint8_t S_Servo1_angle = 0;
uint16_t S_Servo1_angle_value = 0;

uint8_t S_Servo2_angle = 0;
uint16_t S_Servo2_angle_value = 0;

uint8_t val;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);


void calculate_checksum(uint8_t *data){
	data[6] = data[5]+data[4]+data[3]+data[2]+data[1];
}

uint8_t check_sum(uint8_t command_H, uint8_t command_L, uint8_t data_H, uint8_t data_L, uint8_t slave_adress){
	return (command_H + command_L + data_H + data_L + slave_adress);
}

float map( uint8_t data_H, uint8_t data_L, uint16_t max_val, uint16_t min_val){
    uint16_t bit16_val = ((uint16_t)data_H << 8) | data_L;
    return min_val + ( (float)(bit16_val) / 65535.0f ) * (max_val - min_val);
}

void generate_message(uint8_t* data ,uint8_t slave_adress, uint8_t command_H, uint8_t command_L, uint8_t data_H, uint8_t data_L){
	data[0] = 0xFF,
	data[1] = slave_adress;
	data[2] = command_H;
	data[3] = command_L;
	data[4] = data_H;
	data[5] = data_L;
	data[6] = 0x00;
}

void S_transmit_message(uint8_t *data, uint8_t slave_adress, uint8_t command_H, uint8_t command_L, uint8_t data_H, uint8_t data_L, uint8_t size){
	generate_message(data, slave_adress, command_H, command_L, data_H, data_L);
	calculate_checksum(data);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart6, data, size);
}

void S_Step_RPM_Response(uint8_t *data, uint8_t slave_adress, uint8_t speed){ //speed is between 14-0

	uint16_t result = (uint16_t)(((float)speed / 14.0f) * 65535.0f + 0.5f);
	uint8_t data_H = (result >> 8) & 0xFF;
	uint8_t data_L = result & 0xFF;

	S_transmit_message(data, slave_adress, 0x00, 0x57, data_H, data_L,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart6, S_RxData, 1);
}

void S_Step_remaining_steps_Response(uint8_t *data, uint8_t slave_adress, uint16_t remaining_steps){

	uint8_t data_H = (remaining_steps >> 8) & 0xFF;
	uint8_t data_L = remaining_steps & 0xFF;

	S_transmit_message(data, slave_adress, 0x00, 0x97, data_H, data_L,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart6, S_RxData, 1);
}

void S_Step_direction_Response(uint8_t *data, uint8_t slave_adress, uint8_t Step_direction){ // DIR_CW = 0 or DIR_CCW = 1
	S_transmit_message(data, slave_adress, 0x00, 0x77, 0x00, Step_direction,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart6, S_RxData, 1);
}

void S_Servo1_angle_Response(uint8_t *data, uint8_t slave_adress, uint8_t angle){ //angle < 360

	S_transmit_message(data, slave_adress, 0x00, 0x17, 0x00, angle,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart6, S_RxData, 1);

}

void S_Servo2_angle_Response(uint8_t *data, uint8_t slave_adress, uint8_t angle){ //angle < 360

	S_transmit_message(data, slave_adress, 0x00, 0xB7, 0x00, angle,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart6, S_RxData, 1);

}
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  STEPPERS_Init_TMR(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_UARTEx_ReceiveToIdle_IT(&huart6, S_RxData, 1);
  HAL_Delay(2000);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART6){ //slave1


		if((S_is_start_byte == 0) & (S_RxData[0] == 0xFF) &(S_is_error == 0)){ // start byte arrived
			S_is_start_byte = 1;
			S_counter++;;

		}
		else if((S_is_start_byte == 1) & (S_RxData[0] == 0x01) & (S_counter == 1) & (S_is_error == 0)){ // adress belongs to the slave
			S_is_adress_me = 1;
			S_counter++;

		}
		else if((S_is_start_byte == 1) & (S_is_adress_me == 0) & (S_counter > 0) & (S_counter < 8) & (S_is_error == 0)){ // adress does not belong to the slave

			if(S_counter == 6){ // end of the message
				S_is_error = 0;
				S_counter = 0;
				S_is_start_byte = 0;
				S_is_adress_me = 0;
			}
			else{
				S_counter++;
			}
		}
		else if((S_is_start_byte == 1) & (S_is_adress_me == SLAVE_ADRESS1) & (S_is_error == 0) & (S_counter > 1) & (S_counter < 8)){ // take the message whose adress belongs to slave

			switch(S_counter){
			case 2:
				S_command_H = S_RxData[0]; // take the high byte of command
				S_counter++;
				break;

			case 3:
				S_command_L = S_RxData[0];// take the low byte of command
				S_counter++;
				break;

			case 4:
				S_data_H = S_RxData[0]; // take the high byte of data
				S_counter++;
				break;

			case 5:
				S_data_L = S_RxData[0]; // take the low byte of data
				S_counter++;
				break;

			case 6:// take the checksum
				val = S_command_L;
				S_checksum = S_RxData[0];
				if(check_sum(S_command_H,S_command_L,S_data_H,S_data_L, SLAVE_ADRESS1) == S_checksum){ //check
					S_is_error = 0; //end of the message
					S_counter = 0;
					S_is_start_byte = 0;
					S_is_adress_me = 0;
					//execute the command
					switch(S_command_L){

					case 0x1C:

						S_Servo1_angle = S_data_L;
						S_Servo1_angle_value = (int)(350 + 2000*(S_Servo1_angle/180.0));
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, S_Servo1_angle_value);
						HAL_Delay(20);
						break;

					case 0x1F:
						S_Servo1_angle_Response(S_TxData, SLAVE_ADRESS1, S_Servo1_angle);

						break;


					case 0x5C:
						S_step_rpm_speed = map(S_data_H, S_data_L, 14, 0);
						STEPPER_SetSpeed(STEPPER_MOTOR1, S_step_rpm_speed);
						break;

					case 0x5F:
						S_Step_RPM_Response(S_TxData, SLAVE_ADRESS1, S_step_rpm_speed);
						break;

					case 0x9C:
						S_step_remaining_steps = ((uint16_t)S_data_H << 8) | S_data_L;
						STEPPER_Step_NonBlocking(STEPPER_MOTOR1, S_step_remaining_steps, S_Step_direction);
						break;

					case 0x9F:
						STEPPER_GET_STEP(STEPPER_MOTOR1, & S_step_remaining_steps);
						S_Step_remaining_steps_Response(S_TxData, SLAVE_ADRESS1, S_step_remaining_steps);

						break;

					case 0x7C:
						S_Step_direction = S_data_L;
						STEPPER_Step_NonBlocking(STEPPER_MOTOR1, 0, S_Step_direction);

						break;

					case 0x7F:
						STEPPER_GET_DIR(STEPPER_MOTOR1, &S_Step_direction);
						S_Step_direction_Response(S_TxData, SLAVE_ADRESS1, S_Step_direction);

						break;
					case 0xBC:
						S_Servo2_angle = S_data_L;
						S_Servo2_angle_value = (int)(300 + 2000*(S_Servo2_angle/180.0));
						__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (int)S_Servo2_angle_value);
						break;

					case 0xBF:
						S_Servo2_angle_Response(S_TxData, SLAVE_ADRESS1, S_Servo2_angle);
						HAL_Delay(20);
						break;

					default:

						break;

					}


					//S_transmit_message(S_TxData, 0x01,0x00,0x4F,0xAA,0xCC,7);
				}
				else{ //error in transmission

					S_is_error = 1; //end of the message
					S_counter = 0;
					S_is_start_byte = 0;
					S_is_adress_me = 0;
				}

				break;

			default:// error in the message
				S_is_error = 1;
				S_counter = 0;
				S_is_start_byte = 0;
				S_is_adress_me = 0;
				break;
			}
		}
		else{

			// error in the message
			S_is_error = 1;

		}
		HAL_UARTEx_ReceiveToIdle_IT(&huart6, S_RxData, 1);

	}

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == USART6) {

    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);

        // Send completed
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3){
		STEPPER_TMR_OVF_ISR(&htim3);
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
