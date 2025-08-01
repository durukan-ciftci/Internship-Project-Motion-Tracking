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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t M_RxData[1];
uint8_t M_TxData[7];
uint8_t M_last_trasmitted_adress = 0x00; // master listens the last adress it transmitted to
uint8_t M_command_H;
uint8_t M_command_L;
uint8_t M_data_H;
uint8_t M_data_L;

uint8_t M_counter = 0;
uint8_t M_is_start_byte = 0;
uint8_t M_is_adress_me = 0;
uint8_t M_is_error = 0;
uint8_t M_checksum = 0;

uint8_t M_step_rpm_speed = 5;

uint16_t M_step_remaining_steps;

uint8_t M_Step_direction = DIR_CW;

uint8_t M_Servo1_angle = 0;
uint8_t M_Servo2_angle = 0;

uint8_t k;
uint16_t Distance;

uint8_t camera_x_pos_H;
uint8_t camera_y_pos_H;
uint8_t camera_x_pos_L;
uint8_t camera_y_pos_L;

uint16_t camera_x_pos;
uint16_t camera_y_pos;

uint16_t steps = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

void M_transmit_message(uint8_t *data, uint8_t slave_adress, uint8_t command_H, uint8_t command_L, uint8_t data_H, uint8_t data_L, uint8_t size){
	generate_message(data, slave_adress, command_H, command_L, data_H, data_L);
	calculate_checksum(data);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart1, data, size);
	M_last_trasmitted_adress = slave_adress;
}

void M_Step_RPM_Write(uint8_t *data, uint8_t slave_adress, uint8_t speed){ //speed is between 14-0

	uint16_t result = (uint16_t)(((float)speed / 14.0f) * 65535.0f + 0.5f);
	uint8_t data_H = (result >> 8) & 0xFF;
	uint8_t data_L = result & 0xFF;
	M_transmit_message(data, slave_adress, 0x00, 0x5C, data_H, data_L,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);
}

void M_Step_RPM_Read(uint8_t *data, uint8_t slave_adress){ //speed is between

	M_transmit_message(data, slave_adress, 0x00, 0x5F, 0x00, 0x00,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);
}

void M_Step_remaining_steps_Write(uint8_t *data, uint8_t slave_adress, uint16_t remaining_steps){ // input is added to the number of steps remaining
	uint8_t data_H = (remaining_steps >> 8) & 0xFF;
	uint8_t data_L = remaining_steps & 0xFF;
	M_transmit_message(data, slave_adress, 0x00, 0x9C, data_H, data_L,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);
}

void M_Step_remaining_steps_Read(uint8_t *data, uint8_t slave_adress){ // reads num of steps remaining

	M_transmit_message(data, slave_adress, 0x00, 0x9F, 0x00, 0x00,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);
}

void M_Step_direction_Write(uint8_t *data, uint8_t slave_adress, uint8_t Step_direction){ // DIR_CW = 0 or DIR_CCW = 1

	M_transmit_message(data, slave_adress, 0x00, 0x7C, 0x00, Step_direction,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);
}

void M_Step_direction_Read(uint8_t *data, uint8_t slave_adress){ // reads num of steps remaining

	M_transmit_message(data, slave_adress, 0x00, 0x7F, 0x00, 0x00,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);
}

void M_Servo1_angle_Write(uint8_t *data, uint8_t slave_adress, uint8_t angle){

	M_transmit_message(data, slave_adress, 0x00, 0x1C, 0x00, angle,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);

}

void M_Servo1_angle_Read(uint8_t *data, uint8_t slave_adress){
	M_transmit_message(data, slave_adress, 0x00, 0x1F, 0x00, 0x00,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);

}

void M_Servo2_angle_Write(uint8_t *data, uint8_t slave_adress, uint8_t angle){

	M_transmit_message(data, slave_adress, 0x00, 0xBC, 0x00, angle,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);

}

void M_Servo2_angle_Read(uint8_t *data, uint8_t slave_adress){
	M_transmit_message(data, slave_adress, 0x00, 0xBF, 0x00, 0x00,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);

}

void M_Sensor_distance_Read(uint8_t *data, uint8_t slave_adress){
	M_transmit_message(data, slave_adress, 0x00, 0x2F, 0x00, 0x00,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);

}

void M_Camera_x_pozition_Read(uint8_t *data, uint8_t slave_adress){
	M_transmit_message(data, slave_adress, 0x00, 0x4F, 0x00, 0x00,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);

}

void M_Camera_y_pozition_Read(uint8_t *data, uint8_t slave_adress){
	M_transmit_message(data, slave_adress, 0x00, 0x6F, 0x00, 0x00,7);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  M_Step_RPM_Write(M_TxData, SLAVE_ADRESS1, 10);
  HAL_Delay(50);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  M_Camera_x_pozition_Read(M_TxData, SLAVE_ADRESS3);
	  HAL_Delay(150);
	  M_Sensor_distance_Read(M_TxData, SLAVE_ADRESS2);
	  HAL_Delay(150);

	  if(camera_x_pos > 1050)
	  {
		  M_Step_direction_Write(M_TxData, SLAVE_ADRESS1, 0);
		  HAL_Delay(20);
		  steps = (camera_x_pos - 950) * 30 / 950;
		  M_Step_remaining_steps_Write(M_TxData, SLAVE_ADRESS1, steps);
		  HAL_Delay(100);
	  }
	  else if(camera_x_pos < 850)
	  {
		  M_Step_direction_Write(M_TxData, SLAVE_ADRESS1, 1);
		  HAL_Delay(20);
		  steps = (950 - camera_x_pos) * 30 / 950;
		  M_Step_remaining_steps_Write(M_TxData, SLAVE_ADRESS1, steps);
		  HAL_Delay(100);
	  }
	  else
	  {
		  M_Step_remaining_steps_Write(M_TxData, SLAVE_ADRESS1, 0);
		  HAL_Delay(100);
	  }


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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  huart1.Init.BaudRate = 115200;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

	if(huart->Instance == USART1){	//master

		if((M_is_start_byte == 0) & (M_RxData[0] == 0xFF) &(M_is_error == 0)){ // start byte arrived
			M_is_start_byte = 1;
			M_counter++;;
			}
		else if((M_is_start_byte == 1) & (M_RxData[0] == M_last_trasmitted_adress) & (M_counter == 1) & (M_is_error == 0)){ // adress belongs to the slave
			M_is_adress_me = 1;
			M_counter++;
			}
		else if((M_is_start_byte == 1) & (M_is_adress_me == 0) & (M_counter > 0) & (M_counter < 8) & (M_is_error == 0)){ // adress does not belong to the slave
			if(M_counter == 7){ // end of the message
				M_is_error = 0;
				M_counter = 0;
				M_is_start_byte = 0;
				M_is_adress_me = 0;
				}
			else{
				M_counter++;
				}
			}
		else if((M_is_start_byte == 1) & (M_is_adress_me == 1) & (M_is_error == 0) & (M_counter > 1) & (M_counter < 8)){ // take the message whose adress belongs to slave
			switch(M_counter){
			case 2:
				M_command_H = M_RxData[0]; // take the high byte of command
				M_counter++;
				break;

			case 3:
				M_command_L = M_RxData[0];// take the low byte of command
				M_counter++;
				break;

			case 4:
				M_data_H = M_RxData[0]; // take the high byte of data
				M_counter++;
				break;

			case 5:

				M_data_L = M_RxData[0]; // take the low byte of data
				M_counter++;
				break;

			case 6:// take the check sum
				M_checksum = M_RxData[0];
				if(check_sum(M_command_H,M_command_L,M_data_H,M_data_L, M_last_trasmitted_adress) == M_checksum){ //check
					switch(M_command_L){

					case 0x17:
						M_Servo1_angle = M_data_L;
						break;

					case 0x57:
						M_step_rpm_speed = map(M_data_H,M_data_L,14,0);
						break;

					case 0x97:
						M_step_remaining_steps = ((uint16_t)M_data_H << 8) | M_data_L;

						break;

					case 0x77:
						M_Step_direction = M_data_L;
						break;

					case 0xB7:
						M_Servo2_angle = M_data_L;
						break;

					case 0x27:
						Distance = M_data_L;
						break;

					case 0x47:
						camera_x_pos_H = M_data_H;
						camera_x_pos_L = M_data_L;
						camera_x_pos = ((uint16_t)camera_x_pos_H << 8) | camera_x_pos_L;
						break;

					case 0x67:
						camera_y_pos_H = M_data_H;
						camera_y_pos_L = M_data_L;
						camera_y_pos = ((uint16_t)camera_y_pos_H << 8) | camera_y_pos_L;
						break;

					default:
						break;


					}

					M_is_error = 0; //end of the message
					M_counter = 0;
					M_is_start_byte = 0;
					M_is_adress_me = 0;
					}
				else{ //error in transmission
					M_is_error = 1; //end of the message
					M_counter = 0;
					M_is_start_byte = 0;
					M_is_adress_me = 0;
					}
				break;

			default:// error in the message
				M_is_error = 1;
				M_counter = 0;
				M_is_start_byte = 0;
				M_is_adress_me = 0;
				break;
			}
		}
		else{
			// error in the message
			M_is_error = 1;
		}
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);

	}
	else{
		M_is_error = 1;
		HAL_UARTEx_ReceiveToIdle_IT(&huart1, M_RxData, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
    	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
        // Send completed
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
