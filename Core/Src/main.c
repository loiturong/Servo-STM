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
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct GPIO_Name {
  GPIO_TypeDef *port;
  uint16_t pin;
} GPIO_Name;

typedef struct {
  // previous state
  float  Ui_previous;
  float  Ud_previous;
  float  ek_previous;

  // control parameters
  float  Ts;
  float  Kp;
  float  Ki;
  float  Kd;
  float  alpha;
  float  Kb;

  // Don't want this to be modified
  const bool  allow_filter;
  const bool  allow_windup;
} PIDState;

typedef struct {
  float Total_PID;       // Final saturated output
  uint8_t dir;          // Direction of control, 1: forward, 0: reverse
} PIDOutputType;

typedef struct {
  int previous_encoder_state;
  int encoder_res;
  long revolutions;
  int counter;

  int Counter_Vec;
  float Current_Vec;
} EncoderState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI        3.14159
#define MAX_SPEED 331  // rad/s
#define PI2REV    (PI / 2000)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// macros for change duty cycle in control program
#define set_duty(duty) __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty);

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
// naming pin
GPIO_Name Encoder_Channel_A = {GPIOB, GPIO_PIN_4};
GPIO_Name Encoder_Channel_B = {GPIOB, GPIO_PIN_6};
GPIO_Name DIR_Port = {GPIOA, GPIO_PIN_8};

// State of Encoder
EncoderState encoder = {.previous_encoder_state = 0b00,
                          .encoder_res = 4000,
                          .revolutions = 0,
                          .counter = 0,
                          .Counter_Vec = 0,
                          .Current_Vec = 0};

// initialize the PID controller parameter
PIDState velocity_pid = {
  .Ui_previous = 0,
  .Ud_previous = 0,
  .ek_previous = 0,
  .Ts = (float)0.01,
  .Kp = 1,
  .Ki = 1,
  .Kd = 0,
  .alpha = 0,
  .Kb = 1,
  .allow_filter = false,
  .allow_windup = true
};

uint8_t* Rx_data;
// Set points
float set_point_velocity = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim5);

  HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
  set_point_velocity = 100;
  HAL_UART_Transmit(&huart1, "V0\r\n", 4, 8);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 999;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 239;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Digital PID function for Discrete system
  * @param pid_state: PID State that store previous state like ek, Ui, Ud (if filter is allowed) and control parameters of that PID like Kp, Ki, Kd, filter and anti wind-up param like alpha, Kb. This variable is made static so that the previous state can be directly modified inside this function.
  * @param set_point: This variable have type double. In Position control, this value is the relative distance with respect to the base point (Global base), therefore, it should have type float to represent real number. In velocity control, this values feature speed we want motor to reach, in such scenarios that speed of motor is controlled by Duty cycle of H-Bridge, we would want this to be the value of TIM->CCRx (uint16_t).
  * @param current: same as set_point, store the current velocity or relative distance.
  * @param ref_val: reference output for PID. In Position control, we have the output of this PID to be the speed (rad/s) and in Velocity control, this value is the maximum CCR (TIM->ARR)
  * @retval PIDOutputType
  */
PIDOutputType digital_PID(PIDState *pid_state,          // Store previous state of Digital PID and control params
                          const float set_point,         // set point for system
                          const float current,           // current value (velocity or distance)
                          const float ref_val)           // reference value (speed or Duty)
{
  // compute error
  const float ek = set_point - current;

  // compute partial terms
  const float Up = pid_state->Kp * ek;
  float Ui = pid_state->Ui_previous + pid_state->Ki * ek * pid_state->Ts;
  float Ud = pid_state->Kd * (ek - pid_state->ek_previous) / pid_state->Ts;

  // Activate filter (if allowed)
  if (pid_state->allow_filter)
  {
    Ud = pid_state->Ud_previous * (1 - pid_state->alpha) + pid_state->alpha * Ud;
    pid_state->Ud_previous = (float)Ud;
  }

  // total compute value
    float Total_PID = Up + Ui + Ud;

  // Activate anti-windup (if allowed)
  if (pid_state->allow_windup)
  {
    double e_reset = 0;
    if (Total_PID < -ref_val)
    {
      e_reset = -ref_val - Total_PID;
    }
    else if (Total_PID > ref_val)
    {
      e_reset = Total_PID - ref_val;
    }
    Ui = (float)(Ui + pid_state->Kb * e_reset * pid_state->Ts);
  }

  // update state for next call
  pid_state->Ui_previous = Ui;
  pid_state->ek_previous = ek;

  // Saturate the output
  uint8_t _dir = 1;   // forward direction
  if (Total_PID < 0)
  {
    Total_PID = -Total_PID;
    _dir ^= 1;
  }
  if (Total_PID > ref_val)
    Total_PID = ref_val;

  return (PIDOutputType){Total_PID, _dir};
}


// external interrupt callback to update the distance
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Handle EXTERNAL interrupts for Channel A and Channel B (PB4, PB6)
  if (GPIO_Pin == GPIO_PIN_4)
  {
    // Update State of encoder
    char encoder_state =
            ((HAL_GPIO_ReadPin(Encoder_Channel_A.port, Encoder_Channel_A.pin) == GPIO_PIN_SET) << 1) |
            ((HAL_GPIO_ReadPin(Encoder_Channel_B.port, Encoder_Channel_B.pin) == GPIO_PIN_SET) << 0);

    // Check the transition between previous and current states to determine rotation direction
    switch (encoder_state)
    {
    case 0b00:
      if (encoder.previous_encoder_state == 0b01)       // 0 | 1 --> 0 | 0
          encoder.counter++;
      else if (encoder.previous_encoder_state == 0b10)  // 1 | 0 --> 0 | 0
          encoder.counter--;
      break;

    case 0b01:
      if (encoder.previous_encoder_state == 0b11)       // 1 | 1 --> 0 | 1
          encoder.counter++;
      else if (encoder.previous_encoder_state == 0b00)  // 0 | 0 --> 0 | 1
          encoder.counter--;
      break;

    case 0b10:
      if (encoder.previous_encoder_state == 0b00)       // 0 | 0 --> 1 | 0
          encoder.counter++;
      else if (encoder.previous_encoder_state == 0b11)  // 1 | 1 --> 1 | 0
          encoder.counter--;
      break;

    case 0b11:
      if (encoder.previous_encoder_state == 0b10)       // 1 | 0 --> 1 | 1
          encoder.counter++;
      else if (encoder.previous_encoder_state == 0b01)  // 0 | 1 --> 1 | 1
          encoder.counter--;
      break;

    // Should not fall into this case
    default:
        break;
    }
    // Handle out of range (Encoder's Resolution)
    if (encoder.counter >= encoder.encoder_res)
    {
      encoder.counter = 0;
      encoder.revolutions ++;
    }
    if (encoder.counter <= -encoder.encoder_res)
    {
      encoder.counter = 0;
      encoder.revolutions --;
    }
    // Update the previous state for the next rotation step
    encoder.previous_encoder_state = encoder_state;
    encoder.Counter_Vec++;
  }
}

void compute_velocity(void)
{
    const int CntTmp = encoder.Counter_Vec;
    encoder.Counter_Vec = 0;

    encoder.Current_Vec = (float)((float)CntTmp * PI / 10);
}

// TIM4 interrupt to run PI on velocity
// TIM5 interrupt to run PID on distance
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)     // transmit data using TIM5 Interrupt
    {
      // Compute velocity
      compute_velocity();

    	// Compute PID
    	PIDOutputType vec_result = digital_PID(&velocity_pid, set_point_velocity, encoder.Current_Vec, 331);

    	// set direction
    	if (vec_result.dir == 1)
    	{
    		HAL_GPIO_WritePin(DIR_Port.port, DIR_Port.pin, GPIO_PIN_SET);
    	}
    	else
    	{
    		HAL_GPIO_WritePin(DIR_Port.port, DIR_Port.pin, GPIO_PIN_RESET);
    	}

    	// set velocity
    	set_duty((int)(vec_result.Total_PID * 1599 / 331))

    	// transmit the velocity
    	char buffer_vec[0xF] = {0};
    	sprintf(buffer_vec, "V%.2f\r\n", vec_result.Total_PID * 1599 / 331);
    	HAL_UART_Transmit(&huart1, buffer_vec, strlen(buffer_vec), 8);
    }
}

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if (huart->Instance == USART1)
//     {
//         switch (Rx_data[0]) {
//             case 'r':
//                 // Start
//                 break;
//             case 'e':
//                 // Stop
//                 break;
//             case 'g':
//                 // resume (after pause)
//                 break;
//             case 'f':
//                 // pause
//                 break;
//             case 'a':
//                 // set vec and pos
//                 int i;
//                 uint8_t buffer_vec[0xF] = {0};
//                 i = 0;
//                 while (buffer_vec[i] != 's')
//                     HAL_UART_Receive(&huart1, &buffer_vec[i++], 1, 5);
//
//                 buffer_vec[0] = 0;
//                 i = 0;
//                 while (buffer_vec[i] != 'v')
//                     HAL_UART_Receive(&huart1, &buffer_vec[i++], 1, 5);
//
//                 set_point_velocity = (float)strtod((char *)buffer_vec + 1, NULL);
//                 break;
//         }
//     }
// }
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
