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
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMBER_OF_SLOTS 5
#define STEPS_PER_SLOT 10000
#define RELAY_ON_DURATION 100 // Duration in milliseconds
#define DEBOUNCE_TIME_MS 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint8_t received_slot[1] = { 0 };
uint8_t slot_data[NUMBER_OF_SLOTS] = { 0 };
uint8_t slotCounter = 0;
uint32_t numberofslotsrecived = 0;
uint32_t numberofslotsprocessed = 0;
uint8_t recivedNewData = 0;
volatile uint32_t last_interrupt_time = 0;
// UART buffer for debug messages
char debug_buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Delay in microseconds
void HAL_Delay_us(uint16_t us) {
	uint32_t start = DWT->CYCCNT; // Get the current cycle count
	uint32_t ticks = us * (SystemCoreClock / 1000000);
	while ((DWT->CYCCNT - start) < ticks)
		; // Wait for the specified number of ticks
}
// check new data
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		if (received_slot[0] <= NUMBER_OF_SLOTS) {
			recivedNewData = 1;
			numberofslotsrecived++;
			slot_data[slotCounter] = received_slot[0];
			slotCounter =
					(slotCounter < (NUMBER_OF_SLOTS - 1)) ? slotCounter + 1 : 0;
			send_debug_message("");
		}
		HAL_UART_Receive_IT(&huart2, received_slot, 1); // Restart reception
	}
}
// Function to send debug messages over UART2
void send_debug_message(char *message) {
	HAL_UART_Transmit(&huart2, (uint8_t*) message, strlen(message),
	HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart2, (uint8_t*) "\r\n", strlen("\r\n"),
	HAL_MAX_DELAY);
}
// for delay in useconds
void enable_dwt() {
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0; // Reset counter
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // Enable counter
	}
}
// Function to move conveyor to the specified slot
void processSlots() {
	for (int i = 0; i < STEPS_PER_SLOT; i++) {
		HAL_GPIO_WritePin(STEP_PIN_GPIO_Port, STEP_PIN_Pin, GPIO_PIN_SET);
		HAL_Delay_us(100); // Step pulse high duration
		HAL_GPIO_WritePin(STEP_PIN_GPIO_Port, STEP_PIN_Pin, GPIO_PIN_RESET);
		HAL_Delay_us(30); // Step pulse low duration
	}
	// turns on the relay
	for (uint8_t slot = 1; slot <= NUMBER_OF_SLOTS; slot++) {
		snprintf(debug_buffer, sizeof(debug_buffer),
				"slot_data_before[%d]:%d\n", slot - 1, slot_data[slot - 1]);
		send_debug_message(debug_buffer);
		if (slot_data[slot - 1] == 1) {
			triggerOnSlotRelay(numberofslotsprocessed - (slot - 1));
		}
		if (slot_data[slot - 1] > 0) {
			slot_data[slot - 1] -= 1;
		}
	}
	HAL_Delay(100);
	triggerOfSlotRelays();    // turns of all relays
}
// funtion for triggering relay
void triggerOnSlotRelay(uint8_t slotNumber) {
	switch (slotNumber) {
	case 0:
		HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
		break;
	case 1:
		HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_RESET);
		break;
	default:
		break;
	}
}
//
void triggerOfSlotRelays() {
	HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_SET);
}
// External Interrupt ISR Handler Callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == PB2_Pin) // Check if the interrupt source is PB2_Pin
    {
        uint32_t current_time = HAL_GetTick(); // Get the current system tick
        if ((current_time - last_interrupt_time) >= DEBOUNCE_TIME_MS) // Check debounce condition
        {
            last_interrupt_time = current_time; // Update last interrupt time
            HAL_UART_Transmit(&huart2, (uint8_t*) "DETECTED\r\n", strlen("DETECTED\r\n"), 10); // Send message
        }
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	enable_dwt();
	HAL_UART_Receive_IT(&huart2, received_slot, 1);
	send_debug_message("Enter a slot number..");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (recivedNewData) {
			recivedNewData = 0;
			processSlots();
			numberofslotsprocessed =
					(numberofslotsprocessed < (NUMBER_OF_SLOTS - 1)) ?
							numberofslotsprocessed + 1 : 0;
//			snprintf(debug_buffer, sizeof(debug_buffer),
//					"Number of slots recived : %d  Numberofslotsprocessed : %d \n",
//					numberofslotsrecived, numberofslotsprocessed);
//			send_debug_message(debug_buffer);
			snprintf(debug_buffer, sizeof(debug_buffer),
					"current slot of 1 index : %d \n", numberofslotsprocessed);
			send_debug_message(debug_buffer);
		}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEP_PIN_Pin|DRV_ENB_Pin|RELAY1_Pin|RELAY2_Pin
                          |RELAY3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2_Pin */
  GPIO_InitStruct.Pin = PB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_PIN_Pin DRV_ENB_Pin RELAY1_Pin RELAY2_Pin
                           RELAY3_Pin */
  GPIO_InitStruct.Pin = STEP_PIN_Pin|DRV_ENB_Pin|RELAY1_Pin|RELAY2_Pin
                          |RELAY3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
	while (1) {
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
