/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int Ultrasonic_readings_index, Post_check = 0,datacollected=0;
char buffer[100], Rx_data[2], User_input;
uint8_t icFlag = 0, captureIdx = 0;
uint32_t edge1Time = 0, edge2Time = 0, Echo_time_interval = 0, data;
const float sos = 0.0343 / 2;
float distance, Ultrasonic_data_array[100],min_dist=9999.99,max_dist=0.0;
//uint8_t buffer[1000];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void customdelay(uint32_t usec);
void myprintf(UART_HandleTypeDef *huart, uint32_t out);
int POST();
float readUS();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM4_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
  data=sprintf(buffer, "Welcome to Grad Project\n\r Place a large object 100 mm away from the sensor");
  	myprintf(&huart2, data);
  	HAL_UART_Receive_IT(&huart2, Rx_data, 2);
  	while(Rx_data[0]!=13);

	HAL_UART_Receive_IT(&huart2, Rx_data, 1);
	while (POST() == 1)										//Checking for POST
		;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if (Post_check == 0) {
			data = sprintf(buffer, "Program Terminated");
			myprintf(&huart2, data);
			break;
		} else if (Post_check == 1) {
			data = sprintf(buffer, "Post Success");
			myprintf(&huart2, data);

			data = sprintf(buffer, "Distance :  %.2f", readUS());
			myprintf(&huart2, data);

			Rx_data[0] = '\0';
			//
			//UltraSonic Data collection
			//
			for (Ultrasonic_readings_index = 0; Ultrasonic_readings_index <= 100;
					Ultrasonic_readings_index++) {
				Ultrasonic_data_array[Ultrasonic_readings_index] = readUS();

				//
				//Checking and Discarding data that is not within the range 50 mm and 10000 mm
				//
				if (Ultrasonic_data_array[Ultrasonic_readings_index] < 50
						|| Ultrasonic_data_array[Ultrasonic_readings_index]
								> 1000)
					Ultrasonic_readings_index--;
				//
				//finding the max distance recorded
				//
				if(Ultrasonic_data_array[Ultrasonic_readings_index]>max_dist)
				max_dist=Ultrasonic_data_array[Ultrasonic_readings_index];
				//
				//finding the min distance recorded
				//
				if(Ultrasonic_data_array[Ultrasonic_readings_index]<min_dist)
				min_dist=Ultrasonic_data_array[Ultrasonic_readings_index];
				//
				//checking constantly for user keyboard interrupt if interrupted stop the data collection
				if (Rx_data[0] != '\0'){
					datacollected=Ultrasonic_readings_index;
					break;
				}
				HAL_Delay(100);
			}

			//
			//Printing the Ultrasonic data collected
			//
			for (Ultrasonic_readings_index = 0;
					Ultrasonic_readings_index <= datacollected;
					Ultrasonic_readings_index++) {
				data = sprintf(buffer, "%.2f",
						Ultrasonic_data_array[Ultrasonic_readings_index]);
				myprintf(&huart2, data);
			}
			//
			//Printing the max and min distance recorded
			//
			data = sprintf(buffer, "Max Distance recorded : %.2f mm",
									max_dist);
			myprintf(&huart2, data);
			data = sprintf(buffer, "Min Distance recorded : %.2f mm",
									min_dist);
			myprintf(&huart2, data);

			break;

		}

		//  	HAL_Delay(1000);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 79;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 79;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 0;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Trig_Pin | trig_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : Trig_Pin trig_Pin */
	GPIO_InitStruct.Pin = Trig_Pin | trig_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : In_Pin */
	GPIO_InitStruct.Pin = In_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(In_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void customdelay(uint32_t usec) {
	if (usec < 2)
		usec = 2;
	TIM4->ARR = usec - 1; /*sets the value in the auto-reload register*/
	TIM4->EGR = 1; /*Re-initialises the timer*/
	TIM4->SR &= ~1; 		//Resets the flag
	TIM4->CR1 |= 1; 		//Enables the counter
	while ((TIM4->SR & 0x0001) != 1)
		;
	TIM4->SR &= ~(0x0001);
}
void myprintf(UART_HandleTypeDef *huart, uint32_t out) {

	HAL_UART_Transmit(huart, buffer, out, 1000);
	char spacer[3] = "\r\n";
	HAL_UART_Transmit(huart, (uint8_t*) spacer, 3, 10);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	data = sprintf(buffer, "%c", Rx_data[0]);
	myprintf(&huart2, data);
	if (Rx_data[0] != 13) {
		User_input = Rx_data[0];
	}
	HAL_UART_Receive_IT(&huart2, Rx_data, 1);

}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (captureIdx == 0) //Fisrt edge
			{
		edge1Time = __HAL_TIM_GET_COUNTER(&htim2); //

		captureIdx = 1;
	} else if (captureIdx == 1) //Second edge
			{
		edge2Time = __HAL_TIM_GET_COUNTER(&htim2);
		;
		captureIdx = 0;
		icFlag = 1;
	}

}
float readUS() {
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
	customdelay(10);
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	uint32_t startTick = HAL_GetTick();
	do {
		if (icFlag)
			break;
	} while ((HAL_GetTick() - startTick) < 500);  //500ms
	icFlag = 0;
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
	if (edge2Time > edge1Time) {
		distance = ((edge2Time - edge1Time) + 0.0) * sos;
	} else {
		distance = 0.0;
	}
	return distance * 10;
}
int POST() {

	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
	customdelay(10);
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	uint32_t startTick = HAL_GetTick();
	do {
		if (icFlag)
			break;
	} while ((HAL_GetTick() - startTick) < 500);  //500ms
	icFlag = 0;
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_2);
	if (edge2Time > edge1Time) {
		Echo_time_interval = edge2Time - edge1Time;


		if (Echo_time_interval >= 300 && Echo_time_interval <= 1000) {
			Post_check = 1;
			return 0;
		} else if (Echo_time_interval < 300) {
			data =
					sprintf(buffer,
							"Object is too close\n\rPress R to retry post or Q to Quit");
			myprintf(&huart2, data);

			while (Rx_data[0] != 13) {

			};
			data = sprintf(buffer, "%c", User_input);
			myprintf(&huart2, data);

			if (User_input == 'R') {
				data = sprintf(buffer, "Retrying Post");
				myprintf(&huart2, data);
				Rx_data[0] = 0;
				return 1;
			} else if (User_input == 'Q') {
				data = sprintf(buffer, "Post Failed");
				myprintf(&huart2, data);
				Post_check = 0;
				Rx_data[0] = 0;
				return 0;
			}

		} else if (Echo_time_interval > 1000) {
			data =
					sprintf(buffer,
							"Object is too Far\n\rPress R to retry post or Q to Quit");
			myprintf(&huart2, data);

			while (Rx_data[0] != 13)
				;

			if (User_input == 'R') {
				data = sprintf(buffer, "Retrying Post");
				myprintf(&huart2, data);
				Rx_data[0] = 0;
				return 1;
			} else if (User_input == 'Q') {
				data = sprintf(buffer, "Post Failed");
				myprintf(&huart2, data);
				Post_check = 0;
				Rx_data[0] = 0;
				return 0;
			}

		}
	} else {
		return 1;
	}

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
