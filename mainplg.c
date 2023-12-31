///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2023 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//
///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#define ARM_MATH_CM4
//#include "arm_math.h"
//
///* USER CODE END Includes */
//
///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */
//
//
//
///* USER CODE END PTD */
//
///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//#define TS_CAL1 (uint16_t*) 0x1FFF75A8
//#define TS_CAL2 (uint16_t*) 0x1FFF75CA
//#define VREFINT (uint16_t*) 0x1FFF75AA
//
///* USER CODE END PD */
//
///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */
//
///* USER CODE END PM */
//
///* Private variables ---------------------------------------------------------*/
//ADC_HandleTypeDef hadc1;
//
//DAC_HandleTypeDef hdac1;
//
//TIM_HandleTypeDef htim2;
//
///* USER CODE BEGIN PV */
//char status;
//uint32_t raw_data;	// data from ADC
///* USER CODE END PV */
//
///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_TIM2_Init(void);
//static void MX_DAC1_Init(void);
//static void MX_ADC1_Init(void);
///* USER CODE BEGIN PFP */
//
///* USER CODE END PFP */
//
///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//
//
//
///**
// * This function calculates the reference voltage
// *
// */
//int readVRef() {
//	// ADC conversion by polling as per data sheet instructions (pg 104 of HAL driver user manual)
//	HAL_ADC_Start(&hadc1); // pg 107 of HAL driver manual
//	while (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {} // pg 142, 69 107 of HAL driver manual; HAL_MAX_DELAY means infinite poll until successful
//	raw_data = HAL_ADC_GetValue(&hadc1); // pg 110 107 of HAL driver manual
//	HAL_ADC_Stop(&hadc1); // pg 107 107 of HAL driver manual
//
//	// calculate v_ref
//	//float v_ref = 3000.0f * (*VREFINT) / raw_data;
//	int v_ref = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw_data, ADC_RESOLUTION_12B); // pg 129 of HAL driver manual
//
//	return v_ref;
//}
//
///**
// * This function reinitializes the ADC for reading VRef
// *
// */
//void ADC_reconfig_vref() {
//	ADC_ChannelConfTypeDef sConfig = {0};
//
//	sConfig.Channel = ADC_CHANNEL_VREFINT;
//	sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
//	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//	{
//	  Error_Handler();
//	}
//}
//
///**
// * This function calculates the temperature
// *
// */
//float readTemp(float v_ref) {
//	// ADC conversion by polling as per data sheet instructions (pg 104 of HAL driver user manual)
//	HAL_ADC_Start(&hadc1); // pg 107 of HAL driver manual
//	while (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) != HAL_OK) {} // pg 142, 69 107 of HAL driver manual; HAL_MAX_DELAY means infinite poll until successful
//	raw_data = HAL_ADC_GetValue(&hadc1); // pg 110 107 of HAL driver manual
//	HAL_ADC_Stop(&hadc1); // pg 107 107 of HAL driver manual
//
//	// page 130 of HAL driver manual for this function
//	//float temp = __HAL_ADC_CALC_TEMPERATURE(v_ref, raw_data, ADC_RESOLUTION_12B);
//
//	// TS_CAL1 taken @ 30 degrees C and TS_CAL2 taken @ 130 degrees C, both w/ Vref = 3.0 (pg 44 of datasheet)
//	// Hence (130.0 - 30.0) and (3.3/3.0)
//	// Equation from page 689 of long datasheet
//	float temp = ((130.0 - 30.0)/(*TS_CAL2 - *TS_CAL1)) * ((v_ref/3000.0)*raw_data - (int) *TS_CAL1) + 30.0;
//
//
//	return temp;
//}
//
///**
// * This function reinitializes the ADC for reading temp
// *
// */
//void ADC_reconfig_temp() {
//	ADC_ChannelConfTypeDef sConfig = {0};
//
//	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
//	sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5; // max for accuracy
//	if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) !=HAL_OK){
//		Error_Handler();
//	}
//}
//
//int _write(int file, char *ptr, int len) {
//	int DataIdx;
//	for (DataIdx = 0; DataIdx < len; DataIdx++) {
//		ITM_SendChar(*ptr++);
//	}
//	return len;
//}
//
///* USER CODE END 0 */
//
///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */
//	int v_ref;
//	float temp;
//  /* USER CODE END 1 */
//
//  /* MCU Configuration--------------------------------------------------------*/
//
//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();
//
//  /* USER CODE BEGIN Init */
//
//  /* USER CODE END Init */
//
//  /* Configure the system clock */
//  SystemClock_Config();
//
//  /* USER CODE BEGIN SysInit */
//
//  /* USER CODE END SysInit */
//
//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_TIM2_Init();
//  MX_DAC1_Init();
//  MX_ADC1_Init();
//  /* USER CODE BEGIN 2 */
////  start timer
////  HAL_TIM_Base_Start_IT(&htim2);
//  /* USER CODE END 2 */
//
//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//
//	uint32_t triangle = 0;
//	uint32_t saw = 0;
//
//	uint32_t upT = 1;
//
//	float sine = 0;
//	float sineX = 0;
//	float pi = 3.1415;
//
//
//	  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED); // pg 104 optional instruction to improve accuracy **note that hadc1 is generated handle for ADC
//	  v_ref = readVRef();
//  while (1)
//  {
//    /* USER CODE END WHILE */
//
//    /* USER CODE BEGIN 3 */
//
//	// turn LED ON on button press
//	if (HAL_GPIO_ReadPin(BUTTON_GPIO_Port, BUTTON_Pin) == 0) {
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	} else {
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	}
//
//	if (saw < 10){
//		saw += 1;
////		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 9000*saw);
//	} else {
//		saw = 0;
////		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 9000*saw);
//	}
//
//	if (upT == 1){
//		triangle += 1;
////		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 9000*triangle);
//	}
//
//	else if (upT == 0){
//		triangle -= 1;
////		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 9000*triangle);
//	}
//
//	if (triangle == 0){
//		upT = 1;
//	}
//
//	if (triangle == 10){
//		upT = 0;
//	}
//
//	sine = 1 + arm_sin_f32(sineX/8.0 * pi);
//
//	sineX += 1;
//	if (sineX > 32.0){
//		sineX = 0.0;
//	}
//
////	if (status == 0) { // calculate vref
////			ADC_reconfig_vref();
////			v_ref = readVRef();
////
////
////	} else { // calculate temperature
//		ADC_reconfig_vref();
//		v_ref = readVRef();
//		ADC_reconfig_temp();
//		temp = readTemp(v_ref);
//
////	}
//
//
////	if (sineX > 2){
////		sineX = 0;
////	}
//
//	//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 9000*sine);
//
//
//	HAL_Delay(1);
//
//
//  }
//  /* USER CODE END 3 */
//}
//
///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//  /** Configure the main internal regulator output voltage
//  */
//  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSICalibrationValue = 0;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
//  RCC_OscInitStruct.PLL.PLLM = 1;
//  RCC_OscInitStruct.PLL.PLLN = 60;
//  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
//  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//
//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}
//
///**
//  * @brief ADC1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_ADC1_Init(void)
//{
//
//  /* USER CODE BEGIN ADC1_Init 0 */
//
//  /* USER CODE END ADC1_Init 0 */
//
//  ADC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN ADC1_Init 1 */
//
//  /* USER CODE END ADC1_Init 1 */
//
//  /** Common config
//  */
//  hadc1.Instance = ADC1;
//  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
//  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
//  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
//  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//  hadc1.Init.LowPowerAutoWait = DISABLE;
//  hadc1.Init.ContinuousConvMode = DISABLE;
//  hadc1.Init.NbrOfConversion = 1;
//  hadc1.Init.DiscontinuousConvMode = DISABLE;
//  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//  hadc1.Init.DMAContinuousRequests = DISABLE;
//  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//  hadc1.Init.OversamplingMode = DISABLE;
//  if (HAL_ADC_Init(&hadc1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure Regular Channel
//  */
//  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
//  sConfig.SingleDiff = ADC_SINGLE_ENDED;
//  sConfig.OffsetNumber = ADC_OFFSET_NONE;
//  sConfig.Offset = 0;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN ADC1_Init 2 */
//
//  /* USER CODE END ADC1_Init 2 */
//
//}
//
///**
//  * @brief DAC1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_DAC1_Init(void)
//{
//
//  /* USER CODE BEGIN DAC1_Init 0 */
//
//  /* USER CODE END DAC1_Init 0 */
//
//  DAC_ChannelConfTypeDef sConfig = {0};
//
//  /* USER CODE BEGIN DAC1_Init 1 */
//
//  /* USER CODE END DAC1_Init 1 */
//
//  /** DAC Initialization
//  */
//  hdac1.Instance = DAC1;
//  if (HAL_DAC_Init(&hdac1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** DAC channel OUT1 config
//  */
//  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
//  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
//  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
//  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
//  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
//  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
//  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** DAC channel OUT2 config
//  */
//  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN DAC1_Init 2 */
//
//  /* USER CODE END DAC1_Init 2 */
//
//}
//
///**
//  * @brief TIM2 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_TIM2_Init(void)
//{
//
//  /* USER CODE BEGIN TIM2_Init 0 */
//
//  /* USER CODE END TIM2_Init 0 */
//
//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//
//  /* USER CODE BEGIN TIM2_Init 1 */
//
//  /* USER CODE END TIM2_Init 1 */
//  htim2.Instance = TIM2;
//  htim2.Init.Prescaler = 40000;
//  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 3000;
//  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
//  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
//  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
//  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
//  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN TIM2_Init 2 */
//
//  /* USER CODE END TIM2_Init 2 */
//
//}
//
///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
///* USER CODE BEGIN MX_GPIO_Init_1 */
///* USER CODE END MX_GPIO_Init_1 */
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : BUTTON_Pin */
//  GPIO_InitStruct.Pin = BUTTON_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LED_Pin */
//  GPIO_InitStruct.Pin = LED_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);
//
///* USER CODE BEGIN MX_GPIO_Init_2 */
///* USER CODE END MX_GPIO_Init_2 */
//}
//
///* USER CODE BEGIN 4 */
///**
//  * @brief  Interrupt handler for TIM2; toggles LED.
//  * @retval None
//  */
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//}
///* USER CODE END 4 */
//
///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}
//
//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */
