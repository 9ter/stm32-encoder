/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define MOTOR_LEFT_PIN GPIO_PIN_2
#define MOTOR_RIGHT_PIN GPIO_PIN_3
#define MOTOR_PORT GPIOA
#define IR_SENSOR_PIN GPIO_PIN_4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
unsigned char  RX_BUFFER[10];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define BUFFER_SIZE 100
char rx_data[] = "";



void myCallback(UART_HandleTypeDef *huart);
void convertdata_6digit(char* data);
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
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  int speed_motor = 255;

  int gap = 10;

  int point0 =0;
  int point1 =36;
  int point2 =72;
  int point3 =108;
  int point4 =144;
  int point5 =180;
  int point6 =216;
  int point7 =252;
  int point8 =288;
  int point9 =324;

int input = 0;

  uint8_t rx_buffer[4];
  uint8_t serial_input[4] = "0";

  char bufdata[60];

  char aoaBuffer[10]; // เปลี่ยนเป็นอาร์เรย์ char เพื่อเ�?็บข้อมูล AOA
  char aoaBuffer_data[10];
  char aoaBuffer_data_value[10];
  int aoaIndex = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  TIM3->CCR2 = speed_motor;

  while (1)
  {

	  HAL_StatusTypeDef status_huart2 = HAL_UART_Receive(&huart2, (uint8_t *)&serial_input, 1, 0);

	  if (status_huart2 == HAL_OK || status_huart2 == HAL_TIMEOUT) {
	      // Data received from huart2 or timeout occurred
		  char* string_serial = (char*) &serial_input;
		  int serial_int = atoi(string_serial);

		  /*if(serial_int == 1){
			  HAL_UART_Transmit(&huart2, string_serial,sizeof(string_serial), 10);
		  }*/

	      // Attempt to receive data from huart6
	      if (HAL_UART_Receive(&huart6, (uint8_t *)&rx_buffer, 1, 10) == HAL_OK) {

	    	  char* string = (char*) &rx_buffer;

	    			if (string[0] == 10)
	    			{

	    			  int sizebuffer = 0;
	    			 for(int a = 6; a< aoaIndex ; a++){
	    				aoaBuffer_data[a-6] = aoaBuffer[a];
	    				sizebuffer++;
	    			 }
	    			 aoaBuffer_data[sizebuffer] = '\0';

	    			 if(sizebuffer == 6){
	    				 aoaBuffer_data_value[0] = '0';
	    				 aoaBuffer_data_value[1] = aoaBuffer_data[0];
	    				 aoaBuffer_data_value[2] = aoaBuffer_data[1];
	    				 aoaBuffer_data_value[3] = aoaBuffer_data[2];
	    				 aoaBuffer_data_value[4] = aoaBuffer_data[3];
	    				 aoaBuffer_data_value[5] = aoaBuffer_data[4];
	    				 aoaBuffer_data_value[6] = aoaBuffer_data[5];
	    			 }
	    			 else if(sizebuffer == 5){
	    				 aoaBuffer_data_value[0] = '0';
	    				 aoaBuffer_data_value[1] = '0';
	    				 aoaBuffer_data_value[2] = aoaBuffer_data[0];
	    				 aoaBuffer_data_value[3] = aoaBuffer_data[1];
	    				 aoaBuffer_data_value[4] = aoaBuffer_data[2];
	    				 aoaBuffer_data_value[5] = aoaBuffer_data[3];
	    				 aoaBuffer_data_value[6] = aoaBuffer_data[4];
	    			 }
	    			 else{
	    				 strcpy(aoaBuffer_data_value,aoaBuffer_data);
	    			 }


	    			 int DDD = ((aoaBuffer_data_value[0]-48) * 100) + ((aoaBuffer_data_value[1]-48) * 10)+ ((aoaBuffer_data_value[2]-48) * 1);
	    			 int DDD2 = ((aoaBuffer_data_value[4]-48) * 100) + ((aoaBuffer_data_value[5]-48) * 10)+ ((aoaBuffer_data_value[6]-48) * 1);

	    			  aoaIndex = 0; // เริ่มต้นตัวชี้อาร์เรย์ใหม่

	    			  //sprintf(bufterkak, "size = %d : data = %s : value = %.2f \n", sizebuffer,aoaBuffer_data_value,AOA );
	    			  sprintf(bufdata, "data = : %d:%d \n",DDD,speed_motor );
	    			 HAL_UART_Transmit(&huart2, bufdata,sizeof(bufdata), 10);
	    			  //HAL_UART_Transmit(&huart2, aoaBuffer_data,sizeof(aoaBuffer_data), 10);

	    			  if(serial_int == 0){
	    				  if(DDD > point0 && DDD < point0+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point0){
	    					   HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					       HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }
	    			  }
	    			  else if(serial_int == 1){
	    				  if(DDD > point1-gap && DDD < point1+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point1){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point1){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }

	    			  }
	    			  else if(serial_int == 2){
	    				  if(DDD > point2-gap && DDD < point2+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point2){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point2){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }
	    			  }
	    			  else if(serial_int == 3){
	    				  if(DDD > point3-gap && DDD < point3+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point3){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point3){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }
	    			  }
	    			  else if(serial_int == 4){
	    				  if(DDD > point4-gap && DDD < point4+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point4){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point4){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }
	    			  }
	    			  else if(serial_int == 5){
	    				  if(DDD > point5-gap && DDD < point5+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point5){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point5){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }
	    			  }
	    			  else if(serial_int == 6){
	    				  if(DDD > point6-gap && DDD < point6+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point6){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point6){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }
	    			  }
	    			  else if(serial_int == 7){
	    				  if(DDD > point7-gap && DDD < point7+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point7){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point7){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }
	    			  }
	    			  else if(serial_int == 8){
	    				  if(DDD > point8-gap && DDD < point8+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point8){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point8){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }
	    			  }
	    			  else if(serial_int == 9){
	    				  if(DDD > point9-gap && DDD < point9+gap){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
	    					  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }else if (DDD > point9){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 1);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 0);
	    				  }else if (DDD < point9){
	    					  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, 0);
    					      HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, 1);
	    				  }
	    			  }

	    			  for(int b = 0; b< sizeof(aoaBuffer_data) ; b++){
	    				  aoaBuffer_data[b] = 0;
	    			  }
	    			}
	    			else if(string[0] == 13){

	    			}
	    			else{
	    				aoaBuffer[aoaIndex++] = string[0];
	    			}
	    		  }
	  } else {
	      // No data received from huart2
	      // Handle the case when no data is received from huart2
	      // You can put additional code here if needed
	  }
  /* USER CODE END 3 */
}
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
  htim3.Init.Prescaler = 84;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 255;
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
  sConfigOC.Pulse = 255;
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
  huart6.Init.BaudRate = 9600;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(mr2_GPIO_Port, mr2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(mr1_GPIO_Port, mr1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : mr2_Pin */
  GPIO_InitStruct.Pin = mr2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(mr2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : mr1_Pin */
  GPIO_InitStruct.Pin = mr1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(mr1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ir_Pin */
  GPIO_InitStruct.Pin = ir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ir_GPIO_Port, &GPIO_InitStruct);

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
