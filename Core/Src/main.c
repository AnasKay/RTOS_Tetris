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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "semphr.h"
#include <stdbool.h>
#include <max7219.h>
#include <max7219_matrix.h>
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
SemaphoreHandle_t buttonA, buttonB, buttonC, displayUsartSem, initializeBoardSem, newBlockSem, checkRowsSem, lostSem, displaySPISem;
QueueHandle_t feld, position;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
void TaskMoveRight( void *pvParameters );
void TaskMoveLeft( void *pvParameters );
void TaskMoveDown( void *pvParameters );
void TaskPeriodical( void *pvParameters );
void TaskDisplayUsart( void *pvParameters );
void TaskInitializeBoard( void *pvParameters );
void TaskNewBlock( void *pvParameters );
void TaskCheckRow( void *pvParameters );
void TaskLost( void *pvParameters );
void TaskLowerBlock( void *pvParameters );
void TaskDisplaySPI( void *pvParameters );
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  buttonA = xSemaphoreCreateBinary();
  buttonB = xSemaphoreCreateBinary();
  buttonC = xSemaphoreCreateBinary();
  displayUsartSem = xSemaphoreCreateBinary();
  initializeBoardSem = xSemaphoreCreateBinary();
  newBlockSem = xSemaphoreCreateBinary();
  checkRowsSem = xSemaphoreCreateBinary();
  lostSem = xSemaphoreCreateBinary();
  displaySPISem = xSemaphoreCreateBinary();

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  feld = xQueueCreate(1, sizeof(int) *8 *8);
  position = xQueueCreate(1, sizeof(int) *2);


  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(TaskMoveRight, "TaskMoveRight", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  xTaskCreate(TaskMoveLeft, "TaskMoveLeft", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  xTaskCreate(TaskMoveDown, "TaskMoveDown", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  xTaskCreate(TaskPeriodical, "TaskPeriodical", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  xTaskCreate(TaskDisplayUsart, "TaskDisplayUsart", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  xTaskCreate(TaskInitializeBoard, "TaskInitializeBoard", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  xTaskCreate(TaskNewBlock, "TaskNewBlock", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  xTaskCreate(TaskCheckRow, "TaskCheckRow", configMINIMAL_STACK_SIZE, NULL, 4, NULL );
  xTaskCreate(TaskLost, "TaskLost", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  //xTaskCreate(TaskLowerBlock, "TaskLowerBlock", configMINIMAL_STACK_SIZE, NULL, 8, NULL );
  xTaskCreate(TaskDisplaySPI, "TaskDisplaySPI", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  xSemaphoreGive(initializeBoardSem);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void TaskPeriodical( void * pvParameters )
{
	char x = '_';
    for(;;)
    {
			HAL_UART_Transmit(&huart2, &x, sizeof(x), HAL_MAX_DELAY);
			HAL_Delay(100);
    }
}
//_________________________________________________________
void TaskLowerBlock( void * pvParameters )
{
	int display[8][8];
	int positionXY[2];
	int xAchse;
	int yAchse;
	char x = '+';
    for(;;)
    {
    	xQueueReceive( position, &positionXY, ( TickType_t ) 10 );
    	xQueueReceive( feld, &display, ( TickType_t ) 10 );
    	xAchse = positionXY[0];
    	yAchse = positionXY[1];

    	if(display[yAchse+1][xAchse] == 0 && display[yAchse+1][xAchse+1] == 0){

    		display[yAchse+1][xAchse] = 1;
    		display[yAchse+1][xAchse+1] = 1;
    		display[yAchse-1][xAchse] = 0;
    		display[yAchse-1][xAchse+1] = 0;
    		positionXY[0] = xAchse;
    		positionXY[1] = yAchse+1;

    		xQueueSend( feld, ( void * ) &display,  1 );
    		xQueueSend( position, ( void * ) &positionXY,  1 );
    		xSemaphoreGive(displayUsartSem);
    	}
    	else{
    		xQueueSend( feld, ( void * ) &display,  1 );
    		xQueueSend( position, ( void * ) &positionXY,  1 );
    		xSemaphoreGive(checkRowsSem);
    	}

		vTaskDelay(1000);
    }
}
//_________________________________________________________
void TaskInitializeBoard( void * pvParameters )
{
	int display[8][8] = {
			{0,0,0,1,1,0,0,0},
			{0,0,0,1,1,0,0,0},
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0},
			{0,0,0,0,0,0,0,0}
	};
	int positionXY[] = {3,1};
    for(;;)
    {
		if(xSemaphoreTake(initializeBoardSem, (TickType_t)portMAX_DELAY)== pdTRUE)
		{
			xQueueSend( feld, ( void * ) &display,  1 );
			xQueueSend( position, ( void * ) &positionXY,  1 );
		}

    }
}
//_________________________________________________________
void TaskNewBlock( void * pvParameters )
{
	int display[8][8];
	int positionXY[] = {3,1};
    for(;;)
    {
		if(xSemaphoreTake(newBlockSem, (TickType_t)portMAX_DELAY)== pdTRUE)
		{
			xQueueReceive( position, &positionXY, ( TickType_t ) 10 );
			xQueueReceive( feld, &display, ( TickType_t ) 10 );

			//if enough space to put new block
			if(display[2][3]==0 && display[2][4]==0){
				display[0][3] =1;
				display[0][4] =1;
				display[1][3] =1;
				display[1][4] =1;

				xQueueSend( feld, ( void * ) &display,  1 );
				xQueueSend( position, ( void * ) &positionXY,  1 );
				xSemaphoreGive(displayUsartSem);
			}
			else{
				//if not create new initial board
				//xSemaphoreGive(lostSem);
				xSemaphoreGive(initializeBoardSem);
			}
		}

    }
}
//_________________________________________________________
void TaskLost( void * pvParameters )
{
	int display[8][8];
    for(;;)
    {
		if(xSemaphoreTake(lostSem, (TickType_t)portMAX_DELAY)== pdTRUE)
		{
			xQueueReceive( feld, &display, ( TickType_t ) 10 );


			for(int i = 0; i <8; i++){
				for(int j = 0; j <8; j++){
					display[i][j]=1;
				}

			}
			xQueueSend( feld, ( void * ) &display,  1 );
			vTaskDelay(500);
			xSemaphoreGive(displayUsartSem);
			vTaskDelay(500);
			xQueueReceive( feld, &display, ( TickType_t ) 10 );

			for(int i = 0; i <8; i++){
				for(int j = 0; j <8; j++){
					display[i][j]=0;
				}

			}
			xQueueSend( feld, ( void * ) &display,  1 );
			vTaskDelay(500);
			xSemaphoreGive(displayUsartSem);
			vTaskDelay(500);
			xQueueReceive( feld, &display, ( TickType_t ) 10 );

			for(int i = 0; i <8; i++){
				for(int j = 0; j <8; j++){
					display[i][j]=1;
				}

			}
			xQueueSend( feld, ( void * ) &display,  1 );
			vTaskDelay(500);
			xSemaphoreGive(displayUsartSem);
			vTaskDelay(500);
			xQueueReceive( feld, &display, ( TickType_t ) 10 );

			for(int i = 0; i <8; i++){
				for(int j = 0; j <8; j++){
					display[i][j]=0;
				}

			}
			xQueueSend( feld, ( void * ) &display,  1 );
			vTaskDelay(500);
			xSemaphoreGive(displayUsartSem);
			vTaskDelay(500);

			xSemaphoreGive(initializeBoardSem);

		}

    }
}
//_________________________________________________________
void TaskMoveRight( void * pvParameters )
{
	char x = 'r';
	int display[8][8];
	int positionXY[2];
	int xAchse;
	int yAchse;
    for(;;)
    {
		if(xSemaphoreTake(buttonA, (TickType_t)portMAX_DELAY)== pdTRUE)
		{
			HAL_UART_Transmit(&huart2, &x, sizeof(x), HAL_MAX_DELAY);
			xQueueReceive( position, &positionXY, ( TickType_t ) 10 );
			xQueueReceive( feld, &display, ( TickType_t ) 10 );
			xAchse = positionXY[0];
			yAchse = positionXY[1];

			// wenn nicht schon ganz rechts
			if(xAchse <6){
				HAL_UART_Transmit(&huart2, "y", sizeof(char), HAL_MAX_DELAY);
				// wenn feld neben block frei
				if(display[yAchse-1][xAchse+2]==0 && display[yAchse][xAchse+2]==0){
					HAL_UART_Transmit(&huart2, "g", sizeof(char), HAL_MAX_DELAY);
					//block an position löschen
					display[yAchse-1][xAchse]=0;
					display[yAchse][xAchse]=0;
					//block verschieben
					display[yAchse-1][xAchse+2]=1;
					display[yAchse][xAchse+2]=1;

					//update pos
					positionXY[0]=xAchse+1;
					positionXY[1]=yAchse;
				}
			}
			xQueueSend( feld, ( void * ) &display,  1 );
			xQueueSend( position, ( void * ) &positionXY,  1 );
			xSemaphoreGive(displayUsartSem);
		}

    }
}
//_________________________________________________________
void TaskMoveLeft( void * pvParameters )
{
	char x = 'l';
	int display[8][8];
	int positionXY[2];
	int xAchse;
	int yAchse;
    for(;;)
    {
		if(xSemaphoreTake(buttonB, (TickType_t)portMAX_DELAY)== pdTRUE)
		{
			HAL_UART_Transmit(&huart2, &x, sizeof(x), HAL_MAX_DELAY);
			xQueueReceive( position, &positionXY, ( TickType_t ) 10 );
			xQueueReceive( feld, &display, ( TickType_t ) 10 );
			xAchse = positionXY[0];
			yAchse = positionXY[1];

			// wenn nicht schon ganz links
			if(xAchse >0){
				//HAL_UART_Transmit(&huart2, "y", sizeof(char), HAL_MAX_DELAY);
				/*char c=xAchse+'0';
				char v=yAchse+'0';
				char u = display[yAchse-1][xAchse-1];
				char w = display[yAchse][xAchse-1];
				HAL_UART_Transmit(&huart2, &c, sizeof(char), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, &v, sizeof(char), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "+", sizeof(char), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, &u, sizeof(char), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, &w, sizeof(char), HAL_MAX_DELAY);*/
				// wenn feld neben block frei
				if(display[yAchse-1][xAchse-1]==0 && display[yAchse][xAchse-1]==0){
					//HAL_UART_Transmit(&huart2, "g", sizeof(char), HAL_MAX_DELAY);
					//block an position löschen
					display[yAchse-1][xAchse+1]=0;
					display[yAchse][xAchse+1]=0;
					//block verschieben
					display[yAchse-1][xAchse-1]=1;
					display[yAchse][xAchse-1]=1;

					//update position
					positionXY[0]=xAchse-1;
					positionXY[1]=yAchse;
				}
			}
			xQueueSend( feld, ( void * ) &display,  1 );
			xQueueSend( position, ( void * ) &positionXY,  1 );
			xSemaphoreGive(displayUsartSem);
		}

    }
}
//_________________________________________________________
void TaskMoveDown( void * pvParameters )
{
	char x = 'd';
	int display[8][8];
	int positionXY[2];
	int xAchse;
	int yAchse;
	int lowest;

    for(;;)
    {
		if(xSemaphoreTake(buttonC, (TickType_t)portMAX_DELAY)== pdTRUE)
		{
			xQueueReceive( position, &positionXY, ( TickType_t ) 10 );
			xQueueReceive( feld, &display, ( TickType_t ) 10 );
			xAchse = positionXY[0];
			yAchse = positionXY[1];

			//find lowest free space
			int counter =1;
			lowest = yAchse;
			for(int i = yAchse; i <= 7; i++){
				if(display[yAchse+counter][xAchse] == 0 && display[yAchse+counter][xAchse+1] == 0){
					lowest = i;
				}
				else if(display[yAchse+counter][xAchse] == 1 && display[yAchse+counter][xAchse+1] == 1){
					break;
				}
				counter++;
			}

			char y=yAchse+'0';
			char v=lowest+'0';
			//HAL_UART_Transmit(&huart2, &x, sizeof(x), HAL_MAX_DELAY);
			//HAL_UART_Transmit(&huart2, &v, sizeof(x), HAL_MAX_DELAY);
			//HAL_UART_Transmit(&huart2, "+", sizeof(x), HAL_MAX_DELAY);
			//HAL_UART_Transmit(&huart2,&y , sizeof(y), HAL_MAX_DELAY);

			lowest++;
			//set block to lowest possible point
			display[lowest][xAchse]=1;
			display[lowest][xAchse+1]=1;
			display[lowest-1][xAchse]=1;
			display[lowest-1][xAchse+1]=1;

			if(lowest-1 != yAchse){
				//remove old block
				display[yAchse][xAchse]=0;
				display[yAchse][xAchse+1]=0;
				display[yAchse-1][xAchse]=0;
				display[yAchse-1][xAchse+1]=0;
			}
			xQueueSend( feld, ( void * ) &display,  1 );
			positionXY[0]=xAchse;
			positionXY[1]=lowest;
			xQueueSend( position, ( void * ) &positionXY,  1 );

			//xSemaphoreGive(displayUsartSem);
			//vTaskDelay(400);
			xSemaphoreGive(checkRowsSem);
			//vTaskDelay(200);
			//xSemaphoreGive(newBlockSem);
		}

    }
}
//_________________________________________________________
void TaskCheckRow( void * pvParameters ){
	int display[8][8];
	int positionXY[2];
	int xAchse;
	int yAchse;
	bool allOnes = true;

    for(;;)
    {
    	if(xSemaphoreTake(checkRowsSem, (TickType_t)portMAX_DELAY)== pdTRUE)
    	{
			xQueueReceive( position, &positionXY, ( TickType_t ) 10 );
			xQueueReceive( feld, &display, ( TickType_t ) 10 );
			xAchse = positionXY[0];
			yAchse = positionXY[1];
			allOnes = true;
			for(int i =0; i<8; i++){
				if (display[yAchse][i] == 0){
					allOnes = false;
				}
			}
			if(allOnes){
				for(int i =0; i<8; i++){
					display[yAchse][i]=0;
				}
			}

			allOnes = true;
			for(int i =0; i<8; i++){
				if (display[yAchse-1][i] == 0){
					allOnes = false;
				}
			}
			if(allOnes){
				for(int i =0; i<8; i++){
					display[yAchse-1][i]=0;
				}
			}

			//test
			if(allOnes){
				for(int i = yAchse; i > 0; i-- ){
					for(int j =0; j<8;j++){
						display[i][j]=display[i-2][j];
					}
				}

				for(int i = 0; i<8;i++){
					display[0][i] =0;
					display[1][i] =0;
				}
			}
			//test ende

			xQueueSend( feld, ( void * ) &display,  1 );
			positionXY[0]=3;
			positionXY[1]=1;
			xQueueSend( position, ( void * ) &positionXY,  1 );
			xSemaphoreGive(displayUsartSem);
			vTaskDelay(400);
			xSemaphoreGive(newBlockSem);

    	}
    }
}
//_________________________________________________________
void TaskDisplayUsart( void * pvParameters ){
	int display[8][8];
    for(;;)
    {
		if(xSemaphoreTake(displayUsartSem, (TickType_t)portMAX_DELAY)== pdTRUE)
		{
			xQueueReceive( feld, &display, ( TickType_t ) 10 );

			HAL_UART_Transmit(&huart2, "\n", sizeof(char), HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart2, "\r", sizeof(char), HAL_MAX_DELAY);

			for(int i = 0; i <8; i++){
				for(int j = 0; j <8; j++){
					char c=display[i][j]+'0';
					HAL_UART_Transmit(&huart2, &c, sizeof(char), HAL_MAX_DELAY);
					HAL_UART_Transmit(&huart2, " ", sizeof(char), HAL_MAX_DELAY);
				}
				HAL_UART_Transmit(&huart2, "\n", sizeof(char), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "\r", sizeof(char), HAL_MAX_DELAY);
			}
			xQueueSend( feld, ( void * ) &display,  1 );
			//xSemaphoreGive(displaySPISem);
		}

    }
}
//_________________________________________________________
void TaskDisplaySPI( void * pvParameters ){
	int display[8][8];
    for(;;)
    {
		if(xSemaphoreTake(displaySPISem, (TickType_t)portMAX_DELAY)== pdTRUE)
		{
			xQueueReceive( feld, &display, ( TickType_t ) 10 );
			//MAX7219_paintPoints(0, display);
			HAL_UART_Transmit(&huart2, "l", sizeof(char), HAL_MAX_DELAY);
			MAX7219_MatrixSetRow64(0, CHR('A'));
			MAX7219_MatrixUpdate();
			xQueueSend( feld, ( void * ) &display,  1 );
		}

    }
}
//_________________________________________________________
void EXTI3_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(buttonA,  &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
}
//_________________________________________________________
void EXTI4_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(buttonB,  &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_4);
}
//_________________________________________________________
void EXTI9_5_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(buttonC,  &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_5);
}
//_________________________________________________________
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
