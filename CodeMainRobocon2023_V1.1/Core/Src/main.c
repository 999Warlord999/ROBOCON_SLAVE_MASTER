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
#include <stdlib.h>
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

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId myTask03Handle;
osThreadId myTask04Handle;
osThreadId Task_settingHandle;
/* USER CODE BEGIN PV */
int goc_target;
int goc_hientai;
int forward;

int goc_target2;
int e1;
int e2;
double pre_e2;
double up2,ui2,ui_p2,ud2,udf2,uf2_p;
double u2;
double kp2 = 0.075, kd2 = 2;

int RotatePid,Dir2;
uint16_t pwm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);



static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);
void TaskBlinkLed(void const * argument);
void TaskUART(void const * argument);
void TaskXoayLaBan(void const * argument);

/* USER CODE BEGIN PFP */

/*G�?i trong  HAL_TIM_PeriodElapsedCallback*/
double alpha;
void Pid_cal(){
	e2 = goc_target2 - goc_hientai;
	if(goc_target2>=0){
		if((e2<goc_target2*(1/3)))
			{alpha = 0.09;
			kp2 = 0.075;}
		else {
		alpha = 0.8;
		kp2 = 0.5;}
	}else{
		if((e2>goc_target2*(1/3)))
					{alpha = 0.09;
					kp2 = 0.075;}
				else {
				alpha = 0.8;
				kp2 = 0.5;}
	}

	    up2 = kp2*e2;
		ud2 = kd2*(e2 - pre_e2)/0.001;
//		ui2 = ui_p2 + ki2*e2*0.001;
		udf2 = (1-alpha)*uf2_p+alpha*ud2;

//		if(ui2>8)ui2=8;
//		else if(ui2<-8)ui2=-8;


		pre_e2 = e2;
		uf2_p = udf2;
//		ui_p2 = ui2;

		if (u2>0)Dir2=1;
		else if (u2<0)Dir2 = -1;
		u2 = up2 + udf2;
		if (u2> 400)u2 =400;//180
		else if (u2<-400)u2=-400;//-180
		pwm = abs(u2);
		if((pwm < 122)&&(e2!=0)){//85
			pwm = 122;
		}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define RFAddress 0x4A

typedef struct{
	int GamePadStatus;
	int CompassStatus;


//	int PushTime;
} stu_System;

stu_System System;

uint8_t UARTRX2_Buffer[10];
uint8_t UARTRX3_Buffer[9];
uint8_t DataTayGame[9];

int GocRobot;
int GocXoayTarget;
int GocXoayHienTai;

int ModeDangChay;

int DangThucThi;


char ds[12];
uint8_t uart1_ds, ds_ind, ds_cnt, ds_flg;

void GetDataCompass(){
	GocRobot = ds[1] - 48;
	int x = 2;
	while((ds[x] >= 48) && (ds[x] <= 57)){
		GocRobot = GocRobot * 10;
		GocRobot += ds[x] -48;
		++x;
	}

	if(ds[0] == '-'){
		GocRobot = -GocRobot;
	}
}

//char UARTRX1_Buffer[17];
//char DataMain[17];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart->Instance == USART1){
//
//	}

	if(huart->Instance == USART2){
//		HAL_UART_Receive_IT(&huart2, (uint8_t*)UARTRX2_Buffer, 10);

		if(uart1_ds != '\n')
				ds[ds_ind++] = uart1_ds;
		else{
				GetDataCompass();
				ds_cnt = ds_ind;
				ds_flg = 1;
				ds_ind = 0;
		}
		HAL_UART_Receive_IT(&huart2, &uart1_ds, 1);
	}

	if(huart->Instance == USART3){
		HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);
		int ViTriData = -1;
		for(int i = 0; i <= 8; ++i){
			if((UARTRX3_Buffer[i] == (RFAddress << 1 | 0)) || (UARTRX3_Buffer[i] == (RFAddress << 1 | 1))){
				ViTriData = i;
			}
		}
		if(ViTriData != -1){
			int cnt = 0;
			while(cnt < 9){
				DataTayGame[cnt] = UARTRX3_Buffer[ViTriData];
				++ViTriData;
				if(ViTriData == 9){
					ViTriData = 0;
				}
				++cnt;
			}
		}
	}
}


uint8_t UART1TxData[17];
/*Hàm truy�?n data xuống cho các board driver*/
int ControlDriver(uint8_t Mode1, int Dir1, uint16_t Speed1, uint16_t Rotate1, uint8_t Mode2, int Dir2, uint16_t Speed2, uint16_t Rotate2, uint8_t Mode3, int Dir3, uint16_t Speed3, uint16_t Rotate3){
	UART1TxData[0] = 149;

	if(Dir1 == -1){
		UART1TxData[1] = (Mode1 & 3) << 1;
	}
	else if(Dir1 == 1){
		UART1TxData[1] = (Mode1 & 3) << 1 | 1;
	}

	UART1TxData[2] = Speed1 >> 8;
	UART1TxData[3] = Speed1;

	UART1TxData[4] = Rotate1 >> 8;
	UART1TxData[5] = Rotate1;

	if(Dir2 == -1){
		UART1TxData[6] = (Mode2 & 3) << 1;
	}
	else if(Dir2 == 1){
		UART1TxData[6] = (Mode2 & 3) << 1 | 1;
	}

	UART1TxData[7] = Speed2 >> 8;
	UART1TxData[8] = Speed2;

	UART1TxData[9] = Rotate2 >> 8;
	UART1TxData[10] = Rotate2;

	if(Dir3 == -1){
		UART1TxData[11] = (Mode3 & 3) << 1;
	}
	else if(Dir3 == 1){
		UART1TxData[11] = (Mode3 & 3) << 1 | 1;
	}

	UART1TxData[12] = Speed3 >> 8;
	UART1TxData[13] = Speed3;

	UART1TxData[14] = Rotate3 >> 8;
	UART1TxData[15] = Rotate3;

	UART1TxData[16] = 10;

	if(HAL_UART_Transmit(&huart1, (uint8_t *) UART1TxData, 17, 1000) != HAL_OK){
		osDelay(1);
		return 0;
	}
	osDelay(1);
	return 1;
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
  HAL_Delay(5000);
  GocRobot = -999;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  HAL_UART_Receive_IT(&huart2, &uart1_ds, 1);
  //Uart3 Connect to HC-12
  HAL_UART_Receive_IT(&huart3, (uint8_t*)UARTRX3_Buffer, 9);

//  HAL_Delay(100);
//
//  while(1){
//	  if(HAL_GPIO_ReadPin(CompassReady_GPIO_Port, CompassReady_Pin)){
//		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//		  System.CompassStatus = 1;
//		  break;
//	  }
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
//	  HAL_Delay(200);
//	  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
//	  HAL_Delay(800);
//  }






  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, TaskBlinkLed, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* definition and creation of myTask04 */
  osThreadDef(myTask04, TaskUART, osPriorityIdle, 0, 128);
  myTask04Handle = osThreadCreate(osThread(myTask04), NULL);

  /* definition and creation of Task_setting */
  osThreadDef(Task_setting, TaskXoayLaBan, osPriorityAboveNormal, 0, 128);
  Task_settingHandle = osThreadCreate(osThread(Task_setting), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  ControlDriver(0, 0, 0, 0, 0);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.WordLength = UART_WORDLENGTH_9B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_EVEN;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|CompassReset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RelayKhanCap_GPIO_Port, RelayKhanCap_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin CompassReset_Pin */
  GPIO_InitStruct.Pin = LED_Pin|CompassReset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CompassReady_Pin */
  GPIO_InitStruct.Pin = CompassReady_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CompassReady_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RelayKhanCap_Pin */
  GPIO_InitStruct.Pin = RelayKhanCap_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RelayKhanCap_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
int dir3, i,i1;
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  /*Nhấn nút F1*/
	  if(DataTayGame[7] == 128 && !DangThucThi){

		  ControlDriver(0, 1, 0, 90, 0, 1, 0, 90, 0, 1, 0, 90);
		  osDelay(1000);
		  goc_target = 0;
		  i+=1;
		  forward = i;
		  RotatePid = 0;

		  osDelay(3200);
		  forward = 0;
		  RotatePid = 0;
		  ControlDriver(0, dir3, 100, 90-1*e1, 50, dir3, 100, 90-1*e1,0, dir3, 100, 90+1*e1);
		  osDelay(500);
		 ControlDriver(0, dir3, 1000, 90, 0, dir3, 1000, 90,0, dir3, 1000, 90);
		 osDelay(500);
		 ControlDriver(0, 1, 0, 90, 0, 1, 0, 90,0, 1, 0, 90);
		 osDelay(1000);
		 goc_target2 = 0;
		 RotatePid = 1;
		  DangThucThi = 1;
	  }

////
	  /*Nhấn nút F2*/
	  if(DataTayGame[7] == 64 && !DangThucThi){
		  forward = 0;
		  RotatePid = 0;
		  ControlDriver(3, 1, 0, 92, 3, 1, 0, 92, 3, 1, 0, 95);
		  osDelay(3000);
		  ControlDriver(0, 1, 0, 90, 0, 1, 0, 90, 0, 1, 0, 90);
	  	  DangThucThi = 1;
	  }
////
////	  /*Nhấn nút F3*/
		  if(DataTayGame[7] == 32 && !DangThucThi){
			  forward = 0;
			  RotatePid = 0;
			  ControlDriver(0, 1, 0, 60, 0, -1, 0, 120, 0, 1, 0, 180);
			  DangThucThi = 1;
		  }
////
//	  /*Nhấn nút F4*/
	  if(DataTayGame[7] == 16 && !DangThucThi){
		  forward = 0;
		  RotatePid = 0;
		  ControlDriver(0, 1, 0, 90, 0, 1, 0, 90, 0, 1, 0, 90);
	  	  DangThucThi = 1;
	  }
//




	  /*Nhấn nút F5*/
	  if(DataTayGame[7] == 8 && !DangThucThi){
		  	  	  forward = 0;
		  		  RotatePid = 0;
		  		 ControlDriver(0, dir3, 2000, 90, 0, dir3, 2000, 90,0, dir3, 2000, 90);
		  		 osDelay(400);
		  		 ControlDriver(0, 1, 0, 90, 0, 1, 0, 90,0, 1, 0, 90);
		  		 osDelay(1000);
		  		 goc_target2 = 0;
		  		 RotatePid = 1;
	  	  DangThucThi = 1;
	  }
//
	  /*Nhấn nút F8*/
	  if(DataTayGame[7] == 4 && !DangThucThi){
		  i1+=1;
		  if((i1%2 == 0)&&(i1 != 0)){
			  goc_target2 = 0;
		  }
		  else if(i1%2 != 0 ){
			  goc_target2 = -35;
		  }

		  forward = 0;
		  ControlDriver(0, 1, 0, 60, 0, -1, 0, 120, 0, 1, 0, 180);
		  osDelay(1000);
		  RotatePid = 1;
//		  forward = 0;
//		  ControlDriver(0, 1, 0, 90, 0, 1, 0, 90, 0, 1, 0, 90);
//		  osDelay(1000);
//		  ControlDriver(0, -1, 500, 90, 0, -1, 500, 90, 0, -1, 500, 90);
//		  DangThucThi = 1;

//		  ControlDriver(0, -1, 500, 60, 0, 1, 500, 120, 0, -1, 500, 180);
//
//		  DangThucThi = 1;
	  }

	  if(DataTayGame[7] == 0){
		  DangThucThi = 0;
	  }

	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_TaskBlinkLed */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskBlinkLed */
void TaskBlinkLed(void const * argument)
{
  /* USER CODE BEGIN TaskBlinkLed */


  /* Infinite loop */
  for(;;)
  {
//	  System.CompassStatus = HAL_GPIO_ReadPin(CompassReady_GPIO_Port, CompassReady_Pin);
	  if(GocRobot != -999){
		  System.CompassStatus = 1;
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  osDelay(50);
	  }
	  else {
		  System.CompassStatus = 0;
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  osDelay(500);
	  }

  }
  /* USER CODE END TaskBlinkLed */
}

/* USER CODE BEGIN Header_TaskUART */
/**
* @brief Function implementing the myTask04 thread.
* @param argument: Not used
* @retval None
*/

char DebugStr[200];
/* USER CODE END Header_TaskUART */
void TaskUART(void const * argument)
{
  /* USER CODE BEGIN TaskUART */
  /* Infinite loop */
  for(;;)
  {
//	  snprintf(DebugStr, "{GocRobot: %d, U: %d, E: %d, EI: %d}", GocRobot, u, e, ei);
//	  HAL_UART_Transmit(&huart3, (uint8_t *) DebugStr, 199, 1000);
	  osDelay(100);
  }
  /* USER CODE END TaskUART */
}

/* USER CODE BEGIN Header_TaskXoayLaBan */
/**
* @brief Function implementing the Task_setting thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskXoayLaBan */
void TaskXoayLaBan(void const * argument)
{
  /* USER CODE BEGIN TaskXoayLaBan */
  /* Infinite loop */
  for(;;)
  {
	  if(forward % 2 != 0){
	  		  ControlDriver(0, -1, 400, (90-1.5*e1), 0, -1, 400, (90-1.5*e1), 0, -1, 400, (90+1.5*e1));
	  		  dir3 = 1;
	  		  osDelay(1);
	  	  }
	  else{
		  osDelay(1);
	  }
	  if(RotatePid == 1){
		  ControlDriver(0, Dir2, pwm, 60, 2, -Dir2, pwm, 120, 2, Dir2, pwm, 180);
	  }
	  if((forward % 2 == 0)&&(forward != 0)){
		  ControlDriver(0, 1, 400, (90+1.5*e1), 0, 1, 400, (90+1.5*e1), 0, 1, 400, (90-1.5*e1));
		  dir3 = -1;
		  osDelay(1);
	  }
  }
  /* USER CODE END TaskXoayLaBan */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  goc_hientai = GocRobot;
  e1 = goc_target - goc_hientai;
  Pid_cal();
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
