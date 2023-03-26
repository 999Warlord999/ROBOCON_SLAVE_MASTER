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
#define MOTORDC_CW 1
#define MOTORDC_CCW -1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
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
uint8_t stop = 0;
int v_t;

int i,i1,dir3,dir2;
int goc_target2;
int e1;
int e2;
double pre_e2;
double up2,ui2,ui_p2,ud2,udf2,uf2_p;
double u2;
double kp2 = 0.05, kd2 = 2;

int RotatePid,Dir2;
uint16_t pwm;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
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
				kp2 = 0.05;}
	}

	    up2 = kp2*e2;
		ud2 = kd2*(e2 - pre_e2)/0.002;
//		ui2 = ui_p2 + ki2*e2*0.001;
		udf2 = (1-alpha)*uf2_p+alpha*ud2;

//		if(ui2>8)ui2=8;
//		else if(ui2<-8)ui2=-8;


		pre_e2 = e2;
		uf2_p = udf2;
//		ui_p2 = ui2;

		if (u2>0)dir2=1;
		else if (u2<0)dir2 = -1;
		u2 = up2 + udf2;
		if (u2> 300)u2 =300;//180
		else if (u2<-300)u2=-300;//-180
		pwm = abs(u2);
		if((pwm < 10)&&(e2!=0)){//85
			pwm = 10;
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

int DaCapVong;


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
uint16_t a[10];

//#define YeuCauBanCotS1 24
//
//int ChoPhepBan, ModeBan;

uint8_t DataGun[3];
int ControlGun(uint8_t Mode){
	DataGun[0] = 149;
	DataGun[1] = Mode;
	DataGun[2] = 10;

	if(HAL_UART_Transmit(&huart4, (uint8_t *) DataGun, 3, 1000) != HAL_OK){
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
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
  osThreadDef(myTask04, TaskUART, osPriorityNormal, 0, 128);
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_9B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_ODD;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Pin|CompassReset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin CompassReset_Pin */
  GPIO_InitStruct.Pin = LED_Pin|CompassReset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CB_thanh_Pin UpperRingSS_Pin */
  GPIO_InitStruct.Pin = CB_thanh_Pin|UpperRingSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Dir_Pin */
  GPIO_InitStruct.Pin = Dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Dir_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : En_Pin */
  GPIO_InitStruct.Pin = En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(En_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : cb_vong_Pin */
  GPIO_InitStruct.Pin = cb_vong_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(cb_vong_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t DangThucThi2;
int pre_t = -88;
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if(GPIO_Pin == cb_vong_Pin){
//	  int cambien;
//  }
//}
void batthanh(void){
	v_t = 20;
	forward = 2;
	osDelay(3000);
	v_t =8;

	while(1){

		if(HAL_GPIO_ReadPin(CB_thanh_GPIO_Port, CB_thanh_Pin)==0){
			v_t = 0;
			osDelay(1000);

			forward = 0;
			RotatePid = 0;
			ControlDriver(0, 1, 0, 600, 0, 1, 0, 600, 0, 1, 0, 600);
			return;
		}
		osDelay(1);
	}
}

void batcot(void){
	while(1){
		if(DataTayGame[2] == 128 && !DangThucThi2){
				pre_t += 2;
				DangThucThi2 =1;
				}
				if(DataTayGame[2] == 32 && !DangThucThi2){
				pre_t -= 2;
				DangThucThi2 =1;
				}
				if(DataTayGame[2] == 0){
				DangThucThi2 = 0;
				}
				goc_target = pre_t;
			if(a[0]>700&&a[0]<3000){
				v_t = 0;
				osDelay(1000);
				forward = 0;
				RotatePid = 0;
				return;
			}
			osDelay(1);
}
}


void batvong(void){
	while(1){
		if(HAL_GPIO_ReadPin(cb_vong_GPIO_Port, cb_vong_Pin)==0){
			v_t = 0;
			osDelay(1000);
			forward = 0;
			RotatePid = 0;
			return;
		}
	}
}
int gun;
double e9,pre_e9;
double current_value9;
int POS_target = 0;
double deltaT9 = 0.002; // Th�?i gian lấy mẫu

double kp9 = 1,ki9 =0.0001, kd9 =0.5;

// Khai biến khâu tỉ lệ
double up9;

// khai biến khâu tích phân
double ui9,ui_p9;
int ui_above_limit9=5,ui_under_limit9=-5;

// khai biến khâu đạo hàm
double ud9,udf9,udf_p9;
double alpha9 = 0.4; // Hệ số bộ l�?c

// khai biến output
double u9;
int u_above_limit9 = 1000,u_under_limit9 = -1000;
int dir9,pwm9,pwm_movable9= 100;

uint8_t cambien;
int count1;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Encoder DC-SPEED
	if (GPIO_Pin == GPIO_PIN_6){
		if (HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_15) == 0) {count1++;}
		else {count1--;}
	}
}
void PID(){

	// khoảng cần đáp ứng của hệ thống
	e9 = POS_target - count1;

	// Khâu tỉ lệ
	up9 = kp9*e9;

	// khâu tích phân
	ui9 = ui_p9 + ki9*e9*deltaT9;
	// Bão hòa khâu tích phân
	if (ui9>ui_above_limit9)ui9=ui_above_limit9;
	else if(ui9<ui_under_limit9)ui9=ui_under_limit9;

	// khâu đạo hàm
	ud9 = kd9*(e9 - pre_e9)/deltaT9;
	// L�?c thông thấp khâu đạo hàm
	udf9 = (1-alpha)*udf_p9+alpha9*ud9;

	pre_e9 = e9;
	ui_p9 = ui9;
	udf_p9 = udf9;

	// Tổng out put
	u9 = up9 + udf9+ui9;
	// Bão hòa output
	if(u9>u_above_limit9)u9 = u_above_limit9;
	else if (u9<u_under_limit9)u9=u_under_limit9;

	// Xác định chi�?u:
	if (u9>0)dir9=-1;
	else if(u9<0)dir9 = 1;
	else dir9 = 0;

	// Xuất giá trị pwm
	pwm9 = abs(u9);
	if ((pwm9<pwm_movable9)&&(e9!=0)){
		pwm9 = pwm_movable9;

	}
}

void ControlMotor(int ChannelA, int ChannelB){
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, ChannelA);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, ChannelB);
}
//Ham cap xung cho dong co
void driveSpeed(int dir , int pwmVal){
	if (dir == MOTORDC_CCW){
		ControlMotor(pwmVal,0);
	}
	else if (dir == MOTORDC_CW){
		ControlMotor(0,pwmVal);
	}
	else{
		ControlMotor(0,0);
	}
}




int pwmVal, up, manual_rpm,s1,s2;
int currentAngle; // goc hien tai
int numstep; //so buoc can di
int pre_angle;
int angle;//goc can huong toi 810
int break_step;
int n, x;

void driveSpeed_tail(int dir , int pwmVal){
	if (dir == MOTORDC_CCW){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwmVal);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
	else if (dir == MOTORDC_CW){
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwmVal);
	}
	else{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
}


void driveStep(){
	int n =0;
	// 900
	while(currentAngle != angle){
		if( currentAngle < angle){
			HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 0);
			n = angle - currentAngle;
		}
		else if( currentAngle > angle){
			HAL_GPIO_WritePin(Dir_GPIO_Port, Dir_Pin, 1);
			n = currentAngle - angle;
		}
		for(int x = 0; x < n; x++) {
			HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 1);
			HAL_Delay(1);
			HAL_GPIO_WritePin(En_GPIO_Port, En_Pin, 0);
			HAL_Delay(1);
		}

		currentAngle = angle;
	}
}

uint8_t moveup;
uint8_t gunReady;
void upHandle(void){
	driveSpeed_tail(MOTORDC_CW, 200);
	while(1){
	  if(!HAL_GPIO_ReadPin(UpperRingSS_GPIO_Port, UpperRingSS_Pin)){
		  driveSpeed_tail(1, 0);
		  osDelay(100);
		  angle = 950;
		  moveup = 1;
		  break;
	  }
	}
}

void downHandle(){
	driveSpeed_tail(MOTORDC_CCW,200);
	while(1)
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 1){
			driveSpeed_tail(1,0);
			up = 0;
			break;
		}

}


void loadingProcess(void){
	upHandle();
	if(moveup){
		driveStep();
		moveup = 0;
	}
	angle  = 0;
	driveStep();
	gunReady = 1;
}
uint8_t step;
uint8_t trangthai;
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
//	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)a, 1);
//	  osDelay(5);
cambien = HAL_GPIO_ReadPin(cb_vong_GPIO_Port, cb_vong_Pin);


		if(DataTayGame[1]== 128&&DataTayGame[2]==0){
			loadingProcess();

			if(gunReady){
			ControlGun(1);
			gunReady = 0;
			}



		}
		if(DataTayGame[1]== 64&&DataTayGame[2]==0){
			up = 1;

				gun = 2;


		}
		if(DataTayGame[1]== 32&&DataTayGame[2]==0){
			up = 1;

				gun = 3;


		}



	  /*Nhấn nút F1*/
	  if(DataTayGame[7] == 128 && !DangThucThi){
		  step = 0;
		  if(trangthai == 0){
			  forward = 0;
			  RotatePid = 0;
			  ControlDriver(0, 1, 0, 300, 0, 1, 0, 300, 0, 1, 0, 300);
			  osDelay(1000);

			  // #2 chay thang trong 3.8s
			  goc_target = 5;
			  forward = 1;
			  v_t = 80;
			  osDelay(6000);
			  // #3 thang gap trong 550ms
			  v_t = 0;
			  osDelay(500);
			  forward = 0;
			  RotatePid = 0;
			  // #4 tha troi trong 3s
			  ControlDriver(0, 1, 0, (300-2*e1), 0, 1, 0, (300-2*e1), 0, 1, 0, (300+2*e1));
			  osDelay(1000);
			  ControlDriver(0, 1, 0, (300), 0, 1, 0, (300), 0, 1, 0, (300));
			  osDelay(2000);
			  // #5 quay 20 do trong 8s
			  ControlDriver(0, 1, 0, (200), 0, 1, 0, (400), 0, 1, 0, (600));
			  osDelay(2000);
			  goc_target2 = -30;

			  RotatePid = 1;
		  }
		  else if (trangthai == 1){
			  downHandle();
			  POS_target = -550;

		  }
		  else if (trangthai == 2){
			  forward = 0;
			  RotatePid = 0;
			  ControlDriver(0, 1, 0, 300, 0, 1, 0, 300, 0, 1, 0, 300);
			  osDelay(1000);
			  goc_target = -30;
			  forward = 1;
			  v_t = 50;
			  batvong();

		  }
		  else if (trangthai == 3){
			  POS_target = 0;
			  upHandle();
		  }
		  // #1 giu goc on dinh truoc khi chay

//		  osDelay(6000);
//		  RotatePid = 0;
		  // #6 lui den chong vong:
//		  goc_target = -20;
//		  ControlDriver(0, 1, 0, (300), 0, 1, 0, (300), 0, 1, 0, (300));
//		  osDelay(2000);
//
//		  forward = 1;
//		  v_t = 20;
//		  osDelay(2000);
//		  v_t = 0;
		  trangthai++;
		  DangThucThi = 1;
	  }

////
	  /*Nhấn nút F2*/
	  if(DataTayGame[7] == 64 && !DangThucThi){
		  step = 1;
//		  forward = 0;
//		  RotatePid = 0;
//		  ControlDriver(0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0);
//		  osDelay(2000);
//		  ControlDriver(3, 1, 0, 0, 3, 1, 0, 0, 3, 1, 0, 0);

		  //#1 quay goc on dinh banh xe trong 2s
		  forward = 0;
		  RotatePid = 0;
		  ControlDriver(0, 1, 0, 200, 0, -1, 0, 400, 0, 1, 0, 600);
		  osDelay(2000);
		  //#2 quay goc trong 8s
		  goc_target2 = -90;
		  RotatePid = 1;
		  osDelay(10000);
		  //#3 quay goc on donh banh xe trong 2s
		  RotatePid = 0;
		  ControlDriver(0, 1, 0, 100, 0, 1, 0, 100, 0, 1, 0, 100);
		  osDelay(2000);
		  //#4 chay theo goc -90
		  goc_target = -90;
		  batthanh();
		  goc_target2 = -90;
		  RotatePid = 1;
		  DangThucThi = 1;
	  }
////
////	  /*Nhấn nút F3*/
	if(DataTayGame[7] == 32 && !DangThucThi){
		step = 2;
		forward = 0;
		RotatePid = 0;
        goc_target = -90;
		ControlDriver(0, 1, 0, 600, 0, 1, 0, 600, 0, 1, 0, 600);
		osDelay(2000);
		goc_target = pre_t;
		v_t = 20;
		forward = 3 ;
		osDelay(1000);
		batcot();
//		while(a[0]>3000){
//			if(DataTayGame[2] == 128 && !DangThucThi){
//			goc_target ++;
//			DangThucThi =1;
//		}
//		if(DataTayGame[2] == 32 && !DangThucThi){
//		goc_target --;
//		DangThucThi =1;
//		}
//		if(DataTayGame[2] == 0){
//		DangThucThi = 0;
//		}
//
//		}
//		v_t = 0;
//
//		osDelay(1000);
//
//		forward = 0;
//		RotatePid = 0;
		ControlDriver(0, 1, 0, 600, 0, 1, 0, 600, 0, 1, 0, 600);

		DangThucThi = 1;
	}
////
//	  /*Nhấn nút F4*/
		if(DataTayGame[7] == 16 && !DangThucThi){
			forward = 0;
			RotatePid = 0;
			ControlDriver(0, 1, 0, 600, 0, 1, 0, 600, 0, 1, 0, 600);
			osDelay(1000);
			goc_target = pre_t;
			v_t = 10;
			forward = 3 ;
			osDelay(800);
			v_t = 0;


			DangThucThi = 1;
		}
//




	  /*Nhấn nút F5*/
	  if(DataTayGame[7] == 8 && !DangThucThi){
		  up = 2;
//		  forward = 0;
//		  RotatePid = 0;
//		  ControlDriver( 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0);
//		  osDelay(2000);
//		  ControlDriver(3, 1, 0, 0, 3, 1, 0, 0, 3, 1, 0, 0);
//		  forward = 0;
//		  RotatePid = 0;
//		  ControlDriver(0, 1, 0, 189, 0, 1, 0, 189, 0, 1, 0, 189);
//		  osDelay(2000);
//		  goc_target = -90;
//		  v_t = 30;
//		  forward = 3 ;
//		  osDelay(1000);
//		  while(a[0]>3000){
//			  if(DataTayGame[2] == 128 && !DangThucThi){
//				  goc_target ++;
//				  DangThucThi =1;
//			  }
//			  if(DataTayGame[2] == 32 && !DangThucThi){
//				  goc_target --;
//				  DangThucThi =1;
//			  }
//			  if(DataTayGame[2] == 0){
//				  DangThucThi = 0;
//			  }
//
//		  }
//		  v_t = 0;

//		osDelay(1000);
//
//		forward = 0;
//		RotatePid = 0;
//		ControlDriver(0, 1, 0, 189, 0, 1, 0, 189, 0, 1, 0, 189);
//		  osDelay(4000);
//		  v_t = 0;
//		  osDelay(1000);

	  	  DangThucThi = 1;
	  }
//
	  /*Nhấn nút F8*/
	  if(DataTayGame[7] == 4 && !DangThucThi){

		  forward = 0;
			RotatePid = 0;
			ControlDriver(0, 1, 0, 600, 0, 1, 0, 600, 0, 1, 0, 600);
			osDelay(1000);
			goc_target = pre_t;
			v_t = 10;
			forward = 4 ;
			osDelay(800);
			v_t = 0;





//		forward = 0;
//		RotatePid = 0;
//		ControlDriver(0, 1, 0, 300, 0, -1, 0, 300, 0, 1, 0, 300);
//		DangThucThi = 1;
//		  i1+=1;
//					  if((i1%2 == 0)&&(i1 != 0)){
//						  goc_target2 = 0;
//					  }
//					  else if(i1%2 != 0 ){
//						  goc_target2 = -20;
//					  }
////	  goc_target2  -= 90;
//		  forward = 0;
//		  ControlDriver(0, 1, 0, 57, 0, -1, 0, 123, 0, 1, 0, 189);
//		  osDelay(1000);
//		  RotatePid = 1;
//		  forward = 0;
//		  ControlDriver(0, 1, 0, 90, 0, 1, 0, 90, 0, 1, 0, 90);
//		  osDelay(1000);
//		  ControlDriver(0, -1, 500, 90, 0, -1, 500, 90, 0, -1, 500, 90);
//		  DangThucThi = 1;

//		  ControlDriver(0, -1, 500, 60, 0, 1, 500, 120, 0, -1, 500, 180);
//
		  DangThucThi = 1;
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

	  s1 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7);
	  	  s2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);

	  	  if (up == 1){
	  		  upHandle();
	  	  }


	  	  else if (up == 2){
	  		  downHandle();

	  	  }

	  	  else if(up == 0){
	  		  driveSpeed(0, 0);
	  		  driveStep();
	  	  }

	  	  if (up == 0&& currentAngle==0){
	  		 if (gun == 1){
				  ControlGun(1);
				  gun= 0;
			 }
	  		 else if (gun == 2){
				  ControlGun(2);
				  gun= 0;
			  } if (gun == 3){
				  ControlGun(3);
				  gun= 0;
			  }
	  	  }

//	  snprintf(DebugStr, "{GocRobot: %d, U: %d, E: %d, EI: %d}", GocRobot, u, e, ei);
//	  HAL_UART_Transmit(&huart3, (uint8_t *) DebugStr, 199, 1000);

//	  if(ChoPhepBan){
//		  ControlGun(ModeBan);
//		  ChoPhepBan = 0;
//	  }
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

	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)a, 1);
	  if (POS_target> 1000){
		  POS_target = 0;
	  }
//	  driveSpeed(dir9,pwm9);
//	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)a, 1);
//	  osDelay(5);
	  if(forward == 1){
	  		  ControlDriver(1, -1, v_t, (300-3*e1), 1, -1, v_t, (300-3*e1), 1, -1, v_t, (300+3*e1));
	  		  osDelay(1);
	  	  }
	  else{
		  osDelay(1);
	  }
	  if(forward == 4){
		  ControlDriver(1, 1, v_t, (600+6*e1), 1, 1, v_t, (600-6*e1), 1, 1, v_t, (600));
		 osDelay(1);
	  }
	  if(RotatePid == 1){
		  ControlDriver(2, -dir2, pwm, 200,2 , dir2, pwm, 400, 2, -dir2, pwm, 600);
	  }
	  if((forward == 2)){
		  ControlDriver(1, 1, v_t, (100+1*e1), 1, 1, v_t, (100+1*e1), 1, 1, v_t, (100-1*e1));
		  osDelay(1);
	  }
	  if((forward == 3)){

	  		  ControlDriver(1, -1, v_t, (600-6*e1), 1, -1, v_t, (600+6*e1), 1, -1, v_t, (600));
	  		  osDelay(1);
	  	  }

//	  if(DataTayGame[2] == 128 && !DangThucThi){
//		  goc_target--;
//	  	 	  	  DangThucThi = 1;
//	  	 	  }
//	  if(DataTayGame[2] == 32 && !DangThucThi){
//	  		  goc_target++;
//	  	  	 	  	  DangThucThi = 1;
//	  	  	 	  }


  }
  /* USER CODE END TaskXoayLaBan */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM14 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM14) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  goc_hientai = GocRobot;
  e1 = goc_target - goc_hientai;
  Pid_cal();
  PID();
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
