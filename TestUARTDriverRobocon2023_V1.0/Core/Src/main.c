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
#include<stdlib.h>
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
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

osThreadId TaskPosHandle;
osThreadId TaskSpeedHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Task_Pos(void const * argument);
void Task_Speed(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define ID 3

int BoardID;
int intial_Rotate = 95;
uint8_t Mode;
int Dir;
uint16_t Speed;
uint16_t Rotate = 90;
//PID_SPEED_VAR:
int count1,home;
int pre_vt;
int  count1,precount1;
double v1; // van toc tho
double v1Filt; // van toc sau khi duoc loc
double v1Prev; // van toc truoc do phuc vu bo loc
double v_target; // van toc nham toi
double e1; // lỗi khâu P
double ei1; // lỗi khâu I
double ed1;
double u1; // tổng
double kp1 = 0.4,ki1 = 5,kd1=0; // chỉ số khâu P,I
double up1,ui1,ud1,ui_p1;

int dir , pwm; // biến chieu và tốc độ
double pos;

int p_target;
double e2;
double ei2;
double ed2;
double pre_e2;
double u2;
double kp2 = 0.4, ki2 = 0.0005, kd2 = 0.3;
double up2,ui2,ud2,ui_p2;
int counter2;
int preMode;

int mode=3;
int delay; //control toc do dong co buoc
int currentAngle; // goc hien tai
int numstep; //so buoc can di
int pre_angle;
double anglePerStep=0.9;//che do /1;
int angle;//goc can huong toi


char UARTRX1_Buffer[17];
char DataMain[17];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		HAL_UART_Receive_IT(&huart1, (uint8_t*)UARTRX1_Buffer, 17);

		int ViTriData = -1;
		for(int i = 0; i <= 16; ++i){
			if(UARTRX1_Buffer[i] == 149){
				ViTriData = i;
			}
		}

		if(ViTriData != -1){
			int cnt = 0;
			while(cnt < 17){
				DataMain[cnt] = UARTRX1_Buffer[ViTriData];
				++ViTriData;
				if(ViTriData == 17){
					ViTriData = 0;
				}
				++cnt;
			}
		}

		if(BoardID == 1){
			Mode = (DataMain[1] >> 1) & 3;
			if((DataMain[1] & 1) == 0){
				Dir = -1;
			}
			else if((DataMain[1] & 1) == 1){
				Dir = 1;
			}

			Speed = DataMain[2] << 8 | DataMain[3];
			Rotate = DataMain[4] << 8 | DataMain[5];
		}
		else if(BoardID == 2){
			Mode = (DataMain[6] >> 1) & 3;
			if((DataMain[6] & 1) == 0){
				Dir = -1;
			}
			else if((DataMain[6] & 1) == 1){
				Dir = 1;
			}

			Speed = DataMain[7] << 8 | DataMain[8];
			Rotate = DataMain[9] << 8 | DataMain[10];
		}
		else if(BoardID == 3){
			Mode = (DataMain[11] >> 1) & 3;
			if((DataMain[11] & 1) == 0){
				Dir = -1;
			}
			else if((DataMain[11] & 1) == 1){
				Dir = 1;
			}

			Speed = DataMain[12] << 8 | DataMain[13];
			Rotate = DataMain[14] << 8 | DataMain[15];
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  BoardID = ID;
  while(HAL_UART_Receive_IT(&huart1, (uint8_t*)UARTRX1_Buffer, 17)!=HAL_OK){};

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
  /* definition and creation of TaskPos */
  osThreadDef(TaskPos, Task_Pos, osPriorityNormal, 0, 128);
  TaskPosHandle = osThreadCreate(osThread(TaskPos), NULL);

  /* definition and creation of TaskSpeed */
  osThreadDef(TaskSpeed, Task_Speed, osPriorityAboveNormal, 0, 128);
  TaskSpeedHandle = osThreadCreate(osThread(TaskSpeed), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


//	  	for(int i = 0; i<= sizeof(UARTRX1_Buffer); ++i){
////	  		ViTriData = -1;
//	  		if((UARTRX1_Buffer[i] == 123) && (UARTRX1_Buffer[i + 6] == BoardID)){
//	  			ViTriData = i;
//	  			memset(DataRecieve, 0, sizeof(DataRecieve));
//	  			int cnt = -1;
//	  			while(1){
//	  				++cnt;
//	  				DataRecieve[cnt] = UARTRX1_Buffer[ViTriData];
//	  				++ViTriData;
//	  				if(UARTRX1_Buffer[ViTriData] == 13){
//	  					break;
//	  				}
//	  			}
//	  			break;
//	  		}
//	  	}
//
//	  	uint16_t _size = sizeof(UARTRX1_Buffer);
//		huart1.RxXferCount = _size;
//		huart1.pRxBuffPtr = (uint8_t *)&UARTRX1_Buffer[0];
//		memset(UARTRX1_Buffer, 0, _size);



//		HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  htim2.Init.Prescaler = 72-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_Pin|STEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DIR_Pin STEP_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_DC1_Pin */
  GPIO_InitStruct.Pin = ENC_DC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_DC1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENC_DC2_Pin */
  GPIO_InitStruct.Pin = ENC_DC2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ENC_DC2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : HOME_Pin */
  GPIO_InitStruct.Pin = HOME_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(HOME_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Encoder DC-SPEED
	if (GPIO_Pin == ENC_DC2_Pin){
		if (HAL_GPIO_ReadPin(ENC_DC1_GPIO_Port,ENC_DC1_Pin) == 0) {count1++;}
		else {count1--;}
	}
	if(GPIO_Pin == HOME_Pin){
			home = 1;
		}
}
int vPos;
void driveStep(){
	int n =0;

	if(angle > 500){angle = currentAngle;}
		if( currentAngle != angle ){
//			vPos = 0;
			if( currentAngle < angle){
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
				n = angle - currentAngle;
				numstep = n/anglePerStep *3;
			}
			else if( currentAngle > angle){
				HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
				n = currentAngle - angle;
				if( angle == 0){
					n = currentAngle;
				}
				numstep = n/anglePerStep *3;
			}
			for(int x = 0; x < numstep; x++) {
				if (angle == intial_Rotate){
					if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1){
	//				  home = 1;
					  break;
				  }
				}
//				if ((x > 3/5*numstep)&&(x<4/5*numstep)){delay = 2;}
//				else if (x>4/5*numstep)  delay = 3;
//				else delay = 1;
				HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
				osDelay(1);
				HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
				osDelay(1);
			}

			currentAngle = angle;

//			if((angle == intial_Rotate)&&HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) != 1){
//				if(pre_angle < currentAngle){
//					HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
//				}else if(pre_angle > currentAngle){
//					HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
//				}
//				HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
//				osDelay(1);
//				HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
//				osDelay(1);
//			}
//			pre_angle = currentAngle;
		}

//
//		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1){
////				  home = 1;
//				  angle = intial_Rotate;
//				  currentAngle = intial_Rotate;
//			  }
////	vPos = 1;
}
int stp;
void findHome(){// Tim lai nha
		Rotate = intial_Rotate;
		angle = intial_Rotate;
		currentAngle = intial_Rotate;

		for (stp = 0;stp<300*3;stp++){
			HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
			HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
			osDelay(1);
			HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
			osDelay(1);
			if (home == 1)break;
		}
		for (stp = 0;stp<200*3;stp++){
					HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
					HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 1);
					osDelay(1);
					HAL_GPIO_WritePin(STEP_GPIO_Port, STEP_Pin, 0);
					osDelay(1);
					if (home == 1)break;
				}



}
double udf2,uf2_p;
void posControlPID(){

	e2 = p_target - counter2;

	up2 = kp2*e2;
	ud2 = kd2*(e2 - pre_e2)/0.001;
	ui2 = ui_p2 + ki2*e2*0.001;
	udf2 = (1-0.5)*uf2_p+0.5*ud2;

	pre_e2 = e2;
	ui_p2 = ui2;
	uf2_p = udf2;

	u2 = up2 + udf2+ui2;
	if (u2>0)dir=-1;
	else if(u2<0)dir = 1;
	else dir = 0;
	if(u2>400)u2 =400;
	else if (u2<-300)u2 =-300;
	pwm = abs(u2);
	if ((pwm<130)&&(e2!=0)){
		pwm = 130;

	}
}
double pre1;
double ui1max ,ui1min;
void calculatePIDSpeed(){

	e1 = v_target - v1; // tinh toan loi ty le

	up1 = kp1*e1;
	ui1 = ui_p1 + ki1*e1*0.001;
	if (ui1>20)ui1 = 20;
	else if(ui1<-20)ui1 = -20;
	u1 = up1  + ui1; //Tinh tong bo dieu khien
	pre1 = e1;
	ui_p1 = ui1;


	if(u1 < 0) dir = 1; //bien doi chiue vong quay
	else dir = -1;


	if(u1>1000)u1 =1000;
	else if (u1<-1000)u1 =-1000;
	pwm = abs(u1);//Bao hoa xung cap
}
//Ham dam bao cap duong nguon xung pwm
void ControlMotor(int ChannelA, int ChannelB){
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000-ChannelA);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000-ChannelB);
}
//Ham cap xung cho dong co
void driveSpeed(int dir , int pwmVal){
	if (dir == -1){
		ControlMotor(pwmVal,0);
	}
	else if (dir == 1){
		ControlMotor(0,pwmVal);
	}
	else{
		ControlMotor(0,0);
	}
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Task_Pos */
/**
  * @brief  Function implementing the TaskPos thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Task_Pos */
void Task_Pos(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if (home == 0){
			findHome();

		}
		else if(home == 1){
			angle = Rotate;
			driveStep();
			osDelay(1);
		}
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Task_Speed */
/**
* @brief Function implementing the TaskSpeed thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Speed */
void Task_Speed(void const * argument)
{
  /* USER CODE BEGIN Task_Speed */
  /* Infinite loop */
  for(;;)
  {
  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == 1){
		  home = 1;
//		  angle = intial_Rotate;
//		  currentAngle = intial_Rotate;
	  }
  	  if (Mode != preMode){
  		 count1 = 0;
  		 precount1 = 0;
  	  }
  	  if((Mode==3)&&(Rotate == intial_Rotate))
  		{home = 0;
  	  Mode = 4;}
  	  if(Mode == 0 ){
  			v_target  = Dir*Speed;

		calculatePIDSpeed();
		driveSpeed(-dir,pwm);
  	  }
  	  else if(Mode == 1){
			p_target  = Dir*Speed;

  		posControlPID();
  		driveSpeed(-dir,pwm);
  	  }
  	  else if(Mode == 2){
  		driveSpeed(-Dir,Speed);
  	  }
  	  if (v_target != pre_target){ei1 = 0;}
  	  pre_target = v_target;
  	  preMode = Mode;

    osDelay(1);
  }
  /* USER CODE END Task_Speed */
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
  	counter2 = count1;
	pos = count1 - precount1;
	v1 = ((pos/0.001)/100)*60; //�?ổi vận tốc qua rpm,(Vận tốc ảo)
	v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;//Bo loc van toc thong thap
	v1Prev = v1; //cập nhật biến V1
	precount1 = count1;

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
