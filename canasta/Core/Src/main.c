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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEG_PORT GPIOB
#define DIG_PORT GPIOA
#define INFLARROJO_PORT GPIOB
#define LEDS_PORT GPIOA

#define SEG_A_PIN  GPIO_PIN_5
#define SEG_B_PIN  GPIO_PIN_7
#define SEG_C_PIN  GPIO_PIN_2
#define SEG_D_PIN  GPIO_PIN_1
#define SEG_E_PIN  GPIO_PIN_0
#define SEG_F_PIN  GPIO_PIN_6
#define SEG_G_PIN  GPIO_PIN_3

#define DIG_1_PIN  GPIO_PIN_9
#define DIG_2_PIN  GPIO_PIN_10
#define DIG_3_PIN  GPIO_PIN_11
#define DIG_4_PIN  GPIO_PIN_8

#define LED_ROJO  GPIO_PIN_4
#define LED_AMARILLO GPIO_PIN_5

#define TRIGGER_PIN GPIO_PIN_13
#define ECHO_PIN GPIO_PIN_14

#define EV_VIB   0x00000001U  // Bit 0
#define EV_PROX  0x00000002U  // Bit 1
#define EV_CANASTA 0x00000004U
#define EV_ACTUALIZA 0x00000008U

#define EV_PULSADO 0x00000001U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTaskADC */
osThreadId_t myTaskADCHandle;
const osThreadAttr_t myTaskADC_attributes = {
  .name = "myTaskADC",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask04 */
osThreadId_t myTask04Handle;
const osThreadAttr_t myTask04_attributes = {
  .name = "myTask04",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myTask06 */
osThreadId_t myTask06Handle;
const osThreadAttr_t myTask06_attributes = {
  .name = "myTask06",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for queueLeer */
osMessageQueueId_t queueLeerHandle;
const osMessageQueueAttr_t queueLeer_attributes = {
  .name = "queueLeer"
};
/* Definitions for myMutex01 */
osMutexId_t myMutex01Handle;
const osMutexAttr_t myMutex01_attributes = {
  .name = "myMutex01"
};
/* Definitions for Triple */
osEventFlagsId_t TripleHandle;
const osEventFlagsAttr_t Triple_attributes = {
  .name = "Triple"
};
/* Definitions for Boton */
osEventFlagsId_t BotonHandle;
const osEventFlagsAttr_t Boton_attributes = {
  .name = "Boton"
};
/* USER CODE BEGIN PV */
uint8_t rx_buff[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void StartTask05(void *argument);
void StartTask06(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SEG_AllOff(void);
void SEG_Show(int digito, uint8_t num);
void SEG_WriteSegments(uint8_t numero);
void Canasta_song(void);
void Boton_song(void);
void Tiempo_song(void);
void tone(uint32_t freq);
void noTone(void);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
int check_prox(void);
uint32_t IR_Trigger(void);
void delay_us(uint32_t us);
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
	// Habilitar contador de ciclos (DWT)
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	static uint8_t canastas = 0;
	SEG_AllOff();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of myMutex01 */
  myMutex01Handle = osMutexNew(&myMutex01_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (3, sizeof(uint8_t*), &myQueue01_attributes);

  /* creation of queueLeer */
  queueLeerHandle = osMessageQueueNew (1, sizeof(char*), &queueLeer_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTaskADC */
  myTaskADCHandle = osThreadNew(StartTask02, NULL, &myTaskADC_attributes);

  /* creation of myTask03 */
  myTask03Handle = osThreadNew(StartTask03, (void*) &canastas, &myTask03_attributes);

  /* creation of myTask04 */
  myTask04Handle = osThreadNew(StartTask04, NULL, &myTask04_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(StartTask05, NULL, &myTask05_attributes);

  /* creation of myTask06 */
  myTask06Handle = osThreadNew(StartTask06, (void*) &canastas, &myTask06_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of Triple */
  TripleHandle = osEventFlagsNew(&Triple_attributes);

  /* creation of Boton */
  BotonHandle = osEventFlagsNew(&Boton_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	HAL_UART_Receive_IT(&huart2, rx_buff, 1);
	osEventFlagsSet(BotonHandle, EV_PULSADO);
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

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
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA8 PA9
                           PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB13
                           PB3 PB5 PB6 PB7
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_13
                          |GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void SEG_AllOff(void) {
	HAL_GPIO_WritePin((GPIO_TypeDef*) SEG_PORT, SEG_A_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) SEG_PORT, SEG_B_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) SEG_PORT, SEG_C_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) SEG_PORT, SEG_D_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) SEG_PORT, SEG_E_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) SEG_PORT, SEG_F_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) SEG_PORT, SEG_G_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) DIG_PORT, DIG_1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) DIG_PORT, DIG_2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) DIG_PORT, DIG_3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin((GPIO_TypeDef*) DIG_PORT, DIG_4_PIN, GPIO_PIN_SET);
}

void SEG_Show(int digito, uint8_t num) {

	SEG_AllOff();

	switch (digito) {
	case 0: // Primer dígito (más a la izquierda)
		HAL_GPIO_WritePin((GPIO_TypeDef*) DIG_PORT, DIG_1_PIN, GPIO_PIN_RESET);
		break;
	case 1: // Segundo dígito
		HAL_GPIO_WritePin((GPIO_TypeDef*) DIG_PORT, DIG_2_PIN, GPIO_PIN_RESET);
		break;
	case 2: // Tercer dígito
		HAL_GPIO_WritePin((GPIO_TypeDef*) DIG_PORT, DIG_3_PIN, GPIO_PIN_RESET);
		break;
	case 3: // Cuarto dígito (más a la derecha)
		HAL_GPIO_WritePin((GPIO_TypeDef*) DIG_PORT, DIG_4_PIN, GPIO_PIN_RESET);
		break;
	default:
		break;
	}

	SEG_WriteSegments(num);

}

void SEG_WriteSegments(uint8_t numero) {

	numero = numero > 9 ? 0 : numero;

	switch (numero) {
	case 0:
		HAL_GPIO_WritePin(SEG_PORT, SEG_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_B_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_D_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_E_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_F_PIN, GPIO_PIN_SET);
		break;

	case 1:
		HAL_GPIO_WritePin(SEG_PORT, SEG_B_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		break;

	case 2:
		HAL_GPIO_WritePin(SEG_PORT, SEG_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_B_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_G_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_E_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_D_PIN, GPIO_PIN_SET);
		break;

	case 3:
		HAL_GPIO_WritePin(SEG_PORT, SEG_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_B_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_G_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_D_PIN, GPIO_PIN_SET);
		break;

	case 4:
		HAL_GPIO_WritePin(SEG_PORT, SEG_F_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_G_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_B_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		break;

	case 5:
		HAL_GPIO_WritePin(SEG_PORT, SEG_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_F_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_G_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_D_PIN, GPIO_PIN_SET);
		break;

	case 6:
		HAL_GPIO_WritePin(SEG_PORT, SEG_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_F_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_E_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_D_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_G_PIN, GPIO_PIN_SET);
		break;

	case 7:
		HAL_GPIO_WritePin(SEG_PORT, SEG_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_B_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		break;

	case 8:
		HAL_GPIO_WritePin(SEG_PORT, SEG_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_B_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_D_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_E_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_F_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_G_PIN, GPIO_PIN_SET);
		break;

	case 9:
		HAL_GPIO_WritePin(SEG_PORT, SEG_A_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_B_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_C_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_D_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_F_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SEG_PORT, SEG_G_PIN, GPIO_PIN_SET);
		break;

	default:
		break;
	}

}

// Canción de acierto de canasta
void Canasta_song(void) {
	tone(392);   // sol grave
	osDelay(200);

	tone(523);   // Do
	osDelay(200);

	tone(659);   // Mi
	osDelay(200);

	tone(784);   // Sol
	osDelay(350);

	tone(659);   // Mi
	osDelay(200);

	tone(784);   // Sol
	osDelay(550);

	noTone();
}

// Canción de reseteo a través del botón externo
void Boton_song(void) {
	tone(523);  //do
	osDelay(300);

	tone(784);  //sol
	osDelay(300);

	tone(659);  //mi
	osDelay(300);

	noTone();
}

//Para hacer cantar al buzzer variamos la frecuencia
void tone(uint32_t freq) {
	uint32_t arr = (1000000 / freq) - 1;

	__HAL_TIM_SET_AUTORELOAD(&htim3, arr);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arr / 2);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}

void noTone(void) {
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

//Interrupción del sensor de vibración
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_13) // sensor en PA0
	{
		osEventFlagsSet(TripleHandle, EV_VIB);
	} else if (GPIO_Pin == GPIO_PIN_4)  //boton
	{
		osEventFlagsSet(BotonHandle, EV_PULSADO);
	}
}

void delay_us(uint32_t us) {
	uint32_t start = DWT->CYCCNT;
	uint32_t cycles = us * 84;  // 84MHz = 84 ciclos/µs

	while ((DWT->CYCCNT - start) < cycles)
		;
}

// Función para generar pulso de trigger
uint32_t IR_Trigger(void) {
//	char buf[64];
	uint32_t t_us = 0;
	HAL_GPIO_WritePin(GPIOB, TRIGGER_PIN, GPIO_PIN_RESET);
	delay_us(200);

	// 1. Generar pulso trigger
	HAL_GPIO_WritePin(GPIOB, TRIGGER_PIN, GPIO_PIN_SET);
	delay_us(80); // ~10us
	HAL_GPIO_WritePin(GPIOB, TRIGGER_PIN, GPIO_PIN_RESET);

	// 2. Esperar flanco de subida en Echo
	while (HAL_GPIO_ReadPin(GPIOB, ECHO_PIN) == GPIO_PIN_RESET)
		;

	// 3. Marcar tiempo de inicio
	uint32_t start = DWT->CYCCNT;

	// 4. Esperar flanco de bajada en Echo
	while (HAL_GPIO_ReadPin(GPIOB, ECHO_PIN) == GPIO_PIN_SET)
		;

	// 5. Marcar tiempo de fin
	uint32_t end = DWT->CYCCNT;

	// 6. Calcular duración en microsegundos
	t_us = (end - start) / (SystemCoreClock / 1000000);

	return t_us;

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) { // solo se activa con los TRANSMIT_IT o _DMA

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static char comando_buffer[5];  // Buffer LOCAL para acumular
	static uint8_t idx = 0;

	if (huart->Instance == USART2) {
		//Recibimos de 1 en 1 asi que hay que detectar cuando llega el enter
		char c = rx_buff[0];  // Carácter recibido

		// Si es \0 (tecla Enter = \r)
		if (c == '\n') {
			if (idx > 0) {  // Si hay algo acumulado
				comando_buffer[idx] = '\0';  // Terminar string

				// IMPORTANTE: Crear copia para la cola
				char copia[5];
				strcpy(copia, comando_buffer);
				osMessageQueuePut(queueLeerHandle, copia, 0, 0);
				idx = 0;  // Resetear
			}
		}

		else if (idx < 5) {  // Si no es ENTER y hay espacio
			comando_buffer[idx] = c;
			idx++;
		} else { // Si buffer lleno (más de 5 chars sin ENTER)
			idx = 0;  // Descartar
		}

		HAL_UART_Receive_IT(&huart2, rx_buff, 1);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Tarea encargada de asegurar que es canasta y activar los eventos asociados
 * Enciende el led y hace sonar la canción
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(LEDS_PORT, LED_AMARILLO, GPIO_PIN_RESET);
		osEventFlagsWait(TripleHandle, EV_VIB, osFlagsWaitAny, osWaitForever);
		// Tras vibración, espera pelota (ventana medio segundo)
		uint32_t res = osEventFlagsWait(TripleHandle, EV_PROX, osFlagsWaitAny, 50);
		if (res == EV_PROX) {
			HAL_GPIO_WritePin(LEDS_PORT, LED_AMARILLO, GPIO_PIN_SET);
			osEventFlagsSet(TripleHandle, EV_CANASTA);
			Canasta_song();
		}

		osDelay(200);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Sensor ultrasónico de distancia. Detecta si la pelota ha pasado por delante
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	static uint32_t last_prox_time = 0;
	static uint8_t pelota_cerca = 0;

	/* Infinite loop */
	for (;;) {

		uint32_t current_time = osKernelGetTickCount();
		uint32_t us = IR_Trigger();

//	  pelota cerca
		if ((us > 150) && (us < 900)) {
			if (!pelota_cerca) {
				osEventFlagsSet(TripleHandle, EV_PROX);
				pelota_cerca = 1;
			}
			last_prox_time = current_time;
		}
		else { //Si pasa tiempo sin detectar pelota, limpiamos flag
			if (pelota_cerca && (current_time - last_prox_time) > 100) {
				osEventFlagsClear(TripleHandle, EV_PROX);
				pelota_cerca = 0;
			}
		}

		osDelay(100);

	}
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
 * @brief Encargado del valor del contador de canastas
 * @param argument: canastas
 * @retval None
 */
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	uint8_t *canastas = (uint8_t*) argument;
	static int actualizar = 0;
	static uint8_t tope = (uint8_t) 9999;
	static char prueba[5];
	char *buffer;
	/* Infinite loop */
	for (;;) {

		uint32_t res = osEventFlagsWait(TripleHandle, EV_CANASTA, osFlagsWaitAny, 10);
		if (res == EV_CANASTA) {
			osMutexAcquire(myMutex01Handle, osWaitForever);
			*canastas += 1;
			*canastas = *canastas >= tope ? 0 : *canastas;
			osMutexRelease(myMutex01Handle);
			actualizar = 1;
		}

		res = osEventFlagsWait(BotonHandle, EV_PULSADO, osFlagsWaitAny, 10);
		if (res == EV_PULSADO) {
			osMutexAcquire(myMutex01Handle, osWaitForever);
			*canastas = 0;
			osMutexRelease(myMutex01Handle);
			actualizar = 1;
		}

		if (actualizar) {
			if (osMessageQueuePut(myQueue01Handle, canastas, 0, osWaitForever) == osOK) {
				osEventFlagsSet(TripleHandle, EV_ACTUALIZA);
			}
			actualizar = 0;
		}

		if (osMessageQueueGetCount(queueLeerHandle) > 0) {

			if (osMessageQueueGet(queueLeerHandle, prueba, NULL, 0) == osOK) {
				buffer = (char*) prueba;

				if (buffer[0] == 'R' || buffer[0] == 'r') {
					osEventFlagsSet(BotonHandle, EV_PULSADO);
				}

				else {
					tope = strtol(prueba, NULL, 10);
					if (tope != 0){
						osMutexAcquire(myMutex01Handle, osWaitForever);
						*canastas = *canastas >= tope ? 0 : (*canastas);
						osMutexRelease(myMutex01Handle);
						if (*canastas == 0)
							osEventFlagsSet(BotonHandle, EV_PULSADO);
					}
				}
			}
		}

	}
	osDelay(1);

  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
 * @brief Consulta si se pulsa el botón externo
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
	GPIO_PinState bt4_estado_anterior = GPIO_PIN_RESET;
	GPIO_PinState bt4_estado;
	/* Infinite loop */
	for (;;) {
		bt4_estado = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		if (bt4_estado == GPIO_PIN_SET && bt4_estado_anterior == GPIO_PIN_RESET) {
			osEventFlagsSet(BotonHandle, EV_PULSADO);
			Boton_song();
		}
		bt4_estado_anterior = bt4_estado;

		osDelay(10);
	}
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_StartTask05 */
/**
 * @brief Encargada de escribir por la UART
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask05 */
void StartTask05(void *argument)
{
  /* USER CODE BEGIN StartTask05 */
	uint8_t valor;
	char buffer[4];
	/* Infinite loop */
	for (;;) {
		osEventFlagsWait(TripleHandle, EV_ACTUALIZA, osFlagsWaitAny, osWaitForever);
			if (osMessageQueueGetCount(myQueue01Handle) > 0) {
				if (osMessageQueueGet(myQueue01Handle, &valor, NULL, osWaitForever)	== osOK) {
					sprintf(buffer, "%d\r\n", valor);
					HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer), osWaitForever);
				}
			}

		osDelay(1);
	}
  /* USER CODE END StartTask05 */
}

/* USER CODE BEGIN Header_StartTask06 */
/**
 * @brief Encargada del display de 7 segmentos
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask06 */
void StartTask06(void *argument)
{
  /* USER CODE BEGIN StartTask06 */
	static int nums[4] = { 0 };
	uint8_t *canastas = (uint8_t*) argument;
	uint8_t copia_canastas;
	/* Infinite loop */
	for (;;) {
		if (osMutexAcquire(myMutex01Handle, 0) == osOK) {
			copia_canastas = *canastas;
			osMutexRelease(myMutex01Handle);
		}

		nums[0] = copia_canastas / 1000;
		nums[1] = (copia_canastas / 100) % 10;
		nums[2] = (copia_canastas / 10) % 10;
		nums[3] = copia_canastas % 10;

		for (int i = 0; i < 4; i++) {
			SEG_Show(i, nums[i]);
			osDelay(2);
		}

	}
  /* USER CODE END StartTask06 */
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
