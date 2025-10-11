/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for RANDOM_SYNC_2017MHZ_V2
  * @author         : WiWi Development Team
  * @version        : 1.0.0
  * @date           : January 2025
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 WiWi Development Team.
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
#include "ethernetif.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ptp.h"
#include "ad9361.h"
#include "gnss.h"
#include "sync_algorithm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SYSTEM_VERSION "RANDOM_SYNC_2017MHZ_V2 v1.0.0"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
static osThreadId_t defaultTaskHandle;

/* USER CODE BEGIN PV */
static ptp_context_t ptp_ctx;
static ad9361_context_t radio_ctx;
static gnss_context_t gnss_ctx;
static sync_context_t sync_ctx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void init_system(void);
static void init_ptp(void);
static void init_radio(void);
static void init_gnss(void);
static void init_sync(void);
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
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE END 1 */

  /* USER CODE BEGIN Init */
  printf("Starting %s...\r\n", SYSTEM_VERSION);

  /* Initialize system components */
  init_system();

  /* USER CODE END Init */

  /* USER CODE BEGIN 2 */
  /* Create the thread(s) */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

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

  /* PLL configuration for 480 MHz */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_BLUE_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_GREEN_Pin LED_BLUE_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_BLUE_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  printf("Default task started\r\n");

  /* Infinite loop */
  for(;;)
  {
    /* Toggle LED to show system is alive */
    HAL_GPIO_TogglePin(GPIOB, LED_GREEN_Pin);
    osDelay(1000);

    /* Check system status */
    check_system_status();
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Initialize system components
  * @retval None
  */
static void init_system(void)
{
  printf("Initializing system...\r\n");

  /* Initialize PTP */
  init_ptp();

  /* Initialize radio */
  init_radio();

  /* Initialize GNSS */
  init_gnss();

  /* Initialize synchronization */
  init_sync();

  printf("System initialization complete\r\n");
}

/**
  * @brief  Initialize PTP subsystem
  * @retval None
  */
static void init_ptp(void)
{
  printf("Initializing PTP...\r\n");

  /* Initialize PTP context */
  ptp_init(&ptp_ctx);

  /* Set clock source to GNSS */
  ptp_set_clock_source(&ptp_ctx, PTP_SOURCE_GNSS);

  /* Enable hardware timestamping */
  ptp_enable_hardware_timestamping(&ptp_ctx);

  /* Configure PTP profile for G.8275.1 */
  ptp_config_t ptp_config = {
    .profile = PTP_PROFILE_G8275_1,
    .domain = 0,
    .priority1 = 128,
    .priority2 = 128,
    .clock_class = PTP_CLASS_LOCKED,
    .clock_accuracy = PTP_ACCURACY_100NS
  };

  ptp_configure(&ptp_ctx, &ptp_config);

  printf("PTP initialization complete\r\n");
}

/**
  * @brief  Initialize radio subsystem
  * @retval None
  */
static void init_radio(void)
{
  printf("Initializing radio...\r\n");

  /* Initialize AD9361 */
  ad9361_init(&radio_ctx);

  /* Configure for 2017 MHz operation */
  ad9361_config_t radio_config = {
    .tx_frequency = 2017000000ULL,  // 2017 MHz
    .rx_frequency = 2017000000ULL,  // 2017 MHz
    .tx_bandwidth = 20000000ULL,     // 20 MHz
    .rx_bandwidth = 20000000ULL,     // 20 MHz
    .tx_gain = 20,                   // 20 dB
    .rx_gain = 40,                   // 40 dB
    .sample_rate = 61440000ULL       // 61.44 MSPS
  };

  ad9361_configure(&radio_ctx, &radio_config);

  printf("Radio initialization complete\r\n");
}

/**
  * @brief  Initialize GNSS subsystem
  * @retval None
  */
static void init_gnss(void)
{
  printf("Initializing GNSS...\r\n");

  /* Initialize GNSS receiver */
  gnss_init(&gnss_ctx);

  /* Configure for multi-constellation */
  gnss_config_t gnss_config = {
    .enable_gps = true,
    .enable_glonass = true,
    .enable_galileo = true,
    .enable_beidou = true,
    .update_rate = 1  // 1 Hz
  };

  gnss_configure(&gnss_ctx, &gnss_config);

  printf("GNSS initialization complete\r\n");
}

/**
  * @brief  Initialize synchronization subsystem
  * @retval None
  */
static void init_sync(void)
{
  printf("Initializing synchronization...\r\n");

  /* Initialize sync algorithms */
  sync_init(&sync_ctx);

  /* Configure sync parameters */
  sync_config_t sync_config = {
    .target_accuracy = 1000,  // 1000 ns = 1 us
    .update_interval = 1000,  // 1 second
    .holdover_timeout = 3600  // 1 hour
  };

  sync_configure(&sync_ctx, &sync_config);

  printf("Synchronization initialization complete\r\n");
}

/**
  * @brief  Check system status
  * @retval None
  */
static void check_system_status(void)
{
  static uint32_t status_counter = 0;
  status_counter++;

  /* Check PTP status */
  ptp_status_t ptp_status;
  ptp_get_status(&ptp_ctx, &ptp_status);

  if (ptp_status.state == PTP_STATE_SLAVE) {
    HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);
  } else if (ptp_status.state == PTP_STATE_MASTER) {
    HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(GPIOB, LED_RED_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, LED_BLUE_Pin, GPIO_PIN_RESET);
  }

  /* Log status every 60 seconds */
  if (status_counter % 60 == 0) {
    printf("System status - PTP: %d, Sync accuracy: %lld ns\r\n",
           ptp_status.state, sync_get_current_accuracy(&sync_ctx));
  }
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
    HAL_GPIO_TogglePin(GPIOB, LED_RED_Pin);
    HAL_Delay(100);
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
