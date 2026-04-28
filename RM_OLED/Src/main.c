/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "Button_AD.h"
#include "ui.h"
#include "bsp_can.h"
#include "GM6020.h"
#include "buzzer.h"
#include "HTMotor.h"

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

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void motor_start(UI_MotorId_t motor, int16_t target_speed)
{
  if (motor == UI_MOTOR_GM6020) GM6020_UIStart(motor, target_speed);
  else if (motor == UI_MOTOR_HT) HTMotor_UIStart(motor, target_speed);
}

static void motor_set_speed(UI_MotorId_t motor, int16_t target_speed)
{
  if (motor == UI_MOTOR_GM6020) GM6020_UISetSpeed(motor, target_speed);
  else if (motor == UI_MOTOR_HT) HTMotor_UISetSpeed(motor, target_speed);
}

static void motor_start_pos(UI_MotorId_t motor, int16_t target_pos_deg)
{
  if (motor == UI_MOTOR_GM6020) GM6020_UIStartPos(motor, target_pos_deg);
  else if (motor == UI_MOTOR_HT) HTMotor_UIStartPos(motor, target_pos_deg);
}

static void motor_set_pos(UI_MotorId_t motor, int16_t target_pos_deg)
{
  if (motor == UI_MOTOR_GM6020) GM6020_UISetPos(motor, target_pos_deg);
  else if (motor == UI_MOTOR_HT) HTMotor_UISetPos(motor, target_pos_deg);
}

static void motor_stop(UI_MotorId_t motor)
{
  if (motor == UI_MOTOR_GM6020) GM6020_UIStop(motor);
  else if (motor == UI_MOTOR_HT) HTMotor_UIStop(motor);
}

static void oled_set_contrast(uint8_t contrast)
{
  oled_write_byte(0x81, OLED_CMD);
  oled_write_byte(contrast, OLED_CMD);
}

static void boot_logo_fade_and_wait_key(void)
{
  uint16_t i = 0U;
  uint16_t level = 0U;
  const uint16_t fade_steps = 64U;

  oled_LOGO();

  for (i = 0U; i <= fade_steps; i++)
  {
    level = (uint16_t)((0xCFU * i * i) / (fade_steps * fade_steps));
    oled_set_contrast((uint8_t)level);
    HAL_Delay(30);
  }
  oled_set_contrast(0xCFU);
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_CAN1_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
	led_off();
  HAL_GPIO_WritePin(GPIOH, POWER1_CTRL_Pin|POWER2_CTRL_Pin|POWER3_CTRL_Pin|POWER4_CTRL_Pin, GPIO_PIN_SET);
  oled_init();
  ButtonAD_Init();
  can_user_init(&hcan1);
  GM6020_Init();
  HTMotor_Init();
  Buzzer_Init();

  boot_logo_fade_and_wait_key();

  UI_Callbacks_t ui_cb = { 0 };
  ui_cb.on_motor_start = motor_start;
  ui_cb.on_motor_set_speed = motor_set_speed;
  ui_cb.on_motor_start_pos = motor_start_pos;
  ui_cb.on_motor_set_pos = motor_set_pos;
  ui_cb.on_motor_stop = motor_stop;
  UI_Init(&ui_cb);
  UI_RequestRedraw();
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    static uint32_t led_last_tick = 0;
    uint32_t now = HAL_GetTick();
    GPIO_PinState key_sample = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
    ButtonAD_Key_t ui_key = ButtonAD_ReadKey();
    GM6020_Task(now, key_sample);
    HTMotor_Task(now);

    UI_Task(now, ui_key);

    if ((HAL_GetTick() - led_last_tick) >= 500U)
    {
      HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
      led_last_tick = HAL_GetTick();
    }

    HAL_Delay(1);

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
  RCC_OscInitStruct.PLL.PLLM = 6;
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
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
