/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include "Emm_V5.h"

#define USB_UART8_BUF_SIZE             512U
#define USB_TX_RETRY_MS                2U
#define EMM_RX_BUF_SIZE                32U
#define EMM_ADDR                       1U
#define EMM_QUERY_PERIOD_MS            80U
#define EMM_RESPONSE_GAP_MS            5U
#define EMM_RESPONSE_TIMEOUT_MS        50U
#define EMM_BOOT_DELAY_MS              500U
#define EMM_POS_CMD_VEL_RPM            1000U
#define EMM_POS_CMD_ACC                0U
#define EMM_HEIGHT_MAX_MM              400
#define EMM_RUN_HEIGHT_MAX_MM          300
#define EMM_HEIGHT_MIN_MM              0
#define EMM_PULSE_PER_MM_CMD           800UL /* 3200 / 4 */
#define EMM_PULSE_PER_REV              65536.0f

extern USBD_HandleTypeDef hUsbDeviceFS;

typedef enum
{
  EMM_QUERY_TPOS = 0,
  EMM_QUERY_VEL,
  EMM_QUERY_CPOS
} EmmQuery_t;

static uint8_t usb_uart8_rx_byte = 0U;
static uint8_t usb_uart8_to_usb_buf[USB_UART8_BUF_SIZE];
static volatile uint16_t usb_uart8_to_usb_head = 0U;
static volatile uint16_t usb_uart8_to_usb_tail = 0U;
static uint32_t usb_uart8_last_usb_tx_try_ms = 0U;

static uint8_t emm_usart6_rx_byte = 0U;
static uint8_t emm_rx_buf[EMM_RX_BUF_SIZE];
static volatile uint16_t emm_rx_len = 0U;
static uint32_t emm_last_rx_tick_ms = 0U;
static uint32_t emm_last_query_tick_ms = 0U;
static uint32_t emm_wait_start_ms = 0U;
static uint32_t emm_boot_tick_ms = 0U;
static uint8_t emm_ready = 0U;
static uint8_t emm_waiting_response = 0U;
static EmmQuery_t emm_query = EMM_QUERY_TPOS;
static float emm_target_pos_deg = 0.0f;
static float emm_current_pos_deg = 0.0f;
static float emm_speed_rpm = 0.0f;
static int32_t emm_height_setting_mm = 0;

static uint16_t USART_NextIndex(uint16_t index, uint16_t size)
{
  index++;
  if (index >= size)
  {
    index = 0U;
  }
  return index;
}

static uint8_t USB_UART8_PushByte(uint8_t data)
{
  uint16_t next = USART_NextIndex(usb_uart8_to_usb_head, USB_UART8_BUF_SIZE);
  if (next == usb_uart8_to_usb_tail)
  {
    return 0U;
  }

  usb_uart8_to_usb_buf[usb_uart8_to_usb_head] = data;
  usb_uart8_to_usb_head = next;
  return 1U;
}

static uint16_t USB_UART8_PeekLinearChunk(uint8_t **data)
{
  uint16_t head_snapshot = usb_uart8_to_usb_head;
  uint16_t tail_snapshot = usb_uart8_to_usb_tail;

  if (head_snapshot == tail_snapshot)
  {
    *data = 0;
    return 0U;
  }

  *data = &usb_uart8_to_usb_buf[tail_snapshot];
  if (head_snapshot > tail_snapshot)
  {
    return (uint16_t)(head_snapshot - tail_snapshot);
  }
  return (uint16_t)(USB_UART8_BUF_SIZE - tail_snapshot);
}

static void USB_UART8_AdvanceTail(uint16_t len)
{
  while (len > 0U)
  {
    usb_uart8_to_usb_tail = USART_NextIndex(usb_uart8_to_usb_tail, USB_UART8_BUF_SIZE);
    len--;
  }
}

static float Emm_PosRawToDeg(uint32_t pos_raw, uint8_t sign)
{
  float deg = ((float)pos_raw * 360.0f) / EMM_PULSE_PER_REV;
  if (sign != 0U)
  {
    deg = -deg;
  }
  return deg;
}

static void Emm_ProcessFrame(const uint8_t *buf, uint16_t len)
{
  uint32_t value = 0U;

  if ((buf == 0) || (len < 2U) || (buf[0] != EMM_ADDR))
  {
    return;
  }

  switch (buf[1])
  {
    case 0x33:
      if (len == 8U)
      {
        value = ((uint32_t)buf[3] << 24)
              | ((uint32_t)buf[4] << 16)
              | ((uint32_t)buf[5] << 8)
              | ((uint32_t)buf[6] << 0);
        emm_target_pos_deg = Emm_PosRawToDeg(value, buf[2]);
      }
      break;

    case 0x35:
      if (len == 6U)
      {
        value = ((uint16_t)buf[3] << 8) | ((uint16_t)buf[4] << 0);
        emm_speed_rpm = (float)value;
        if (buf[2] != 0U)
        {
          emm_speed_rpm = -emm_speed_rpm;
        }
      }
      break;

    case 0x36:
      if (len == 8U)
      {
        value = ((uint32_t)buf[3] << 24)
              | ((uint32_t)buf[4] << 16)
              | ((uint32_t)buf[5] << 8)
              | ((uint32_t)buf[6] << 0);
        emm_current_pos_deg = Emm_PosRawToDeg(value, buf[2]);
      }
      break;

    default:
      break;
  }
}

static void Emm_SendNextQuery(void)
{
  switch (emm_query)
  {
    case EMM_QUERY_TPOS:
      Emm_V5_Read_Sys_Params(EMM_ADDR, S_TPOS);
      emm_query = EMM_QUERY_VEL;
      break;

    case EMM_QUERY_VEL:
      Emm_V5_Read_Sys_Params(EMM_ADDR, S_VEL);
      emm_query = EMM_QUERY_CPOS;
      break;

    case EMM_QUERY_CPOS:
    default:
      Emm_V5_Read_Sys_Params(EMM_ADDR, S_CPOS);
      emm_query = EMM_QUERY_TPOS;
      break;
  }

  emm_waiting_response = 1U;
}

static uint8_t USB_IsConfigured(void)
{
  return (uint8_t)(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED);
}

/* USER CODE END 0 */

UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart6;
static uint8_t usart6_rx_byte = 0U;

/* UART7 init function */
void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}
/* UART8 init function */
void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}
/* USART6 init function */

void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
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

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspInit 0 */

  /* USER CODE END UART7_MspInit 0 */
    /* UART7 clock enable */
    __HAL_RCC_UART7_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART7;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN UART7_MspInit 1 */

  /* USER CODE END UART7_MspInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspInit 0 */

  /* USER CODE END UART8_MspInit 0 */
    /* UART8 clock enable */
    __HAL_RCC_UART8_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(UART8_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(UART8_IRQn);

  /* USER CODE BEGIN UART8_MspInit 1 */

  /* USER CODE END UART8_MspInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspInit 0 */

  /* USER CODE END USART6_MspInit 0 */
    /* USART6 clock enable */
    __HAL_RCC_USART6_CLK_ENABLE();

    __HAL_RCC_GPIOG_CLK_ENABLE();
    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_MspInit 1 */

  /* USER CODE END USART6_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART7)
  {
  /* USER CODE BEGIN UART7_MspDeInit 0 */

  /* USER CODE END UART7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART7_CLK_DISABLE();

    /**UART7 GPIO Configuration
    PE8     ------> UART7_TX
    PE7     ------> UART7_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_8|GPIO_PIN_7);

  /* USER CODE BEGIN UART7_MspDeInit 1 */

  /* USER CODE END UART7_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART8)
  {
  /* USER CODE BEGIN UART8_MspDeInit 0 */

  /* USER CODE END UART8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART8_CLK_DISABLE();

    /**UART8 GPIO Configuration
    PE1     ------> UART8_TX
    PE0     ------> UART8_RX
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_1|GPIO_PIN_0);

    HAL_NVIC_DisableIRQ(UART8_IRQn);

  /* USER CODE BEGIN UART8_MspDeInit 1 */

  /* USER CODE END UART8_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART6)
  {
  /* USER CODE BEGIN USART6_MspDeInit 0 */

  /* USER CODE END USART6_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART6_CLK_DISABLE();

    /**USART6 GPIO Configuration
    PG14     ------> USART6_TX
    PG9     ------> USART6_RX
    */
    HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14|GPIO_PIN_9);

    HAL_NVIC_DisableIRQ(USART6_IRQn);

  /* USER CODE BEGIN USART6_MspDeInit 1 */

  /* USER CODE END USART6_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void EmmMotor_Init(void)
{
  emm_rx_len = 0U;
  emm_last_rx_tick_ms = 0U;
  emm_last_query_tick_ms = 0U;
  emm_wait_start_ms = 0U;
  emm_boot_tick_ms = HAL_GetTick();
  emm_ready = 0U;
  emm_waiting_response = 0U;
  emm_query = EMM_QUERY_TPOS;
  emm_target_pos_deg = 0.0f;
  emm_current_pos_deg = 0.0f;
  emm_speed_rpm = 0.0f;
  emm_height_setting_mm = 0;
  USART6_StartReceiveIT();
}

void EmmMotor_Task(uint32_t now_ms)
{
  if ((emm_ready == 0U) && ((now_ms - emm_boot_tick_ms) >= EMM_BOOT_DELAY_MS))
  {
    emm_ready = 1U;
  }

  if ((emm_rx_len > 0U) && ((now_ms - emm_last_rx_tick_ms) >= EMM_RESPONSE_GAP_MS))
  {
    Emm_ProcessFrame(emm_rx_buf, emm_rx_len);
    emm_rx_len = 0U;
    emm_waiting_response = 0U;
  }

  if ((emm_waiting_response != 0U) && ((now_ms - emm_wait_start_ms) >= EMM_RESPONSE_TIMEOUT_MS))
  {
    emm_waiting_response = 0U;
    emm_rx_len = 0U;
  }

  if ((emm_ready != 0U)
   && (emm_waiting_response == 0U)
   && ((now_ms - emm_last_query_tick_ms) >= EMM_QUERY_PERIOD_MS))
  {
    Emm_SendNextQuery();
    emm_last_query_tick_ms = now_ms;
    emm_wait_start_ms = now_ms;
  }
}

void USART6_StartReceiveIT(void)
{
  HAL_UART_Receive_IT(&huart6, &emm_usart6_rx_byte, 1U);
}

void USART6_RxCpltCallback(void)
{
  if (emm_rx_len < EMM_RX_BUF_SIZE)
  {
    emm_rx_buf[emm_rx_len++] = emm_usart6_rx_byte;
  }
  else
  {
    emm_rx_len = 0U;
  }
  emm_last_rx_tick_ms = HAL_GetTick();
  USART6_StartReceiveIT();
}

void USART6_ErrorCallback(void)
{
  emm_rx_len = 0U;
  USART6_StartReceiveIT();
}

float EmmMotor_GetCurrentPosDeg(void)
{
  return emm_current_pos_deg;
}

float EmmMotor_GetTargetPosDeg(void)
{
  return emm_target_pos_deg;
}

float EmmMotor_GetSpeedRpm(void)
{
  return emm_speed_rpm;
}

static float Emm_HeightFromPosDeg(float pos_deg)
{
  float deg = pos_deg;
  if (deg < 0.0f)
  {
    deg = -deg;
  }
  return deg / 90.0f; /* 1mm = 90deg (4mm per rev) */
}

void EmmMotor_SetHeightSettingMm(int32_t height_mm)
{
  if (height_mm < EMM_HEIGHT_MIN_MM)
  {
    height_mm = EMM_HEIGHT_MIN_MM;
  }
  if (height_mm > EMM_HEIGHT_MAX_MM)
  {
    height_mm = EMM_HEIGHT_MAX_MM;
  }
  emm_height_setting_mm = height_mm;
  emm_target_pos_deg = (float)emm_height_setting_mm * 90.0f;
}

int32_t EmmMotor_GetHeightSettingMm(void)
{
  return emm_height_setting_mm;
}

float EmmMotor_GetCurrentHeightMm(void)
{
  return Emm_HeightFromPosDeg(emm_current_pos_deg);
}

float EmmMotor_GetTargetHeightMm(void)
{
  return emm_target_pos_deg / 90.0f;
}

void EmmMotor_StartMoveToSetting(void)
{
  int32_t run_height_mm = emm_height_setting_mm;
  float cur_height_mm = EmmMotor_GetCurrentHeightMm();
  uint8_t dir = 0U;
  uint32_t pulse = 0U;

  if (run_height_mm < EMM_HEIGHT_MIN_MM)
  {
    run_height_mm = EMM_HEIGHT_MIN_MM;
  }
  if (run_height_mm > EMM_RUN_HEIGHT_MAX_MM)
  {
    run_height_mm = EMM_RUN_HEIGHT_MAX_MM;
  }

  dir = (run_height_mm >= cur_height_mm) ? 1U : 0U; /* up=CCW, down=CW */
  pulse = (uint32_t)run_height_mm * EMM_PULSE_PER_MM_CMD;
  Emm_V5_Pos_Control(EMM_ADDR, dir, EMM_POS_CMD_VEL_RPM, EMM_POS_CMD_ACC, pulse, 1U, 0U);
  emm_target_pos_deg = (float)run_height_mm * 90.0f;
}

void EmmMotor_StopMove(void)
{
  Emm_V5_Stop_Now(EMM_ADDR, 0U);
}

void USB_UART8_BridgeInit(void)
{
  usb_uart8_to_usb_head = 0U;
  usb_uart8_to_usb_tail = 0U;
  usb_uart8_last_usb_tx_try_ms = 0U;
  UART8_StartReceiveIT();
}

void USB_UART8_OnUsbReceive(const uint8_t *data, uint16_t len)
{
  if ((data != 0) && (len > 0U))
  {
    HAL_UART_Transmit(&huart8, (uint8_t *)data, len, 50U);
  }
}

void USB_UART8_BridgeTask(uint32_t now_ms)
{
  uint8_t *data = 0;
  uint16_t len = 0U;

  if ((now_ms - usb_uart8_last_usb_tx_try_ms) < USB_TX_RETRY_MS)
  {
    return;
  }
  usb_uart8_last_usb_tx_try_ms = now_ms;

  if (!USB_IsConfigured())
  {
    return;
  }

  len = USB_UART8_PeekLinearChunk(&data);
  if ((len == 0U) || (data == 0))
  {
    return;
  }

  if (len > 64U)
  {
    len = 64U;
  }

  if (CDC_Transmit_FS(data, len) == USBD_OK)
  {
    USB_UART8_AdvanceTail(len);
  }
}

void UART8_StartReceiveIT(void)
{
  HAL_UART_Receive_IT(&huart8, &usb_uart8_rx_byte, 1U);
}

void UART8_RxCpltCallback(void)
{
  (void)USB_UART8_PushByte(usb_uart8_rx_byte);
  UART8_StartReceiveIT();
}

void UART8_ErrorCallback(void)
{
  UART8_StartReceiveIT();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    USART6_RxCpltCallback();
  }
  else if (huart->Instance == UART8)
  {
    UART8_RxCpltCallback();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    USART6_ErrorCallback();
  }
  else if (huart->Instance == UART8)
  {
    UART8_ErrorCallback();
  }
}

/* USER CODE END 1 */

