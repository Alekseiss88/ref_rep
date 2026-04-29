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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BNO08X_SPI_TIMEOUT_MS            20U
#define BNO08X_SHTP_HEADER_SIZE          4U
#define BNO08X_MAX_PACKET_PAYLOAD        64U
#define BNO08X_MAX_CHANNELS              6U
#define BNO08X_MAX_REASONABLE_PACKET_LEN 512U

#define BNO08X_CHANNEL_EXECUTABLE        1U
#define BNO08X_CHANNEL_CONTROL           2U

#define BNO08X_REPORT_PRODUCT_ID_REQUEST 0xF9U
#define BNO08X_REPORT_PRODUCT_ID_RESP    0xF8U
#define BNO08X_PRODUCT_ID_ETALON         0x0098A6B4UL

#define BNO08X_RESET_PULSE_MS            2U
#define BNO08X_BOOT_TIMEOUT_MS           200U
#define BNO08X_WAKE_ASSERT_MS            1U
#define BNO08X_WAKE_TIMEOUT_MS           10U
#define BNO08X_RX_DRAIN_MAX_PACKETS      8U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
typedef struct
{
  volatile uint32_t tx_attempts;
  volatile uint32_t tx_ok;
  volatile uint32_t tx_errors;
  volatile uint32_t rx_packets;
  volatile uint32_t rx_errors;
  volatile uint32_t rx_empty_reads;
  volatile uint32_t rx_invalid_headers;
  volatile uint32_t wake_requests;
  volatile uint32_t wake_timeouts;
  volatile uint32_t hw_resets;
  volatile uint32_t product_id_read;
  volatile uint32_t product_id_etalon;
  volatile uint8_t boot_ok;
  volatile uint8_t product_id_valid;
  volatile uint8_t product_id_match;
  volatile uint16_t last_rx_len;
  volatile uint8_t last_channel;
  volatile uint8_t last_report_id;
  volatile uint8_t link_ok;
  volatile uint8_t last_payload[8];
} BNO08X_Diag_t;

static BNO08X_Diag_t g_bno08x_diag = {
    .product_id_etalon = BNO08X_PRODUCT_ID_ETALON,
};
static uint8_t g_bno08x_seq[BNO08X_MAX_CHANNELS] = {0};
static volatile uint8_t g_bno08x_int_pending = 0U;
static volatile uint8_t g_spi_tx_done = 0U;
static volatile uint8_t g_spi_rx_done = 0U;
static volatile uint8_t g_spi_error = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline void BNO08X_CS_Select(void);
static inline void BNO08X_CS_Deselect(void);
static HAL_StatusTypeDef BNO08X_SPI_WritePacket(uint8_t channel, const uint8_t *payload, uint16_t payload_len);
static HAL_StatusTypeDef BNO08X_SPI_ReadPacket(uint8_t *payload, uint16_t payload_size, uint16_t *payload_len, uint8_t *channel);
static HAL_StatusTypeDef BNO08X_SPI_Transmit_IT_Blocking(const uint8_t *data, uint16_t size, uint32_t timeout_ms);
static HAL_StatusTypeDef BNO08X_SPI_Receive_IT_Blocking(uint8_t *data, uint16_t size, uint32_t timeout_ms);
static HAL_StatusTypeDef BNO08X_SendSoftReset(void);
static HAL_StatusTypeDef BNO08X_SendProductIdRequest(void);
static void BNO08X_SetProtocolSPI(void);
static HAL_StatusTypeDef BNO08X_HardwareResetAndBoot(void);
static HAL_StatusTypeDef BNO08X_Wake(void);
static void BNO08X_UpdateDiagFromPayload(uint8_t channel, const uint8_t *payload, uint16_t payload_len);
static uint32_t BNO08X_ReadU32LE(const uint8_t *buf);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void BNO08X_CS_Select(void)
{
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
}

static inline void BNO08X_CS_Deselect(void)
{
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

static void BNO08X_SetProtocolSPI(void)
{
  HAL_GPIO_WritePin(BNO08X_PS1_GPIO_Port, BNO08X_PS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BNO08X_PS0_WAKE_GPIO_Port, BNO08X_PS0_WAKE_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef BNO08X_HardwareResetAndBoot(void)
{
  uint32_t start_ms;

  BNO08X_SetProtocolSPI();
  g_bno08x_int_pending = 0U;

  HAL_GPIO_WritePin(BNO08X_RST_GPIO_Port, BNO08X_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(BNO08X_RESET_PULSE_MS);
  HAL_GPIO_WritePin(BNO08X_RST_GPIO_Port, BNO08X_RST_Pin, GPIO_PIN_SET);
  g_bno08x_diag.hw_resets++;

  start_ms = HAL_GetTick();
  while (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - start_ms) > BNO08X_BOOT_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  /* Keep PS0/WAKE high through first H_INTN assertion to lock SPI mode. */
  g_bno08x_diag.boot_ok = 1U;
  g_bno08x_int_pending = 1U;

  return HAL_OK;
}

static HAL_StatusTypeDef BNO08X_Wake(void)
{
  uint32_t start_ms;

  g_bno08x_diag.wake_requests++;

  if (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == GPIO_PIN_RESET)
  {
    g_bno08x_int_pending = 1U;
    return HAL_OK;
  }

  HAL_GPIO_WritePin(BNO08X_PS0_WAKE_GPIO_Port, BNO08X_PS0_WAKE_Pin, GPIO_PIN_RESET);
  HAL_Delay(BNO08X_WAKE_ASSERT_MS);
  HAL_GPIO_WritePin(BNO08X_PS0_WAKE_GPIO_Port, BNO08X_PS0_WAKE_Pin, GPIO_PIN_SET);

  start_ms = HAL_GetTick();
  while (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - start_ms) > BNO08X_WAKE_TIMEOUT_MS)
    {
      g_bno08x_diag.wake_timeouts++;
      return HAL_TIMEOUT;
    }
  }

  g_bno08x_int_pending = 1U;
  return HAL_OK;
}

static HAL_StatusTypeDef BNO08X_SPI_WritePacket(uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
  HAL_StatusTypeDef status;
  uint8_t tx_buf[BNO08X_SHTP_HEADER_SIZE + BNO08X_MAX_PACKET_PAYLOAD];
  uint16_t packet_len = (uint16_t)(BNO08X_SHTP_HEADER_SIZE + payload_len);
  uint16_t i;

  if ((channel >= BNO08X_MAX_CHANNELS) || (payload_len > BNO08X_MAX_PACKET_PAYLOAD))
  {
    return HAL_ERROR;
  }

  tx_buf[0] = (uint8_t)(packet_len & 0xFFU);
  tx_buf[1] = (uint8_t)((packet_len >> 8) & 0x7FU);
  tx_buf[2] = channel;
  tx_buf[3] = g_bno08x_seq[channel]++;

  for (i = 0; i < payload_len; i++)
  {
    tx_buf[BNO08X_SHTP_HEADER_SIZE + i] = payload[i];
  }

  BNO08X_CS_Select();
  status = BNO08X_SPI_Transmit_IT_Blocking(tx_buf, packet_len, BNO08X_SPI_TIMEOUT_MS);
  BNO08X_CS_Deselect();

  return status;
}

static HAL_StatusTypeDef BNO08X_SPI_ReadPacket(uint8_t *payload, uint16_t payload_size, uint16_t *payload_len, uint8_t *channel)
{
  HAL_StatusTypeDef status;
  uint8_t header[BNO08X_SHTP_HEADER_SIZE];
  uint16_t packet_len;
  uint16_t data_len;
  uint16_t to_copy;

  if ((payload == NULL) || (payload_len == NULL) || (channel == NULL))
  {
    return HAL_ERROR;
  }

  BNO08X_CS_Select();
  status = BNO08X_SPI_Receive_IT_Blocking(header, BNO08X_SHTP_HEADER_SIZE, BNO08X_SPI_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    BNO08X_CS_Deselect();
    return status;
  }

  packet_len = (uint16_t)header[0] | ((uint16_t)header[1] << 8);

  if ((header[0] == 0xFFU) && (header[1] == 0xFFU))
  {
    *payload_len = 0U;
    BNO08X_CS_Deselect();
    g_bno08x_diag.rx_empty_reads++;
    return HAL_BUSY;
  }

  packet_len &= 0x7FFFU;
  *channel = header[2];

  if (packet_len < BNO08X_SHTP_HEADER_SIZE)
  {
    *payload_len = 0U;
    BNO08X_CS_Deselect();
    g_bno08x_diag.rx_invalid_headers++;
    return HAL_ERROR;
  }

  if ((packet_len > BNO08X_MAX_REASONABLE_PACKET_LEN) || (*channel >= BNO08X_MAX_CHANNELS))
  {
    *payload_len = 0U;
    BNO08X_CS_Deselect();
    g_bno08x_diag.rx_invalid_headers++;
    return HAL_ERROR;
  }

  data_len = (uint16_t)(packet_len - BNO08X_SHTP_HEADER_SIZE);
  to_copy = (data_len <= payload_size) ? data_len : payload_size;

  if (to_copy > 0U)
  {
    status = BNO08X_SPI_Receive_IT_Blocking(payload, to_copy, BNO08X_SPI_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      BNO08X_CS_Deselect();
      return status;
    }
  }

  if (data_len > to_copy)
  {
    uint8_t drain[16];
    uint16_t remaining = (uint16_t)(data_len - to_copy);
    while (remaining > 0U)
    {
      uint16_t chunk = (remaining > sizeof(drain)) ? (uint16_t)sizeof(drain) : remaining;
      status = BNO08X_SPI_Receive_IT_Blocking(drain, chunk, BNO08X_SPI_TIMEOUT_MS);
      if (status != HAL_OK)
      {
        BNO08X_CS_Deselect();
        return status;
      }
      remaining = (uint16_t)(remaining - chunk);
    }
  }

  *payload_len = data_len;
  BNO08X_CS_Deselect();

  return HAL_OK;
}

static HAL_StatusTypeDef BNO08X_SPI_Transmit_IT_Blocking(const uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;
  uint32_t start_ms;

  if ((data == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

  g_spi_tx_done = 0U;
  g_spi_error = 0U;

  status = HAL_SPI_Transmit_IT(&hspi2, (uint8_t *)data, size);
  if (status != HAL_OK)
  {
    return status;
  }

  start_ms = HAL_GetTick();
  while (g_spi_tx_done == 0U)
  {
    if (g_spi_error != 0U)
    {
      return HAL_ERROR;
    }
    if ((HAL_GetTick() - start_ms) > timeout_ms)
    {
      (void)HAL_SPI_Abort(&hspi2);
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef BNO08X_SPI_Receive_IT_Blocking(uint8_t *data, uint16_t size, uint32_t timeout_ms)
{
  HAL_StatusTypeDef status;
  uint32_t start_ms;

  if ((data == NULL) || (size == 0U))
  {
    return HAL_ERROR;
  }

  g_spi_rx_done = 0U;
  g_spi_error = 0U;

  status = HAL_SPI_Receive_IT(&hspi2, data, size);
  if (status != HAL_OK)
  {
    return status;
  }

  start_ms = HAL_GetTick();
  while (g_spi_rx_done == 0U)
  {
    if (g_spi_error != 0U)
    {
      return HAL_ERROR;
    }
    if ((HAL_GetTick() - start_ms) > timeout_ms)
    {
      (void)HAL_SPI_Abort(&hspi2);
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

static HAL_StatusTypeDef BNO08X_SendSoftReset(void)
{
  const uint8_t soft_reset_cmd[1] = {0x01U};
  return BNO08X_SPI_WritePacket(BNO08X_CHANNEL_EXECUTABLE, soft_reset_cmd, sizeof(soft_reset_cmd));
}

static HAL_StatusTypeDef BNO08X_SendProductIdRequest(void)
{
  const uint8_t request[2] = {BNO08X_REPORT_PRODUCT_ID_REQUEST, 0x00U};
  return BNO08X_SPI_WritePacket(BNO08X_CHANNEL_CONTROL, request, sizeof(request));
}

static uint32_t BNO08X_ReadU32LE(const uint8_t *buf)
{
  return ((uint32_t)buf[0]) |
         ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[3] << 24);
}

static void BNO08X_UpdateDiagFromPayload(uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
  uint16_t i;
  uint16_t copy_len;

  g_bno08x_diag.last_channel = channel;
  g_bno08x_diag.last_rx_len = payload_len;
  g_bno08x_diag.last_report_id = (payload_len > 0U) ? payload[0] : 0U;

  copy_len = (payload_len < sizeof(g_bno08x_diag.last_payload)) ? payload_len : sizeof(g_bno08x_diag.last_payload);
  for (i = 0; i < copy_len; i++)
  {
    g_bno08x_diag.last_payload[i] = payload[i];
  }

  if ((channel == BNO08X_CHANNEL_CONTROL) && (payload_len > 0U) && (payload[0] == BNO08X_REPORT_PRODUCT_ID_RESP))
  {
    if (payload_len >= 8U)
    {
      g_bno08x_diag.product_id_read = BNO08X_ReadU32LE(&payload[4]);
      g_bno08x_diag.product_id_valid = 1U;
      g_bno08x_diag.product_id_match = (g_bno08x_diag.product_id_read == g_bno08x_diag.product_id_etalon) ? 1U : 0U;
    }
    g_bno08x_diag.link_ok = 1U;
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
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  uint32_t next_product_id_request_ms = HAL_GetTick() + 300U;
  uint8_t rx_payload[BNO08X_MAX_PACKET_PAYLOAD];
  uint16_t rx_len = 0U;
  uint8_t rx_channel = 0U;

  BNO08X_CS_Deselect();
  HAL_Delay(10);

  if (BNO08X_HardwareResetAndBoot() == HAL_OK)
  {
    g_bno08x_diag.tx_attempts++;
    if (BNO08X_SendSoftReset() == HAL_OK)
    {
      g_bno08x_diag.tx_ok++;
      g_bno08x_int_pending = 1U;
    }
    else
    {
      g_bno08x_diag.tx_errors++;
    }
  }
  else
  {
    g_bno08x_diag.tx_errors++;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if ((int32_t)(HAL_GetTick() - next_product_id_request_ms) >= 0)
    {
      HAL_StatusTypeDef wake_status = BNO08X_Wake();
      if (wake_status != HAL_OK)
      {
        g_bno08x_diag.tx_errors++;
        next_product_id_request_ms = HAL_GetTick() + 100U;
        continue;
      }

      g_bno08x_diag.tx_attempts++;
      if (BNO08X_SendProductIdRequest() == HAL_OK)
      {
        g_bno08x_diag.tx_ok++;
      }
      else
      {
        g_bno08x_diag.tx_errors++;
      }

      next_product_id_request_ms = HAL_GetTick() + (g_bno08x_diag.link_ok ? 3000U : 1000U);
    }

    if ((g_bno08x_int_pending != 0U) || (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == GPIO_PIN_RESET))
    {
      uint8_t drain_count = 0U;
      g_bno08x_int_pending = 0U;

      while (drain_count < BNO08X_RX_DRAIN_MAX_PACKETS)
      {
        HAL_StatusTypeDef read_status = BNO08X_SPI_ReadPacket(rx_payload, sizeof(rx_payload), &rx_len, &rx_channel);
        if (read_status == HAL_OK)
        {
          if (rx_len > 0U)
          {
            g_bno08x_diag.rx_packets++;
            BNO08X_UpdateDiagFromPayload(rx_channel, rx_payload, rx_len);
          }
        }
        else if (read_status == HAL_BUSY)
        {
          break;
        }
        else
        {
          g_bno08x_diag.rx_errors++;
          break;
        }

        drain_count++;
        if (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) != GPIO_PIN_RESET)
        {
          break;
        }
      }

      if (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin) == GPIO_PIN_RESET)
      {
        g_bno08x_int_pending = 1U;
      }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI2)
  {
    g_spi_tx_done = 1U;
  }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI2)
  {
    g_spi_rx_done = 1U;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI2)
  {
    g_spi_error = 1U;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT_Pin)
  {
    g_bno08x_int_pending = 1U;
  }
}

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
