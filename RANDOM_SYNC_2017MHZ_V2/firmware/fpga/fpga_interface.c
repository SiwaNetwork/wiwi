/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : fpga_interface.c
  * @brief          : FPGA interface driver for RANDOM_SYNC_2017MHZ_V2
  * @author         : WiWi Development Team
  * @version        : 1.0.0
  * @date           : January 2025
  ******************************************************************************
  * @attention
  *
  * FPGA Interface Implementation for STM32H7
  * Provides SPI communication with Lattice ECP5 FPGA
  * Supports I/Q data streaming and synchronization
  *
  * Based on Lattice ECP5 FPGA architecture
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fpga_interface.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>

/* Private defines -----------------------------------------------------------*/

/* FPGA SPI Commands */
#define FPGA_CMD_READ               0x80
#define FPGA_CMD_WRITE              0x00
#define FPGA_CMD_BURST_READ         0xC0
#define FPGA_CMD_BURST_WRITE        0x40

/* FPGA Status Bits */
#define FPGA_STATUS_CONFIGURED      (1 << 0)
#define FPGA_STATUS_INITIALIZED     (1 << 1)
#define FPGA_STATUS_TX_ENABLED      (1 << 2)
#define FPGA_STATUS_RX_ENABLED      (1 << 3)
#define FPGA_STATUS_SYNC_ENABLED    (1 << 4)
#define FPGA_STATUS_ERROR           (1 << 7)

/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef *fpga_spi_handle;
static GPIO_TypeDef *fpga_cs_port;
static uint16_t fpga_cs_pin;
static GPIO_TypeDef *fpga_reset_port;
static uint16_t fpga_reset_pin;
static GPIO_TypeDef *fpga_done_port;
static uint16_t fpga_done_pin;

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Initialize FPGA interface
  * @param  ctx: FPGA context structure
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_init(fpga_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Store hardware handles */
  fpga_spi_handle = ctx->spi_handle;
  fpga_cs_port = ctx->cs_port;
  fpga_cs_pin = ctx->cs_pin;
  fpga_reset_port = ctx->reset_port;
  fpga_reset_pin = ctx->reset_pin;
  fpga_done_port = ctx->done_port;
  fpga_done_pin = ctx->done_pin;

  /* Initialize GPIO */
  HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(fpga_reset_port, fpga_reset_pin, GPIO_PIN_RESET);

  /* Hardware reset FPGA */
  HAL_Delay(10);
  HAL_GPIO_WritePin(fpga_reset_port, fpga_reset_pin, GPIO_PIN_SET);
  HAL_Delay(50);

  /* Configure SPI interface */
  ret = fpga_configure_spi(ctx);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Initialize I/Q buffers */
  ctx->buffer_size = FPGA_DEFAULT_BUFFER_SIZE;
  ctx->iq_tx_buffer = (uint16_t *)malloc(ctx->buffer_size * sizeof(uint16_t));
  ctx->iq_rx_buffer = (uint16_t *)malloc(ctx->buffer_size * sizeof(uint16_t));

  if (ctx->iq_tx_buffer == NULL || ctx->iq_rx_buffer == NULL) {
    return HAL_ERROR;
  }

  memset(ctx->iq_tx_buffer, 0, ctx->buffer_size * sizeof(uint16_t));
  memset(ctx->iq_rx_buffer, 0, ctx->buffer_size * sizeof(uint16_t));

  ctx->tx_buffer_index = 0;
  ctx->rx_buffer_index = 0;
  ctx->data_ready = false;
  ctx->sync_counter = 0;

  /* Initialize statistics */
  ctx->spi_errors = 0;
  ctx->buffer_overflows = 0;
  ctx->buffer_underflows = 0;

  /* Check if FPGA is configured */
  ctx->configured = fpga_is_configured(ctx);

  if (ctx->configured) {
    ctx->initialized = true;
  } else {
    ctx->initialized = false;
  }

  return HAL_OK;
}

/**
  * @brief  Configure FPGA parameters
  * @param  ctx: FPGA context structure
  * @param  config: Configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_configure(fpga_context_t *ctx, fpga_config_t *config)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || config == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  /* Set buffer size */
  if (config->buffer_size >= FPGA_MIN_BUFFER_SIZE &&
      config->buffer_size <= FPGA_MAX_BUFFER_SIZE) {
    ctx->buffer_size = config->buffer_size;
  }

  /* Configure sample rate */
  ret = fpga_write_register(ctx, FPGA_REG_SAMPLE_RATE, config->sample_rate);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Configure bandwidths */
  ret = fpga_write_register(ctx, FPGA_REG_TX_BANDWIDTH, config->tx_bandwidth);
  if (ret != HAL_OK) {
    return ret;
  }

  ret = fpga_write_register(ctx, FPGA_REG_RX_BANDWIDTH, config->rx_bandwidth);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Configure control register */
  uint32_t control = 0;
  if (config->enable_filtering) control |= FPGA_CTRL_ENABLE_FILTERING;
  if (config->enable_sync) control |= FPGA_CTRL_ENABLE_SYNC;

  ret = fpga_write_register(ctx, FPGA_REG_CONTROL, control);
  if (ret != HAL_OK) {
    return ret;
  }

  return HAL_OK;
}

/**
  * @brief  Load FPGA bitstream
  * @param  ctx: FPGA context structure
  * @param  bitstream: Bitstream data
  * @param  size: Bitstream size
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_load_bitstream(fpga_context_t *ctx, uint8_t *bitstream, uint32_t size)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || bitstream == NULL || size == 0) {
    return HAL_ERROR;
  }

  /* Reset FPGA before loading */
  ret = fpga_reset(ctx);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Load bitstream via SPI (simplified implementation) */
  /* In real implementation, this would use FPGA configuration interface */

  /* Wait for configuration to complete */
  HAL_Delay(100);

  /* Check if FPGA is now configured */
  ctx->configured = fpga_is_configured(ctx);

  if (ctx->configured) {
    ctx->initialized = true;
    return HAL_OK;
  } else {
    return HAL_ERROR;
  }
}

/**
  * @brief  Get FPGA status
  * @param  ctx: FPGA context structure
  * @param  status: Status information (output)
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_get_status(fpga_context_t *ctx, fpga_status_t *status)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || status == NULL) {
    return HAL_ERROR;
  }

  status->configured = ctx->configured;
  status->initialized = ctx->initialized;

  /* Get version information */
  ret = fpga_get_version(ctx, &status->version, &status->build_date);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Calculate buffer usage */
  status->buffer_usage_tx = (ctx->tx_buffer_index * 100) / ctx->buffer_size;
  status->buffer_usage_rx = (ctx->rx_buffer_index * 100) / ctx->buffer_size;

  status->sync_counter = ctx->sync_counter;
  status->spi_errors = ctx->spi_errors;
  status->buffer_overflows = ctx->buffer_overflows;
  status->buffer_underflows = ctx->buffer_underflows;

  return HAL_OK;
}

/**
  * @brief  Start FPGA I/Q streaming
  * @param  ctx: FPGA context structure
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_start_streaming(fpga_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  /* Enable TX and RX */
  uint32_t control;
  ret = fpga_read_register(ctx, FPGA_REG_CONTROL, &control);
  if (ret != HAL_OK) {
    return ret;
  }

  control |= (FPGA_CTRL_ENABLE_TX | FPGA_CTRL_ENABLE_RX);
  ret = fpga_write_register(ctx, FPGA_REG_CONTROL, control);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Reset buffer pointers */
  fpga_update_buffer_pointers(ctx);

  return HAL_OK;
}

/**
  * @brief  Stop FPGA I/Q streaming
  * @param  ctx: FPGA context structure
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_stop_streaming(fpga_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  /* Disable TX and RX */
  uint32_t control;
  ret = fpga_read_register(ctx, FPGA_REG_CONTROL, &control);
  if (ret != HAL_OK) {
    return ret;
  }

  control &= ~(FPGA_CTRL_ENABLE_TX | FPGA_CTRL_ENABLE_RX);
  ret = fpga_write_register(ctx, FPGA_REG_CONTROL, control);
  if (ret != HAL_OK) {
    return ret;
  }

  return HAL_OK;
}

/**
  * @brief  Write I/Q data to FPGA TX buffer
  * @param  ctx: FPGA context structure
  * @param  data: I/Q data buffer
  * @param  length: Data length
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_write_iq_data(fpga_context_t *ctx, uint16_t *data, uint32_t length)
{
  if (ctx == NULL || data == NULL || length == 0 || ctx->iq_tx_buffer == NULL) {
    return HAL_ERROR;
  }

  if (length > ctx->buffer_size) {
    ctx->buffer_overflows++;
    length = ctx->buffer_size;
  }

  /* Copy data to TX buffer */
  memcpy(&ctx->iq_tx_buffer[ctx->tx_buffer_index], data, length * sizeof(uint16_t));

  /* Update buffer index */
  ctx->tx_buffer_index += length;
  if (ctx->tx_buffer_index >= ctx->buffer_size) {
    ctx->tx_buffer_index = 0;
  }

  /* Update FPGA buffer pointer */
  fpga_update_buffer_pointers(ctx);

  return HAL_OK;
}

/**
  * @brief  Read I/Q data from FPGA RX buffer
  * @param  ctx: FPGA context structure
  * @param  data: I/Q data buffer (output)
  * @param  length: Data length
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_read_iq_data(fpga_context_t *ctx, uint16_t *data, uint32_t length)
{
  if (ctx == NULL || data == NULL || length == 0 || ctx->iq_rx_buffer == NULL) {
    return HAL_ERROR;
  }

  if (length > ctx->buffer_size) {
    ctx->buffer_underflows++;
    length = ctx->buffer_size;
  }

  /* Copy data from RX buffer */
  memcpy(data, &ctx->iq_rx_buffer[ctx->rx_buffer_index], length * sizeof(uint16_t));

  /* Update buffer index */
  ctx->rx_buffer_index += length;
  if (ctx->rx_buffer_index >= ctx->buffer_size) {
    ctx->rx_buffer_index = 0;
  }

  return HAL_OK;
}

/**
  * @brief  Check if FPGA has new data ready
  * @param  ctx: FPGA context structure
  * @retval true if data ready, false otherwise
  */
bool fpga_is_data_ready(fpga_context_t *ctx)
{
  if (ctx == NULL || !ctx->initialized) {
    return false;
  }

  return ctx->data_ready;
}

/**
  * @brief  Get FPGA synchronization counter
  * @param  ctx: FPGA context structure
  * @retval Synchronization counter value
  */
uint32_t fpga_get_sync_counter(fpga_context_t *ctx)
{
  if (ctx == NULL || !ctx->initialized) {
    return 0;
  }

  return ctx->sync_counter;
}

/**
  * @brief  Reset FPGA
  * @param  ctx: FPGA context structure
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_reset(fpga_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Hardware reset */
  HAL_GPIO_WritePin(fpga_reset_port, fpga_reset_pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(fpga_reset_port, fpga_reset_pin, GPIO_PIN_SET);
  HAL_Delay(50);

  /* Reset context */
  ctx->configured = false;
  ctx->initialized = false;
  ctx->tx_buffer_index = 0;
  ctx->rx_buffer_index = 0;
  ctx->data_ready = false;
  ctx->sync_counter = 0;

  /* Clear buffers */
  memset(ctx->iq_tx_buffer, 0, ctx->buffer_size * sizeof(uint16_t));
  memset(ctx->iq_rx_buffer, 0, ctx->buffer_size * sizeof(uint16_t));

  return HAL_OK;
}

/**
  * @brief  Enable FPGA synchronization
  * @param  ctx: FPGA context structure
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_enable_sync(fpga_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  /* Enable synchronization */
  uint32_t control;
  ret = fpga_read_register(ctx, FPGA_REG_CONTROL, &control);
  if (ret != HAL_OK) {
    return ret;
  }

  control |= FPGA_CTRL_ENABLE_SYNC;
  ret = fpga_write_register(ctx, FPGA_REG_CONTROL, control);
  if (ret != HAL_OK) {
    return ret;
  }

  return HAL_OK;
}

/**
  * @brief  Disable FPGA synchronization
  * @param  ctx: FPGA context structure
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_disable_sync(fpga_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  /* Disable synchronization */
  uint32_t control;
  ret = fpga_read_register(ctx, FPGA_REG_CONTROL, &control);
  if (ret != HAL_OK) {
    return ret;
  }

  control &= ~FPGA_CTRL_ENABLE_SYNC;
  ret = fpga_write_register(ctx, FPGA_REG_CONTROL, control);
  if (ret != HAL_OK) {
    return ret;
  }

  return HAL_OK;
}

/**
  * @brief  Get FPGA statistics
  * @param  ctx: FPGA context structure
  * @param  spi_errors: SPI errors counter (output)
  * @param  buffer_overflows: Buffer overflows counter (output)
  * @param  buffer_underflows: Buffer underflows counter (output)
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_get_statistics(fpga_context_t *ctx, uint32_t *spi_errors,
                                     uint32_t *buffer_overflows, uint32_t *buffer_underflows)
{
  if (ctx == NULL || spi_errors == NULL ||
      buffer_overflows == NULL || buffer_underflows == NULL) {
    return HAL_ERROR;
  }

  *spi_errors = ctx->spi_errors;
  *buffer_overflows = ctx->buffer_overflows;
  *buffer_underflows = ctx->buffer_underflows;

  return HAL_OK;
}

/**
  * @brief  Read FPGA register
  * @param  ctx: FPGA context structure
  * @param  address: Register address
  * @param  data: Read data (output)
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_read_register(fpga_context_t *ctx, uint16_t address, uint32_t *data)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t tx_buffer[4] = {FPGA_CMD_READ, (address >> 8) & 0xFF, address & 0xFF, 0x00};
  uint8_t rx_buffer[4] = {0};

  if (ctx == NULL || data == NULL) {
    return HAL_ERROR;
  }

  /* Chip select low */
  HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_RESET);

  /* SPI transfer */
  ret = HAL_SPI_TransmitReceive(fpga_spi_handle, tx_buffer, rx_buffer, 4, FPGA_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    ctx->spi_errors++;
    HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_SET);
    return ret;
  }

  /* Chip select high */
  HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_SET);

  /* Convert to 32-bit value */
  *data = ((uint32_t)rx_buffer[1] << 24) | ((uint32_t)rx_buffer[2] << 16) |
          ((uint32_t)rx_buffer[3] << 8) | rx_buffer[0];

  return HAL_OK;
}

/**
  * @brief  Write FPGA register
  * @param  ctx: FPGA context structure
  * @param  address: Register address
  * @param  data: Data to write
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_write_register(fpga_context_t *ctx, uint16_t address, uint32_t data)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t tx_buffer[4] = {FPGA_CMD_WRITE, (address >> 8) & 0xFF, address & 0xFF,
                         (data >> 24) & 0xFF};
  uint8_t rx_buffer[4] = {0};

  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Chip select low */
  HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_RESET);

  /* SPI transfer */
  ret = HAL_SPI_TransmitReceive(fpga_spi_handle, tx_buffer, rx_buffer, 4, FPGA_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    ctx->spi_errors++;
    HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_SET);
    return ret;
  }

  /* Chip select high */
  HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_SET);

  return HAL_OK;
}

/**
  * @brief  Read FPGA version information
  * @param  ctx: FPGA context structure
  * @param  version: Version (output)
  * @param  build_date: Build date (output)
  * @retval HAL status
  */
HAL_StatusTypeDef fpga_get_version(fpga_context_t *ctx, uint32_t *version, uint32_t *build_date)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || version == NULL || build_date == NULL) {
    return HAL_ERROR;
  }

  /* Read version register */
  ret = fpga_read_register(ctx, FPGA_REG_VERSION, version);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Read build date register */
  ret = fpga_read_register(ctx, FPGA_REG_STATUS, build_date);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Extract version from status register */
  *build_date = (*build_date >> 16) & 0xFFFF;
  *version = *version & 0xFFFF;

  return HAL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  FPGA SPI transfer function
  * @param  ctx: FPGA context structure
  * @param  address: Register address
  * @param  tx_data: TX data buffer
  * @param  rx_data: RX data buffer
  * @param  size: Transfer size
  * @retval HAL status
  */
static HAL_StatusTypeDef fpga_spi_transfer(fpga_context_t *ctx, uint16_t address,
                                          uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
  HAL_StatusTypeDef ret = HAL_OK;
  uint8_t addr_buffer[2] = {(address >> 8) & 0xFF, address & 0xFF};

  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Chip select low */
  HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_RESET);

  /* Send address */
  ret = HAL_SPI_Transmit(fpga_spi_handle, addr_buffer, 2, FPGA_SPI_TIMEOUT);
  if (ret != HAL_OK) {
    ctx->spi_errors++;
    HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_SET);
    return ret;
  }

  /* Send/receive data */
  if (tx_data != NULL && rx_data != NULL) {
    ret = HAL_SPI_TransmitReceive(fpga_spi_handle, tx_data, rx_data, size, FPGA_SPI_TIMEOUT);
  } else if (tx_data != NULL) {
    ret = HAL_SPI_Transmit(fpga_spi_handle, tx_data, size, FPGA_SPI_TIMEOUT);
  } else if (rx_data != NULL) {
    ret = HAL_SPI_Receive(fpga_spi_handle, rx_data, size, FPGA_SPI_TIMEOUT);
  }

  if (ret != HAL_OK) {
    ctx->spi_errors++;
    HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_SET);
    return ret;
  }

  /* Chip select high */
  HAL_GPIO_WritePin(fpga_cs_port, fpga_cs_pin, GPIO_PIN_SET);

  return HAL_OK;
}

/**
  * @brief  Configure FPGA SPI interface
  * @param  ctx: FPGA context structure
  * @retval HAL status
  */
static HAL_StatusTypeDef fpga_configure_spi(fpga_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (fpga_spi_handle == NULL) {
    return HAL_ERROR;
  }

  /* Configure SPI for FPGA communication */
  fpga_spi_handle->Init.Mode = SPI_MODE_MASTER;
  fpga_spi_handle->Init.Direction = SPI_DIRECTION_2LINES;
  fpga_spi_handle->Init.DataSize = SPI_DATASIZE_8BIT;
  fpga_spi_handle->Init.CLKPolarity = SPI_POLARITY_LOW;
  fpga_spi_handle->Init.CLKPhase = SPI_PHASE_1EDGE;
  fpga_spi_handle->Init.NSS = SPI_NSS_SOFT;
  fpga_spi_handle->Init.BaudRatePrescaler = FPGA_SPI_BAUDRATE;
  fpga_spi_handle->Init.FirstBit = SPI_FIRSTBIT_MSB;
  fpga_spi_handle->Init.TIMode = SPI_TIMODE_DISABLE;
  fpga_spi_handle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  fpga_spi_handle->Init.CRCPolynomial = 10;

  ret = HAL_SPI_Init(fpga_spi_handle);
  if (ret != HAL_OK) {
    return ret;
  }

  return HAL_OK;
}

/**
  * @brief  Check FPGA configuration status
  * @param  ctx: FPGA context structure
  * @retval true if configured, false otherwise
  */
static bool fpga_is_configured(fpga_context_t *ctx)
{
  if (fpga_done_port == NULL || fpga_done_pin == 0) {
    return false;
  }

  /* Check FPGA DONE pin */
  return (HAL_GPIO_ReadPin(fpga_done_port, fpga_done_pin) == GPIO_PIN_SET);
}

/**
  * @brief  Wait for FPGA data ready
  * @param  ctx: FPGA context structure
  * @param  timeout_ms: Timeout in milliseconds
  * @retval HAL status
  */
static HAL_StatusTypeDef fpga_wait_data_ready(fpga_context_t *ctx, uint32_t timeout_ms)
{
  uint32_t start_time = HAL_GetTick();

  while (!ctx->data_ready) {
    if ((HAL_GetTick() - start_time) >= timeout_ms) {
      return HAL_TIMEOUT;
    }
    HAL_Delay(1);
  }

  return HAL_OK;
}

/**
  * @brief  Update buffer pointers
  * @param  ctx: FPGA context structure
  * @retval None
  */
static void fpga_update_buffer_pointers(fpga_context_t *ctx)
{
  /* Update TX buffer pointer */
  fpga_write_register(ctx, FPGA_REG_IQ_TX_PTR, ctx->tx_buffer_index);

  /* Update RX buffer pointer */
  fpga_write_register(ctx, FPGA_REG_IQ_RX_PTR, ctx->rx_buffer_index);

  /* Update sync counter */
  ctx->sync_counter++;
  fpga_write_register(ctx, FPGA_REG_SYNC_COUNTER, ctx->sync_counter);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
