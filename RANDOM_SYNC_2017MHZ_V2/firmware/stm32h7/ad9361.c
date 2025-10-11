/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ad9361.c
  * @brief          : AD9361 radio transceiver driver for RANDOM_SYNC_2017MHZ_V2
  * @author         : WiWi Development Team
  * @version        : 1.0.0
  * @date           : January 2025
  ******************************************************************************
  * @attention
  *
  * AD9361 Driver Implementation for STM32H7
  * Supports 70 MHz - 6 GHz operation
  * Hardware timestamping for PTP synchronization
  *
  * Based on Analog Devices AD9361 driver
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ad9361.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/

/* AD9361 Register Addresses */
#define AD9361_SPI_CONFIG_A         0x000
#define AD9361_SPI_CONFIG_B         0x001
#define AD9361_MULTICHIP_SYNC       0x002
#define AD9361_TX_LO_FREQ_MSB       0x05A
#define AD9361_TX_LO_FREQ_LSB       0x05B
#define AD9361_RX_LO_FREQ_MSB       0x05C
#define AD9361_RX_LO_FREQ_LSB       0x05D
#define AD9361_TX_BB_FILTER_TUNE    0x0C4
#define AD9361_RX_BB_FILTER_TUNE    0x0C5
#define AD9361_TX_RF_BW             0x0C6
#define AD9361_RX_RF_BW             0x0C7
#define AD9361_TX_GAIN              0x073
#define AD9361_RX_GAIN              0x074
#define AD9361_ADC_CONTROL          0x001
#define AD9361_ADC_STATUS           0x002
#define AD9361_PRODUCT_ID           0x037

/* AD9361 Constants */
#define AD9361_SPI_READ             0x80
#define AD9361_SPI_WRITE            0x00
#define AD9361_DEFAULT_SPI_FREQ     8000000  // 8 MHz

/* Reference clock frequency */
#define AD9361_REF_CLK              40000000ULL  // 40 MHz

/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef *ad9361_spi_handle;
static GPIO_TypeDef *ad9361_cs_port;
static uint16_t ad9361_cs_pin;
static GPIO_TypeDef *ad9361_reset_port;
static uint16_t ad9361_reset_pin;
static GPIO_TypeDef *ad9361_enable_port;
static uint16_t ad9361_enable_pin;

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef ad9361_spi_transfer(uint16_t address, uint8_t *tx_data, uint8_t *rx_data, uint16_t size);
static HAL_StatusTypeDef ad9361_write_register(uint16_t address, uint8_t data);
static HAL_StatusTypeDef ad9361_read_register(uint16_t address, uint8_t *data);
static uint64_t ad9361_calculate_tuning_word(uint64_t frequency, uint32_t ref_clock);
static HAL_StatusTypeDef ad9361_configure_clocking(void);
static HAL_StatusTypeDef ad9361_configure_baseband(void);
static HAL_StatusTypeDef ad9361_calibrate(void);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Initialize AD9361 radio transceiver
  * @param  ctx: AD9361 context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_init(ad9361_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Store GPIO handles */
  ad9361_spi_handle = ctx->spi_handle;
  ad9361_cs_port = ctx->cs_port;
  ad9361_cs_pin = ctx->cs_pin;
  ad9361_reset_port = ctx->reset_port;
  ad9361_reset_pin = ctx->reset_pin;
  ad9361_enable_port = ctx->enable_port;
  ad9361_enable_pin = ctx->enable_pin;

  /* Initialize GPIO */
  HAL_GPIO_WritePin(ad9361_cs_port, ad9361_cs_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ad9361_reset_port, ad9361_reset_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ad9361_enable_port, ad9361_enable_pin, GPIO_PIN_RESET);

  /* Hardware reset */
  HAL_Delay(10);
  HAL_GPIO_WritePin(ad9361_reset_port, ad9361_reset_pin, GPIO_PIN_SET);
  HAL_Delay(50);

  /* Check product ID */
  uint8_t product_id;
  ret = ad9361_read_register(AD9361_PRODUCT_ID, &product_id);
  if (ret != HAL_OK || product_id != 0x0A) {
    return HAL_ERROR;
  }

  /* Enable AD9361 */
  HAL_GPIO_WritePin(ad9361_enable_port, ad9361_enable_pin, GPIO_PIN_SET);

  /* Configure clocking */
  ret = ad9361_configure_clocking();
  if (ret != HAL_OK) {
    return ret;
  }

  /* Configure baseband */
  ret = ad9361_configure_baseband();
  if (ret != HAL_OK) {
    return ret;
  }

  /* Calibrate */
  ret = ad9361_calibrate();
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->initialized = true;
  ctx->tx_enabled = false;
  ctx->rx_enabled = false;

  return HAL_OK;
}

/**
  * @brief  Configure AD9361 parameters
  * @param  ctx: AD9361 context structure
  * @param  config: Configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_configure(ad9361_context_t *ctx, ad9361_config_t *config)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || config == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  /* Set TX frequency */
  ret = ad9361_set_tx_frequency(ctx, config->tx_frequency);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Set RX frequency */
  ret = ad9361_set_rx_frequency(ctx, config->rx_frequency);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Set TX bandwidth */
  ret = ad9361_set_tx_bandwidth(ctx, config->tx_bandwidth);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Set RX bandwidth */
  ret = ad9361_set_rx_bandwidth(ctx, config->rx_bandwidth);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Set TX gain */
  ret = ad9361_set_tx_gain(ctx, config->tx_gain);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Set RX gain */
  ret = ad9361_set_rx_gain(ctx, config->rx_gain);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Update context */
  ctx->tx_frequency = config->tx_frequency;
  ctx->rx_frequency = config->rx_frequency;
  ctx->tx_bandwidth = config->tx_bandwidth;
  ctx->rx_bandwidth = config->rx_bandwidth;
  ctx->sample_rate = config->sample_rate;
  ctx->tx_gain = config->tx_gain;
  ctx->rx_gain = config->rx_gain;

  return HAL_OK;
}

/**
  * @brief  Set TX frequency
  * @param  ctx: AD9361 context structure
  * @param  frequency: Frequency in Hz
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_set_tx_frequency(ad9361_context_t *ctx, uint64_t frequency)
{
  uint64_t tuning_word = ad9361_calculate_tuning_word(frequency, AD9361_REF_CLK);
  uint8_t tx_lo_msb = (tuning_word >> 8) & 0xFF;
  uint8_t tx_lo_lsb = tuning_word & 0xFF;

  HAL_StatusTypeDef ret = ad9361_write_register(AD9361_TX_LO_FREQ_MSB, tx_lo_msb);
  if (ret != HAL_OK) {
    return ret;
  }

  ret = ad9361_write_register(AD9361_TX_LO_FREQ_LSB, tx_lo_lsb);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->tx_frequency = frequency;
  return HAL_OK;
}

/**
  * @brief  Set RX frequency
  * @param  ctx: AD9361 context structure
  * @param  frequency: Frequency in Hz
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_set_rx_frequency(ad9361_context_t *ctx, uint64_t frequency)
{
  uint64_t tuning_word = ad9361_calculate_tuning_word(frequency, AD9361_REF_CLK);
  uint8_t rx_lo_msb = (tuning_word >> 8) & 0xFF;
  uint8_t rx_lo_lsb = tuning_word & 0xFF;

  HAL_StatusTypeDef ret = ad9361_write_register(AD9361_RX_LO_FREQ_MSB, rx_lo_msb);
  if (ret != HAL_OK) {
    return ret;
  }

  ret = ad9361_write_register(AD9361_RX_LO_FREQ_LSB, rx_lo_lsb);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->rx_frequency = frequency;
  return HAL_OK;
}

/**
  * @brief  Set TX bandwidth
  * @param  ctx: AD9361 context structure
  * @param  bandwidth: Bandwidth in Hz
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_set_tx_bandwidth(ad9361_context_t *ctx, uint32_t bandwidth)
{
  /* Calculate bandwidth register value */
  uint8_t bw_value = 0;
  if (bandwidth <= 1750000) bw_value = 0x00;    // 1.75 MHz
  else if (bandwidth <= 3500000) bw_value = 0x01;  // 3.5 MHz
  else if (bandwidth <= 5000000) bw_value = 0x02;  // 5 MHz
  else if (bandwidth <= 7000000) bw_value = 0x03;  // 7 MHz
  else if (bandwidth <= 10000000) bw_value = 0x04; // 10 MHz
  else if (bandwidth <= 14000000) bw_value = 0x05; // 14 MHz
  else if (bandwidth <= 20000000) bw_value = 0x06; // 20 MHz
  else bw_value = 0x07;  // 28 MHz

  HAL_StatusTypeDef ret = ad9361_write_register(AD9361_TX_RF_BW, bw_value);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->tx_bandwidth = bandwidth;
  return HAL_OK;
}

/**
  * @brief  Set RX bandwidth
  * @param  ctx: AD9361 context structure
  * @param  bandwidth: Bandwidth in Hz
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_set_rx_bandwidth(ad9361_context_t *ctx, uint32_t bandwidth)
{
  /* Calculate bandwidth register value */
  uint8_t bw_value = 0;
  if (bandwidth <= 1750000) bw_value = 0x00;    // 1.75 MHz
  else if (bandwidth <= 3500000) bw_value = 0x01;  // 3.5 MHz
  else if (bandwidth <= 5000000) bw_value = 0x02;  // 5 MHz
  else if (bandwidth <= 7000000) bw_value = 0x03;  // 7 MHz
  else if (bandwidth <= 10000000) bw_value = 0x04; // 10 MHz
  else if (bandwidth <= 14000000) bw_value = 0x05; // 14 MHz
  else if (bandwidth <= 20000000) bw_value = 0x06; // 20 MHz
  else bw_value = 0x07;  // 28 MHz

  HAL_StatusTypeDef ret = ad9361_write_register(AD9361_RX_RF_BW, bw_value);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->rx_bandwidth = bandwidth;
  return HAL_OK;
}

/**
  * @brief  Set TX gain
  * @param  ctx: AD9361 context structure
  * @param  gain: Gain in dB
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_set_tx_gain(ad9361_context_t *ctx, int32_t gain)
{
  /* Clamp gain to valid range */
  if (gain < -89) gain = -89;
  if (gain > 0) gain = 0;

  uint8_t gain_value = (uint8_t)(-gain);  // Convert to register value

  HAL_StatusTypeDef ret = ad9361_write_register(AD9361_TX_GAIN, gain_value);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->tx_gain = gain;
  return HAL_OK;
}

/**
  * @brief  Set RX gain
  * @param  ctx: AD9361 context structure
  * @param  gain: Gain in dB
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_set_rx_gain(ad9361_context_t *ctx, int32_t gain)
{
  /* Clamp gain to valid range */
  if (gain < 0) gain = 0;
  if (gain > 73) gain = 73;

  uint8_t gain_value = (uint8_t)gain;

  HAL_StatusTypeDef ret = ad9361_write_register(AD9361_RX_GAIN, gain_value);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->rx_gain = gain;
  return HAL_OK;
}

/**
  * @brief  Enable TX operation
  * @param  ctx: AD9361 context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_enable_tx(ad9361_context_t *ctx)
{
  if (!ctx->initialized) {
    return HAL_ERROR;
  }

  /* Enable TX frontend */
  uint8_t reg_value;
  HAL_StatusTypeDef ret = ad9361_read_register(AD9361_SPI_CONFIG_A, &reg_value);
  if (ret != HAL_OK) {
    return ret;
  }

  reg_value |= (1 << 0);  // Enable TX
  ret = ad9361_write_register(AD9361_SPI_CONFIG_A, reg_value);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->tx_enabled = true;
  return HAL_OK;
}

/**
  * @brief  Enable RX operation
  * @param  ctx: AD9361 context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_enable_rx(ad9361_context_t *ctx)
{
  if (!ctx->initialized) {
    return HAL_ERROR;
  }

  /* Enable RX frontend */
  uint8_t reg_value;
  HAL_StatusTypeDef ret = ad9361_read_register(AD9361_SPI_CONFIG_A, &reg_value);
  if (ret != HAL_OK) {
    return ret;
  }

  reg_value |= (1 << 1);  // Enable RX
  ret = ad9361_write_register(AD9361_SPI_CONFIG_A, reg_value);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->rx_enabled = true;
  return HAL_OK;
}

/**
  * @brief  Disable TX operation
  * @param  ctx: AD9361 context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_disable_tx(ad9361_context_t *ctx)
{
  /* Disable TX frontend */
  uint8_t reg_value;
  HAL_StatusTypeDef ret = ad9361_read_register(AD9361_SPI_CONFIG_A, &reg_value);
  if (ret != HAL_OK) {
    return ret;
  }

  reg_value &= ~(1 << 0);  // Disable TX
  ret = ad9361_write_register(AD9361_SPI_CONFIG_A, reg_value);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->tx_enabled = false;
  return HAL_OK;
}

/**
  * @brief  Disable RX operation
  * @param  ctx: AD9361 context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_disable_rx(ad9361_context_t *ctx)
{
  /* Disable RX frontend */
  uint8_t reg_value;
  HAL_StatusTypeDef ret = ad9361_read_register(AD9361_SPI_CONFIG_A, &reg_value);
  if (ret != HAL_OK) {
    return ret;
  }

  reg_value &= ~(1 << 1);  // Disable RX
  ret = ad9361_write_register(AD9361_SPI_CONFIG_A, reg_value);
  if (ret != HAL_OK) {
    return ret;
  }

  ctx->rx_enabled = false;
  return HAL_OK;
}

/**
  * @brief  Read AD9361 register
  * @param  ctx: AD9361 context structure
  * @param  address: Register address
  * @param  data: Pointer to read data
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_read_register(ad9361_context_t *ctx, uint16_t address, uint8_t *data)
{
  return ad9361_spi_transfer(address | AD9361_SPI_READ, NULL, data, 1);
}

/**
  * @brief  Write AD9361 register
  * @param  ctx: AD9361 context structure
  * @param  address: Register address
  * @param  data: Data to write
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_write_register(ad9361_context_t *ctx, uint16_t address, uint8_t data)
{
  return ad9361_spi_transfer(address | AD9361_SPI_WRITE, &data, NULL, 1);
}

/**
  * @brief  Get AD9361 status
  * @param  ctx: AD9361 context structure
  * @param  status: Status information
  * @retval HAL status
  */
HAL_StatusTypeDef ad9361_get_status(ad9361_context_t *ctx, uint32_t *status)
{
  if (ctx == NULL || status == NULL) {
    return HAL_ERROR;
  }

  *status = 0;

  if (ctx->initialized) {
    *status |= (1 << 0);  // Initialized
  }

  if (ctx->tx_enabled) {
    *status |= (1 << 1);  // TX enabled
  }

  if (ctx->rx_enabled) {
    *status |= (1 << 2);  // RX enabled
  }

  return HAL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  AD9361 SPI transfer function
  * @param  ctx: AD9361 context structure
  * @param  address: Register address
  * @param  tx_data: TX data
  * @param  rx_data: RX data buffer
  * @param  size: Transfer size
  * @retval HAL status
  */
static HAL_StatusTypeDef ad9361_spi_transfer(uint16_t address, uint8_t *tx_data, uint8_t *rx_data, uint16_t size)
{
  HAL_StatusTypeDef ret;
  uint8_t tx_buffer[2];
  uint8_t rx_buffer[2];

  /* Prepare SPI transaction */
  tx_buffer[0] = (address >> 8) & 0xFF;
  tx_buffer[1] = address & 0xFF;

  /* Chip select low */
  HAL_GPIO_WritePin(ad9361_cs_port, ad9361_cs_pin, GPIO_PIN_RESET);

  /* Transfer address */
  ret = HAL_SPI_TransmitReceive(ad9361_spi_handle, tx_buffer, rx_buffer, 2, 1000);
  if (ret != HAL_OK) {
    HAL_GPIO_WritePin(ad9361_cs_port, ad9361_cs_pin, GPIO_PIN_SET);
    return ret;
  }

  /* Transfer data if needed */
  if (tx_data != NULL && size > 0) {
    memset(rx_buffer, 0, sizeof(rx_buffer));
    ret = HAL_SPI_TransmitReceive(ad9361_spi_handle, tx_data, rx_buffer, size, 1000);
    if (ret != HAL_OK) {
      HAL_GPIO_WritePin(ad9361_cs_port, ad9361_cs_pin, GPIO_PIN_SET);
      return ret;
    }
  } else if (rx_data != NULL && size > 0) {
    memset(tx_buffer, 0, sizeof(tx_buffer));
    ret = HAL_SPI_TransmitReceive(ad9361_spi_handle, tx_buffer, rx_buffer, size, 1000);
    if (ret != HAL_OK) {
      HAL_GPIO_WritePin(ad9361_cs_port, ad9361_cs_pin, GPIO_PIN_SET);
      return ret;
    }
    memcpy(rx_data, rx_buffer, size);
  }

  /* Chip select high */
  HAL_GPIO_WritePin(ad9361_cs_port, ad9361_cs_pin, GPIO_PIN_SET);

  return HAL_OK;
}

/**
  * @brief  Calculate frequency tuning word
  * @param  frequency: Target frequency in Hz
  * @param  ref_clock: Reference clock frequency
  * @retval Tuning word
  */
static uint64_t ad9361_calculate_tuning_word(uint64_t frequency, uint32_t ref_clock)
{
  /* Calculate tuning word: frequency * 2^32 / ref_clock */
  uint64_t tuning_word = (frequency * 4294967296ULL) / ref_clock;
  return tuning_word;
}

/**
  * @brief  Configure AD9361 clocking
  * @param  ctx: AD9361 context structure
  * @retval HAL status
  */
static HAL_StatusTypeDef ad9361_configure_clocking(void)
{
  HAL_StatusTypeDef ret;

  /* Configure SPI interface */
  ret = ad9361_write_register(AD9361_SPI_CONFIG_A, 0x00);
  if (ret != HAL_OK) return ret;

  ret = ad9361_write_register(AD9361_SPI_CONFIG_B, 0x00);
  if (ret != HAL_OK) return ret;

  /* Configure multichip sync */
  ret = ad9361_write_register(AD9361_MULTICHIP_SYNC, 0x00);
  if (ret != HAL_OK) return ret;

  return HAL_OK;
}

/**
  * @brief  Configure AD9361 baseband
  * @param  ctx: AD9361 context structure
  * @retval HAL status
  */
static HAL_StatusTypeDef ad9361_configure_baseband(void)
{
  HAL_StatusTypeDef ret;

  /* Configure TX baseband filter */
  ret = ad9361_write_register(AD9361_TX_BB_FILTER_TUNE, 0x1E);
  if (ret != HAL_OK) return ret;

  /* Configure RX baseband filter */
  ret = ad9361_write_register(AD9361_RX_BB_FILTER_TUNE, 0x1E);
  if (ret != HAL_OK) return ret;

  /* Configure ADC */
  ret = ad9361_write_register(AD9361_ADC_CONTROL, 0x01);
  if (ret != HAL_OK) return ret;

  return HAL_OK;
}

/**
  * @brief  Calibrate AD9361
  * @param  ctx: AD9361 context structure
  * @retval HAL status
  */
static HAL_StatusTypeDef ad9361_calibrate(void)
{
  HAL_StatusTypeDef ret;

  /* Start calibration sequence */
  ret = ad9361_write_register(0x016, 0x01);  // Start RX calibration
  if (ret != HAL_OK) return ret;

  HAL_Delay(100);  // Wait for calibration

  ret = ad9361_write_register(0x016, 0x00);  // Stop calibration
  if (ret != HAL_OK) return ret;

  /* Check calibration status */
  uint8_t cal_status;
  ret = ad9361_read_register(AD9361_ADC_STATUS, &cal_status);
  if (ret != HAL_OK) return ret;

  if (cal_status != 0x01) {
    return HAL_ERROR;  // Calibration failed
  }

  return HAL_OK;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
