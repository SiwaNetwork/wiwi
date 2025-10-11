#ifndef __AD9361_H
#define __AD9361_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* AD9361 Context Structure */
typedef struct {
    SPI_HandleTypeDef *spi_handle;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *enable_port;
    uint16_t enable_pin;

    uint64_t tx_frequency;
    uint64_t rx_frequency;
    uint32_t tx_bandwidth;
    uint32_t rx_bandwidth;
    uint32_t sample_rate;
    int32_t tx_gain;
    int32_t rx_gain;

    bool initialized;
    bool tx_enabled;
    bool rx_enabled;
} ad9361_context_t;

/* AD9361 Configuration Structure */
typedef struct {
    uint64_t tx_frequency;      // TX frequency in Hz
    uint64_t rx_frequency;      // RX frequency in Hz
    uint32_t tx_bandwidth;      // TX bandwidth in Hz
    uint32_t rx_bandwidth;      // RX bandwidth in Hz
    uint32_t sample_rate;       // Sample rate in SPS
    int32_t tx_gain;           // TX gain in dB
    int32_t rx_gain;           // RX gain in dB
} ad9361_config_t;

/* Exported constants --------------------------------------------------------*/

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

/* AD9361 Constants */
#define AD9361_SPI_READ             0x80
#define AD9361_SPI_WRITE            0x00
#define AD9361_DEFAULT_SPI_FREQ     8000000  // 8 MHz

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize AD9361 radio transceiver
 * @param  ctx: AD9361 context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_init(ad9361_context_t *ctx);

/**
 * @brief  Configure AD9361 parameters
 * @param  ctx: AD9361 context structure
 * @param  config: Configuration parameters
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_configure(ad9361_context_t *ctx, ad9361_config_t *config);

/**
 * @brief  Set TX frequency
 * @param  ctx: AD9361 context structure
 * @param  frequency: Frequency in Hz
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_set_tx_frequency(ad9361_context_t *ctx, uint64_t frequency);

/**
 * @brief  Set RX frequency
 * @param  ctx: AD9361 context structure
 * @param  frequency: Frequency in Hz
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_set_rx_frequency(ad9361_context_t *ctx, uint64_t frequency);

/**
 * @brief  Set TX bandwidth
 * @param  ctx: AD9361 context structure
 * @param  bandwidth: Bandwidth in Hz
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_set_tx_bandwidth(ad9361_context_t *ctx, uint32_t bandwidth);

/**
 * @brief  Set RX bandwidth
 * @param  ctx: AD9361 context structure
 * @param  bandwidth: Bandwidth in Hz
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_set_rx_bandwidth(ad9361_context_t *ctx, uint32_t bandwidth);

/**
 * @brief  Set TX gain
 * @param  ctx: AD9361 context structure
 * @param  gain: Gain in dB
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_set_tx_gain(ad9361_context_t *ctx, int32_t gain);

/**
 * @brief  Set RX gain
 * @param  ctx: AD9361 context structure
 * @param  gain: Gain in dB
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_set_rx_gain(ad9361_context_t *ctx, int32_t gain);

/**
 * @brief  Enable TX operation
 * @param  ctx: AD9361 context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_enable_tx(ad9361_context_t *ctx);

/**
 * @brief  Enable RX operation
 * @param  ctx: AD9361 context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_enable_rx(ad9361_context_t *ctx);

/**
 * @brief  Disable TX operation
 * @param  ctx: AD9361 context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_disable_tx(ad9361_context_t *ctx);

/**
 * @brief  Disable RX operation
 * @param  ctx: AD9361 context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_disable_rx(ad9361_context_t *ctx);

/**
 * @brief  Read AD9361 register
 * @param  ctx: AD9361 context structure
 * @param  address: Register address
 * @param  data: Pointer to read data
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_read_register(ad9361_context_t *ctx, uint16_t address, uint8_t *data);

/**
 * @brief  Write AD9361 register
 * @param  ctx: AD9361 context structure
 * @param  address: Register address
 * @param  data: Data to write
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_write_register(ad9361_context_t *ctx, uint16_t address, uint8_t data);

/**
 * @brief  Get AD9361 status
 * @param  ctx: AD9361 context structure
 * @param  status: Status information
 * @retval HAL status
 */
HAL_StatusTypeDef ad9361_get_status(ad9361_context_t *ctx, uint32_t *status);

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
static HAL_StatusTypeDef ad9361_spi_transfer(ad9361_context_t *ctx, uint16_t address,
                                           uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

/**
 * @brief  Calculate frequency tuning word
 * @param  frequency: Target frequency in Hz
 * @param  ref_clock: Reference clock frequency
 * @retval Tuning word
 */
static uint64_t ad9361_calculate_tuning_word(uint64_t frequency, uint32_t ref_clock);

/**
 * @brief  Configure AD9361 clocking
 * @param  ctx: AD9361 context structure
 * @retval HAL status
 */
static HAL_StatusTypeDef ad9361_configure_clocking(ad9361_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __AD9361_H */
