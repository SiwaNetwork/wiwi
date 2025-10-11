#ifndef __FPGA_INTERFACE_H
#define __FPGA_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* FPGA Context Structure */
typedef struct {
    SPI_HandleTypeDef *spi_handle;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *done_port;
    uint16_t done_pin;

    /* FPGA Status */
    bool configured;
    bool initialized;
    uint32_t version;
    uint32_t build_date;

    /* I/Q Data Buffers */
    uint16_t *iq_tx_buffer;
    uint16_t *iq_rx_buffer;
    uint32_t buffer_size;
    volatile uint32_t tx_buffer_index;
    volatile uint32_t rx_buffer_index;

    /* Synchronization */
    volatile bool data_ready;
    volatile uint32_t sync_counter;

    /* Statistics */
    uint32_t spi_errors;
    uint32_t buffer_overflows;
    uint32_t buffer_underflows;
} fpga_context_t;

/* FPGA Configuration Structure */
typedef struct {
    uint32_t buffer_size;           // Size of I/Q buffers
    uint32_t sample_rate;           // Sample rate in Hz
    uint32_t tx_bandwidth;          // TX bandwidth in Hz
    uint32_t rx_bandwidth;          // RX bandwidth in Hz
    bool enable_filtering;          // Enable digital filtering
    bool enable_sync;               // Enable synchronization
} fpga_config_t;

/* FPGA Status Structure */
typedef struct {
    bool configured;
    bool initialized;
    uint32_t version;
    uint32_t build_date;
    uint32_t buffer_usage_tx;       // TX buffer usage percentage
    uint32_t buffer_usage_rx;       // RX buffer usage percentage
    uint32_t sync_counter;
    uint32_t spi_errors;
    uint32_t buffer_overflows;
    uint32_t buffer_underflows;
} fpga_status_t;

/* FPGA Register Addresses */
#define FPGA_REG_VERSION            0x00
#define FPGA_REG_STATUS             0x01
#define FPGA_REG_CONTROL            0x02
#define FPGA_REG_BUFFER_SIZE        0x03
#define FPGA_REG_SAMPLE_RATE        0x04
#define FPGA_REG_TX_BANDWIDTH       0x05
#define FPGA_REG_RX_BANDWIDTH       0x06
#define FPGA_REG_IQ_TX_PTR          0x10
#define FPGA_REG_IQ_RX_PTR          0x11
#define FPGA_REG_SYNC_COUNTER       0x12
#define FPGA_REG_ERROR_COUNT        0x13

/* FPGA Control Bits */
#define FPGA_CTRL_RESET             (1 << 0)
#define FPGA_CTRL_ENABLE_TX         (1 << 1)
#define FPGA_CTRL_ENABLE_RX         (1 << 2)
#define FPGA_CTRL_ENABLE_SYNC       (1 << 3)
#define FPGA_CTRL_ENABLE_FILTERING  (1 << 4)

/* Exported constants --------------------------------------------------------*/

/* FPGA SPI Configuration */
#define FPGA_SPI_BAUDRATE           SPI_BAUDRATEPRESCALER_16  // 6.25 MHz @ 100 MHz
#define FPGA_SPI_TIMEOUT            1000  // 1 second timeout

/* FPGA Buffer Configuration */
#define FPGA_DEFAULT_BUFFER_SIZE    8192  // 8K samples
#define FPGA_MIN_BUFFER_SIZE        1024  // 1K samples
#define FPGA_MAX_BUFFER_SIZE        32768 // 32K samples

/* FPGA Synchronization */
#define FPGA_SYNC_THRESHOLD         100   // Minimum samples for sync
#define FPGA_SYNC_TIMEOUT           1000  // Sync timeout in ms

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize FPGA interface
 * @param  ctx: FPGA context structure
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_init(fpga_context_t *ctx);

/**
 * @brief  Configure FPGA parameters
 * @param  ctx: FPGA context structure
 * @param  config: Configuration parameters
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_configure(fpga_context_t *ctx, fpga_config_t *config);

/**
 * @brief  Load FPGA bitstream
 * @param  ctx: FPGA context structure
 * @param  bitstream: Bitstream data
 * @param  size: Bitstream size
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_load_bitstream(fpga_context_t *ctx, uint8_t *bitstream, uint32_t size);

/**
 * @brief  Get FPGA status
 * @param  ctx: FPGA context structure
 * @param  status: Status information (output)
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_get_status(fpga_context_t *ctx, fpga_status_t *status);

/**
 * @brief  Start FPGA I/Q streaming
 * @param  ctx: FPGA context structure
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_start_streaming(fpga_context_t *ctx);

/**
 * @brief  Stop FPGA I/Q streaming
 * @param  ctx: FPGA context structure
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_stop_streaming(fpga_context_t *ctx);

/**
 * @brief  Write I/Q data to FPGA TX buffer
 * @param  ctx: FPGA context structure
 * @param  data: I/Q data buffer
 * @param  length: Data length
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_write_iq_data(fpga_context_t *ctx, uint16_t *data, uint32_t length);

/**
 * @brief  Read I/Q data from FPGA RX buffer
 * @param  ctx: FPGA context structure
 * @param  data: I/Q data buffer (output)
 * @param  length: Data length
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_read_iq_data(fpga_context_t *ctx, uint16_t *data, uint32_t length);

/**
 * @brief  Check if FPGA has new data ready
 * @param  ctx: FPGA context structure
 * @retval true if data ready, false otherwise
 */
bool fpga_is_data_ready(fpga_context_t *ctx);

/**
 * @brief  Get FPGA synchronization counter
 * @param  ctx: FPGA context structure
 * @retval Synchronization counter value
 */
uint32_t fpga_get_sync_counter(fpga_context_t *ctx);

/**
 * @brief  Reset FPGA
 * @param  ctx: FPGA context structure
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_reset(fpga_context_t *ctx);

/**
 * @brief  Enable FPGA synchronization
 * @param  ctx: FPGA context structure
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_enable_sync(fpga_context_t *ctx);

/**
 * @brief  Disable FPGA synchronization
 * @param  ctx: FPGA context structure
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_disable_sync(fpga_context_t *ctx);

/**
 * @brief  Get FPGA statistics
 * @param  ctx: FPGA context structure
 * @param  spi_errors: SPI errors counter (output)
 * @param  buffer_overflows: Buffer overflows counter (output)
 * @param  buffer_underflows: Buffer underflows counter (output)
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_get_statistics(fpga_context_t *ctx, uint32_t *spi_errors,
                                     uint32_t *buffer_overflows, uint32_t *buffer_underflows);

/* FPGA Register Access Functions -------------------------------------------*/

/**
 * @brief  Read FPGA register
 * @param  ctx: FPGA context structure
 * @param  address: Register address
 * @param  data: Read data (output)
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_read_register(fpga_context_t *ctx, uint16_t address, uint32_t *data);

/**
 * @brief  Write FPGA register
 * @param  ctx: FPGA context structure
 * @param  address: Register address
 * @param  data: Data to write
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_write_register(fpga_context_t *ctx, uint16_t address, uint32_t data);

/**
 * @brief  Read FPGA version information
 * @param  ctx: FPGA context structure
 * @param  version: Version (output)
 * @param  build_date: Build date (output)
 * @retval HAL status
 */
HAL_StatusTypeDef fpga_get_version(fpga_context_t *ctx, uint32_t *version, uint32_t *build_date);

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
                                          uint8_t *tx_data, uint8_t *rx_data, uint16_t size);

/**
 * @brief  Configure FPGA SPI interface
 * @param  ctx: FPGA context structure
 * @retval HAL status
 */
static HAL_StatusTypeDef fpga_configure_spi(fpga_context_t *ctx);

/**
 * @brief  Check FPGA configuration status
 * @param  ctx: FPGA context structure
 * @retval true if configured, false otherwise
 */
static bool fpga_is_configured(fpga_context_t *ctx);

/**
 * @brief  Wait for FPGA data ready
 * @param  ctx: FPGA context structure
 * @param  timeout_ms: Timeout in milliseconds
 * @retval HAL status
 */
static HAL_StatusTypeDef fpga_wait_data_ready(fpga_context_t *ctx, uint32_t timeout_ms);

/**
 * @brief  Update buffer pointers
 * @param  ctx: FPGA context structure
 * @retval None
 */
static void fpga_update_buffer_pointers(fpga_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __FPGA_INTERFACE_H */
