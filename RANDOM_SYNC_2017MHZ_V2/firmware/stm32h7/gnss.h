#ifndef __GNSS_H
#define __GNSS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* GNSS Fix Type */
typedef enum {
    GNSS_FIX_NONE,
    GNSS_FIX_2D,
    GNSS_FIX_3D,
    GNSS_FIX_DGPS,
    GNSS_FIX_RTK_FLOAT,
    GNSS_FIX_RTK_FIXED
} gnss_fix_type_t;

/* GNSS System Status */
typedef enum {
    GNSS_STATUS_IDLE,
    GNSS_STATUS_SEARCHING,
    GNSS_STATUS_TRACKING,
    GNSS_STATUS_ERROR
} gnss_status_t;

/* GNSS Context Structure */
typedef struct {
    UART_HandleTypeDef *uart_handle;
    GPIO_TypeDef *pps_port;
    uint16_t pps_pin;
    GPIO_TypeDef *reset_port;
    uint16_t reset_pin;
    GPIO_TypeDef *standby_port;
    uint16_t standby_pin;

    /* Configuration */
    bool enable_gps;
    bool enable_glonass;
    bool enable_galileo;
    bool enable_beidou;
    uint8_t update_rate;  // Hz

    /* Status */
    gnss_status_t status;
    gnss_fix_type_t fix_type;
    uint8_t satellites_used;
    uint8_t satellites_visible;

    /* Position data */
    double latitude;
    double longitude;
    double altitude;
    float speed;
    float course;

    /* Time data */
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    /* Accuracy data */
    float hdop;  // Horizontal dilution of precision
    float pdop;  // Position dilution of precision
    float vdop;  // Vertical dilution of precision

    /* PPS data */
    volatile bool pps_received;
    uint64_t last_pps_time;

    /* Statistics */
    uint32_t messages_received;
    uint32_t checksum_errors;
    uint32_t parse_errors;

    bool initialized;
} gnss_context_t;

/* GNSS Configuration Structure */
typedef struct {
    bool enable_gps;
    bool enable_glonass;
    bool enable_galileo;
    bool enable_beidou;
    uint8_t update_rate;  // Hz (1, 2, 5, 10)
} gnss_config_t;

/* NMEA Message Types */
typedef enum {
    NMEA_MSG_GGA,  // Global Positioning System Fix Data
    NMEA_MSG_GSA,  // GNSS DOP and Active Satellites
    NMEA_MSG_GSV,  // GNSS Satellites in View
    NMEA_MSG_RMC,  // Recommended Minimum Navigation Information
    NMEA_MSG_VTG,  // Track Made Good and Ground Speed
    NMEA_MSG_GLL,  // Geographic Position - Latitude/Longitude
    NMEA_MSG_ZDA   // Time & Date
} nmea_msg_type_t;

/* Exported constants --------------------------------------------------------*/

/* GNSS UART Configuration */
#define GNSS_BAUDRATE           9600
#define GNSS_WORD_LENGTH        UART_WORDLENGTH_8B
#define GNSS_STOP_BITS          UART_STOPBITS_1
#define GNSS_PARITY             UART_PARITY_NONE
#define GNSS_MODE               UART_MODE_TX_RX
#define GNSS_HWCONTROL          UART_HWCONTROL_NONE
#define GNSS_OVERSAMPLING       UART_OVERSAMPLING_16

/* NMEA Message Sizes */
#define NMEA_MAX_MESSAGE_LENGTH 128
#define NMEA_FIELD_MAX_LENGTH   20

/* PPS Timing */
#define PPS_PULSE_WIDTH_US      100
#define PPS_TIMEOUT_MS          2000

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize GNSS receiver
 * @param  ctx: GNSS context structure
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_init(gnss_context_t *ctx);

/**
 * @brief  Configure GNSS parameters
 * @param  ctx: GNSS context structure
 * @param  config: Configuration parameters
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_configure(gnss_context_t *ctx, gnss_config_t *config);

/**
 * @brief  Get GNSS status
 * @param  ctx: GNSS context structure
 * @param  status: Status information
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_get_status(gnss_context_t *ctx, gnss_status_t *status);

/**
 * @brief  Get current position
 * @param  ctx: GNSS context structure
 * @param  latitude: Latitude (output)
 * @param  longitude: Longitude (output)
 * @param  altitude: Altitude (output)
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_get_position(gnss_context_t *ctx, double *latitude,
                                   double *longitude, double *altitude);

/**
 * @brief  Get current time from GNSS
 * @param  ctx: GNSS context structure
 * @param  year: Year (output)
 * @param  month: Month (output)
 * @param  day: Day (output)
 * @param  hour: Hour (output)
 * @param  minute: Minute (output)
 * @param  second: Second (output)
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_get_time(gnss_context_t *ctx, uint16_t *year, uint8_t *month,
                               uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second);

/**
 * @brief  Get PPS status
 * @param  ctx: GNSS context structure
 * @param  pps_received: PPS received flag (output)
 * @param  last_pps_time: Last PPS timestamp (output)
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_get_pps_status(gnss_context_t *ctx, bool *pps_received,
                                    uint64_t *last_pps_time);

/**
 * @brief  Parse NMEA message
 * @param  ctx: GNSS context structure
 * @param  message: NMEA message string
 * @param  length: Message length
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_parse_nmea(gnss_context_t *ctx, char *message, uint16_t length);

/**
 * @brief  Send command to GNSS receiver
 * @param  ctx: GNSS context structure
 * @param  command: Command string
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_send_command(gnss_context_t *ctx, const char *command);

/**
 * @brief  Reset GNSS receiver
 * @param  ctx: GNSS context structure
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_reset(gnss_context_t *ctx);

/**
 * @brief  Put GNSS receiver in standby mode
 * @param  ctx: GNSS context structure
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_standby(gnss_context_t *ctx);

/**
 * @brief  Wake up GNSS receiver from standby
 * @param  ctx: GNSS context structure
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_wakeup(gnss_context_t *ctx);

/**
 * @brief  Get GNSS statistics
 * @param  ctx: GNSS context structure
 * @param  messages_received: Messages received counter (output)
 * @param  checksum_errors: Checksum errors counter (output)
 * @param  parse_errors: Parse errors counter (output)
 * @retval HAL status
 */
HAL_StatusTypeDef gnss_get_statistics(gnss_context_t *ctx, uint32_t *messages_received,
                                     uint32_t *checksum_errors, uint32_t *parse_errors);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Calculate NMEA checksum
 * @param  message: NMEA message
 * @param  length: Message length
 * @retval Checksum value
 */
static uint8_t calculate_nmea_checksum(const char *message, uint16_t length);

/**
 * @brief  Parse GGA message (Global Positioning System Fix Data)
 * @param  ctx: GNSS context structure
 * @param  fields: Message fields
 * @param  field_count: Number of fields
 * @retval HAL status
 */
static HAL_StatusTypeDef parse_gga_message(gnss_context_t *ctx, char **fields, uint8_t field_count);

/**
 * @brief  Parse RMC message (Recommended Minimum Navigation Information)
 * @param  ctx: GNSS context structure
 * @param  fields: Message fields
 * @param  field_count: Number of fields
 * @retval HAL status
 */
static HAL_StatusTypeDef parse_rmc_message(gnss_context_t *ctx, char **fields, uint8_t field_count);

/**
 * @brief  Parse GSA message (GNSS DOP and Active Satellites)
 * @param  ctx: GNSS context structure
 * @param  fields: Message fields
 * @param  field_count: Number of fields
 * @retval HAL status
 */
static HAL_StatusTypeDef parse_gsa_message(gnss_context_t *ctx, char **fields, uint8_t field_count);

/**
 * @brief  Convert NMEA coordinate to decimal degrees
 * @param  nmea_coord: NMEA coordinate string
 * @param  hemisphere: Hemisphere character
 * @retval Coordinate in decimal degrees
 */
static double convert_nmea_to_decimal(const char *nmea_coord, char hemisphere);

/**
 * @brief  PPS interrupt handler
 * @param  ctx: GNSS context structure
 * @retval None
 */
static void pps_interrupt_handler(gnss_context_t *ctx);

/**
 * @brief  UART receive complete callback
 * @param  ctx: GNSS context structure
 * @retval None
 */
static void uart_rx_complete_callback(gnss_context_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* __GNSS_H */
