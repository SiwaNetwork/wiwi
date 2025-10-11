#ifndef __PTP_H
#define __PTP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* PTP Clock States */
typedef enum {
    PTP_STATE_INITIALIZING,
    PTP_STATE_FAULTY,
    PTP_STATE_DISABLED,
    PTP_STATE_LISTENING,
    PTP_STATE_PRE_MASTER,
    PTP_STATE_MASTER,
    PTP_STATE_PASSIVE,
    PTP_STATE_UNCALIBRATED,
    PTP_STATE_SLAVE
} ptp_state_t;

/* PTP Clock Classes */
typedef enum {
    PTP_CLASS_PRIMARY = 6,
    PTP_CLASS_LOCKED = 7,
    PTP_CLASS_HOLDOVER = 135,
    PTP_CLASS_FREE_RUNNING = 248
} ptp_clock_class_t;

/* PTP Profiles */
typedef enum {
    PTP_PROFILE_DEFAULT,
    PTP_PROFILE_G8275_1,  // Telecom profile
    PTP_PROFILE_G8265_1,  // Power profile
    PTP_PROFILE_IEEE_C37_238
} ptp_profile_t;

/* PTP Accuracy */
typedef enum {
    PTP_ACCURACY_25NS = 0x20,
    PTP_ACCURACY_100NS = 0x21,
    PTP_ACCURACY_250NS = 0x22,
    PTP_ACCURACY_1US = 0x23,
    PTP_ACCURACY_2_5US = 0x24,
    PTP_ACCURACY_10US = 0x25,
    PTP_ACCURACY_25US = 0x26,
    PTP_ACCURACY_100US = 0x27,
    PTP_ACCURACY_250US = 0x28,
    PTP_ACCURACY_1MS = 0x29,
    PTP_ACCURACY_2_5MS = 0x2A,
    PTP_ACCURACY_10MS = 0x2B,
    PTP_ACCURACY_25MS = 0x2C,
    PTP_ACCURACY_100MS = 0x2D,
    PTP_ACCURACY_250MS = 0x2E,
    PTP_ACCURACY_1S = 0x2F,
    PTP_ACCURACY_10S = 0x30,
    PTP_ACCURACY_10S_PLUS = 0x31
} ptp_accuracy_t;

/* PTP Clock Source */
typedef enum {
    PTP_SOURCE_INTERNAL,
    PTP_SOURCE_GNSS,
    PTP_SOURCE_ATOMIC,
    PTP_SOURCE_PTP,
    PTP_SOURCE_NTP,
    PTP_SOURCE_HAND_SET,
    PTP_SOURCE_OTHER,
    PTP_SOURCE_INTERNAL_OSCILLATOR
} ptp_clock_source_t;

/* PTP Context Structure */
typedef struct {
    ptp_state_t state;
    ptp_clock_class_t clock_class;
    ptp_accuracy_t clock_accuracy;
    ptp_clock_source_t clock_source;
    ptp_profile_t profile;

    uint8_t domain_number;
    uint8_t priority1;
    uint8_t priority2;
    uint8_t clock_identity[8];

    int64_t master_offset;      // Offset from master (nanoseconds)
    int64_t path_delay;         // Path delay (nanoseconds)
    uint32_t steps_removed;     // Steps removed from grandmaster

    bool hardware_timestamping;
    uint32_t sync_interval;     // Sync message interval (log2 seconds)

    /* Statistics */
    uint32_t sync_count;
    uint32_t delay_req_count;
    uint32_t announce_count;
    uint64_t last_sync_time;
    uint64_t last_delay_req_time;
} ptp_context_t;

/* PTP Configuration Structure */
typedef struct {
    ptp_profile_t profile;
    uint8_t domain_number;
    uint8_t priority1;
    uint8_t priority2;
    ptp_clock_class_t clock_class;
    ptp_accuracy_t clock_accuracy;
    ptp_clock_source_t clock_source;
    uint32_t sync_interval;
    bool hardware_timestamping;
} ptp_config_t;

/* PTP Status Structure */
typedef struct {
    ptp_state_t state;
    ptp_clock_class_t clock_class;
    ptp_accuracy_t clock_accuracy;
    int64_t offset_from_master;     // Current offset (ns)
    int64_t mean_path_delay;        // Mean path delay (ns)
    uint32_t observed_variance;     // Observed variance
    uint32_t steps_removed;
    uint32_t grandmaster_identity[8];
    uint64_t last_sync_timestamp;
    uint32_t sync_count;
    uint32_t delay_req_count;
} ptp_status_t;

/* Exported constants --------------------------------------------------------*/

/* PTP Protocol Constants */
#define PTP_ETHERTYPE           0x88F7
#define PTP_VERSION             2
#define PTP_MULTICAST_ADDR      "224.0.1.129"
#define PTP_DEFAULT_PORT        319  // Event port
#define PTP_GENERAL_PORT        320  // General port

/* PTP Message Types */
#define PTP_MSG_SYNC            0x0
#define PTP_MSG_DELAY_REQ       0x1
#define PTP_MSG_FOLLOW_UP       0x8
#define PTP_MSG_DELAY_RESP      0x9
#define PTP_MSG_ANNOUNCE        0xB
#define PTP_MSG_SIGNALING       0xC
#define PTP_MSG_MANAGEMENT      0xD

/* PTP Transport Specific */
#define PTP_TSP_ETHERNET        0x1
#define PTP_TSP_UDP_IPV4        0x2
#define PTP_TSP_UDP_IPV6        0x3

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize PTP subsystem
 * @param  ctx: PTP context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_init(ptp_context_t *ctx);

/**
 * @brief  Configure PTP parameters
 * @param  ctx: PTP context structure
 * @param  config: Configuration parameters
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_configure(ptp_context_t *ctx, ptp_config_t *config);

/**
 * @brief  Set PTP clock source
 * @param  ctx: PTP context structure
 * @param  source: Clock source
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_set_clock_source(ptp_context_t *ctx, ptp_clock_source_t source);

/**
 * @brief  Enable hardware timestamping
 * @param  ctx: PTP context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_enable_hardware_timestamping(ptp_context_t *ctx);

/**
 * @brief  Disable hardware timestamping
 * @param  ctx: PTP context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_disable_hardware_timestamping(ptp_context_t *ctx);

/**
 * @brief  Get PTP status
 * @param  ctx: PTP context structure
 * @param  status: Status information
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_get_status(ptp_context_t *ctx, ptp_status_t *status);

/**
 * @brief  Process received PTP message
 * @param  ctx: PTP context structure
 * @param  message: PTP message buffer
 * @param  length: Message length
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_process_message(ptp_context_t *ctx, uint8_t *message, uint16_t length);

/**
 * @brief  Send PTP sync message
 * @param  ctx: PTP context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_send_sync(ptp_context_t *ctx);

/**
 * @brief  Send PTP delay request
 * @param  ctx: PTP context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_send_delay_req(ptp_context_t *ctx);

/**
 * @brief  Send PTP announce message
 * @param  ctx: PTP context structure
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_send_announce(ptp_context_t *ctx);

/**
 * @brief  Set local clock time
 * @param  ctx: PTP context structure
 * @param  seconds: Seconds since epoch
 * @param  nanoseconds: Nanoseconds
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_set_time(ptp_context_t *ctx, uint64_t seconds, uint32_t nanoseconds);

/**
 * @brief  Get local clock time
 * @param  ctx: PTP context structure
 * @param  seconds: Seconds since epoch (output)
 * @param  nanoseconds: Nanoseconds (output)
 * @retval HAL status
 */
HAL_StatusTypeDef ptp_get_time(ptp_context_t *ctx, uint64_t *seconds, uint32_t *nanoseconds);

/**
 * @brief  Calculate clock offset from master
 * @param  ctx: PTP context structure
 * @param  t1: T1 timestamp (local send time)
 * @param  t2: T2 timestamp (remote receive time)
 * @param  t3: T3 timestamp (remote send time)
 * @param  t4: T4 timestamp (local receive time)
 * @retval Offset in nanoseconds
 */
int64_t ptp_calculate_offset(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4);

/**
 * @brief  Calculate path delay
 * @param  ctx: PTP context structure
 * @param  t1: T1 timestamp (local send time)
 * @param  t2: T2 timestamp (remote receive time)
 * @param  t3: T3 timestamp (remote send time)
 * @param  t4: T4 timestamp (local receive time)
 * @retval Path delay in nanoseconds
 */
int64_t ptp_calculate_delay(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4);

/* PTP Message Structures ---------------------------------------------------*/

/* PTP Header */
typedef struct __attribute__((packed)) {
    uint8_t transport_specific;
    uint8_t message_type;
    uint8_t version_ptp;
    uint16_t message_length;
    uint8_t domain_number;
    uint8_t reserved1;
    uint16_t flags;
    int64_t correction_field;
    uint32_t reserved2;
    uint8_t clock_identity[8];
    uint16_t source_port_id;
    uint16_t sequence_id;
    uint8_t control;
    uint8_t log_message_interval;
} ptp_header_t;

/* PTP Sync Message */
typedef struct __attribute__((packed)) {
    ptp_header_t header;
    uint64_t origin_timestamp_seconds;
    uint32_t origin_timestamp_nanoseconds;
} ptp_sync_msg_t;

/* PTP Delay Request Message */
typedef struct __attribute__((packed)) {
    ptp_header_t header;
    uint64_t origin_timestamp_seconds;
    uint32_t origin_timestamp_nanoseconds;
} ptp_delay_req_msg_t;

/* PTP Delay Response Message */
typedef struct __attribute__((packed)) {
    ptp_header_t header;
    uint64_t receive_timestamp_seconds;
    uint32_t receive_timestamp_nanoseconds;
    uint8_t requesting_clock_identity[8];
    uint16_t requesting_source_port_id;
} ptp_delay_resp_msg_t;

/* PTP Announce Message */
typedef struct __attribute__((packed)) {
    ptp_header_t header;
    int16_t current_utc_offset;
    uint8_t grandmaster_priority1;
    uint8_t grandmaster_clock_quality_class;
    uint8_t grandmaster_clock_quality_accuracy;
    uint16_t grandmaster_clock_quality_offset_scaled_log_variance;
    uint8_t grandmaster_priority2;
    uint8_t grandmaster_clock_identity[8];
    uint16_t steps_removed;
    uint8_t time_source;
} ptp_announce_msg_t;

#ifdef __cplusplus
}
#endif

#endif /* __PTP_H */
