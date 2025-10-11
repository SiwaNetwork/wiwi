#ifndef __PTP_STACK_H
#define __PTP_STACK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <time.h>

/* Exported types ------------------------------------------------------------*/

/* PTP Clock Types */
typedef enum {
    PTP_CLOCK_TYPE_ORDINARY,
    PTP_CLOCK_TYPE_BOUNDARY,
    PTP_CLOCK_TYPE_END_TO_END_TRANSPARENT,
    PTP_CLOCK_TYPE_PEER_TO_PEER_TRANSPARENT,
    PTP_CLOCK_TYPE_MANAGEMENT
} ptp_clock_type_t;

/* PTP Port State */
typedef enum {
    PTP_PORT_STATE_INITIALIZING,
    PTP_PORT_STATE_FAULTY,
    PTP_PORT_STATE_DISABLED,
    PTP_PORT_STATE_LISTENING,
    PTP_PORT_STATE_PRE_MASTER,
    PTP_PORT_STATE_MASTER,
    PTP_PORT_STATE_PASSIVE,
    PTP_PORT_STATE_UNCALIBRATED,
    PTP_PORT_STATE_SLAVE
} ptp_port_state_t;

/* PTP Message Types */
typedef enum {
    PTP_MSG_TYPE_SYNC = 0x0,
    PTP_MSG_TYPE_DELAY_REQ = 0x1,
    PTP_MSG_TYPE_PDELAY_REQ = 0x2,
    PTP_MSG_TYPE_PDELAY_RESP = 0x3,
    PTP_MSG_TYPE_FOLLOW_UP = 0x8,
    PTP_MSG_TYPE_DELAY_RESP = 0x9,
    PTP_MSG_TYPE_PDELAY_RESP_FOLLOW_UP = 0xA,
    PTP_MSG_TYPE_ANNOUNCE = 0xB,
    PTP_MSG_TYPE_SIGNALING = 0xC,
    PTP_MSG_TYPE_MANAGEMENT = 0xD
} ptp_message_type_t;

/* PTP Transport Types */
typedef enum {
    PTP_TRANSPORT_UDP_IPV4 = 1,
    PTP_TRANSPORT_UDP_IPV6 = 2,
    PTP_TRANSPORT_IEEE_802_3 = 3,
    PTP_TRANSPORT_DEVICE_NET = 4,
    PTP_TRANSPORT_CONTROL_NET = 5,
    PTP_TRANSPORT_PROFINET = 6
} ptp_transport_t;

/* PTP Timestamp Structure */
typedef struct {
    uint64_t seconds;
    uint32_t nanoseconds;
} ptp_timestamp_t;

/* PTP Clock Identity */
typedef struct {
    uint8_t id[8];
} ptp_clock_id_t;

/* PTP Port Identity */
typedef struct {
    ptp_clock_id_t clock_id;
    uint16_t port_number;
} ptp_port_id_t;

/* PTP Dataset Structure */
typedef struct {
    uint8_t priority1;
    uint8_t priority2;
    uint8_t clock_class;
    uint8_t clock_accuracy;
    uint16_t offset_scaled_log_variance;
    ptp_clock_id_t clock_id;
    uint16_t steps_removed;
    uint8_t time_source;
} ptp_dataset_t;

/* PTP Stack Configuration */
typedef struct {
    ptp_clock_type_t clock_type;
    ptp_transport_t transport_type;
    uint8_t domain_number;
    uint8_t priority1;
    uint8_t priority2;
    bool enable_hardware_timestamping;
    bool enable_two_step_sync;
    uint32_t sync_interval;  // log2(seconds)
    uint32_t announce_interval;  // log2(seconds)
    uint32_t delay_req_interval;  // log2(seconds)
} ptp_config_t;

/* PTP Stack Context */
typedef struct {
    ptp_config_t config;
    ptp_port_state_t port_state;
    ptp_dataset_t current_dataset;
    ptp_dataset_t parent_dataset;
    ptp_dataset_t grandmaster_dataset;

    /* Timing information */
    ptp_timestamp_t last_sync_timestamp;
    ptp_timestamp_t last_delay_req_timestamp;
    int64_t master_offset_ns;
    int64_t path_delay_ns;
    int64_t mean_path_delay_ns;

    /* Statistics */
    uint32_t sync_messages_sent;
    uint32_t sync_messages_received;
    uint32_t delay_req_messages_sent;
    uint32_t delay_req_messages_received;
    uint32_t announce_messages_sent;
    uint32_t announce_messages_received;

    /* Hardware timestamping */
    bool hardware_ts_enabled;
    void (*hw_ts_callback)(uint64_t timestamp_ns);

    /* Network interface */
    int socket_fd;
    uint16_t event_port;
    uint16_t general_port;

    bool initialized;
} ptp_context_t;

/* PTP Message Header */
typedef struct __attribute__((packed)) {
    uint8_t transport_specific;
    uint8_t message_type;
    uint8_t reserved1;
    uint8_t version_ptp;
    uint16_t message_length;
    uint8_t domain_number;
    uint8_t reserved2;
    uint16_t flags;
    int64_t correction_field;
    uint32_t reserved3;
    ptp_clock_id_t source_clock_id;
    uint16_t source_port_id;
    uint16_t sequence_id;
    uint8_t control;
    uint8_t log_message_interval;
} ptp_header_t;

/* PTP Sync Message */
typedef struct __attribute__((packed)) {
    ptp_header_t header;
    ptp_timestamp_t origin_timestamp;
} ptp_sync_msg_t;

/* PTP Delay Request Message */
typedef struct __attribute__((packed)) {
    ptp_header_t header;
    ptp_timestamp_t origin_timestamp;
} ptp_delay_req_msg_t;

/* PTP Follow Up Message */
typedef struct __attribute__((packed)) {
    ptp_header_t header;
    ptp_timestamp_t precise_origin_timestamp;
} ptp_follow_up_msg_t;

/* PTP Delay Response Message */
typedef struct __attribute__((packed)) {
    ptp_header_t header;
    ptp_timestamp_t receive_timestamp;
    ptp_port_id_t requesting_port_id;
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
    ptp_clock_id_t grandmaster_clock_id;
    uint16_t steps_removed;
    uint8_t time_source;
} ptp_announce_msg_t;

/* Exported constants --------------------------------------------------------*/

/* PTP Protocol Constants */
#define PTP_VERSION                 2
#define PTP_DEFAULT_DOMAIN          0
#define PTP_DEFAULT_PRIORITY1       128
#define PTP_DEFAULT_PRIORITY2       128
#define PTP_EVENT_PORT              319
#define PTP_GENERAL_PORT            320
#define PTP_MULTICAST_ADDR          "224.0.1.129"

/* PTP Message Flags */
#define PTP_FLAG_TWO_STEP           (1 << 0)
#define PTP_FLAG_UNICAST            (1 << 1)
#define PTP_FLAG_PROFILE_SPECIFIC1  (1 << 5)
#define PTP_FLAG_PROFILE_SPECIFIC2  (1 << 6)

/* PTP Control Field Values */
#define PTP_CONTROL_SYNC            0x00
#define PTP_CONTROL_DELAY_REQ       0x01
#define PTP_CONTROL_FOLLOW_UP       0x02
#define PTP_CONTROL_DELAY_RESP      0x03
#define PTP_CONTROL_MANAGEMENT      0x04
#define PTP_CONTROL_OTHER           0x05
#define PTP_CONTROL_SIGNALING       0x06
#define PTP_CONTROL_RESERVED        0x07

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize PTP stack
 * @param  ctx: PTP context structure
 * @param  config: Configuration parameters
 * @retval 0 on success, negative on error
 */
int ptp_stack_init(ptp_context_t *ctx, ptp_config_t *config);

/**
 * @brief  Deinitialize PTP stack
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
int ptp_stack_deinit(ptp_context_t *ctx);

/**
 * @brief  Start PTP operation
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
int ptp_stack_start(ptp_context_t *ctx);

/**
 * @brief  Stop PTP operation
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
int ptp_stack_stop(ptp_context_t *ctx);

/**
 * @brief  Process incoming PTP message
 * @param  ctx: PTP context structure
 * @param  buffer: Message buffer
 * @param  length: Message length
 * @retval 0 on success, negative on error
 */
int ptp_stack_process_message(ptp_context_t *ctx, uint8_t *buffer, uint16_t length);

/**
 * @brief  Send PTP sync message
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
int ptp_stack_send_sync(ptp_context_t *ctx);

/**
 * @brief  Send PTP delay request message
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
int ptp_stack_send_delay_req(ptp_context_t *ctx);

/**
 * @brief  Send PTP announce message
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
int ptp_stack_send_announce(ptp_context_t *ctx);

/**
 * @brief  Get current PTP status
 * @param  ctx: PTP context structure
 * @param  state: Port state (output)
 * @param  offset_ns: Current offset in nanoseconds (output)
 * @param  delay_ns: Current path delay in nanoseconds (output)
 * @retval 0 on success, negative on error
 */
int ptp_stack_get_status(ptp_context_t *ctx, ptp_port_state_t *state,
                        int64_t *offset_ns, int64_t *delay_ns);

/**
 * @brief  Set local clock time
 * @param  ctx: PTP context structure
 * @param  timestamp: New timestamp
 * @retval 0 on success, negative on error
 */
int ptp_stack_set_time(ptp_context_t *ctx, ptp_timestamp_t *timestamp);

/**
 * @brief  Get local clock time
 * @param  ctx: PTP context structure
 * @param  timestamp: Current timestamp (output)
 * @retval 0 on success, negative on error
 */
int ptp_stack_get_time(ptp_context_t *ctx, ptp_timestamp_t *timestamp);

/**
 * @brief  Enable hardware timestamping
 * @param  ctx: PTP context structure
 * @param  callback: Hardware timestamp callback function
 * @retval 0 on success, negative on error
 */
int ptp_stack_enable_hardware_timestamping(ptp_context_t *ctx,
                                          void (*callback)(uint64_t timestamp_ns));

/**
 * @brief  Disable hardware timestamping
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
int ptp_stack_disable_hardware_timestamping(ptp_context_t *ctx);

/**
 * @brief  Get PTP statistics
 * @param  ctx: PTP context structure
 * @param  sync_sent: Sync messages sent (output)
 * @param  sync_received: Sync messages received (output)
 * @param  delay_req_sent: Delay request messages sent (output)
 * @param  delay_req_received: Delay request messages received (output)
 * @retval 0 on success, negative on error
 */
int ptp_stack_get_statistics(ptp_context_t *ctx, uint32_t *sync_sent,
                            uint32_t *sync_received, uint32_t *delay_req_sent,
                            uint32_t *delay_req_received);

/* Message creation functions ----------------------------------------------*/

/**
 * @brief  Create PTP sync message
 * @param  ctx: PTP context structure
 * @param  message: Message buffer (output)
 * @param  timestamp: Origin timestamp
 * @retval Message length on success, negative on error
 */
int ptp_create_sync_message(ptp_context_t *ctx, uint8_t *message, ptp_timestamp_t *timestamp);

/**
 * @brief  Create PTP delay request message
 * @param  ctx: PTP context structure
 * @param  message: Message buffer (output)
 * @param  timestamp: Origin timestamp
 * @retval Message length on success, negative on error
 */
int ptp_create_delay_req_message(ptp_context_t *ctx, uint8_t *message, ptp_timestamp_t *timestamp);

/**
 * @brief  Create PTP follow up message
 * @param  ctx: PTP context structure
 * @param  message: Message buffer (output)
 * @param  timestamp: Precise origin timestamp
 * @retval Message length on success, negative on error
 */
int ptp_create_follow_up_message(ptp_context_t *ctx, uint8_t *message, ptp_timestamp_t *timestamp);

/**
 * @brief  Create PTP delay response message
 * @param  ctx: PTP context structure
 * @param  message: Message buffer (output)
 * @param  timestamp: Receive timestamp
 * @param  requesting_port: Requesting port identity
 * @retval Message length on success, negative on error
 */
int ptp_create_delay_resp_message(ptp_context_t *ctx, uint8_t *message,
                                 ptp_timestamp_t *timestamp, ptp_port_id_t *requesting_port);

/**
 * @brief  Create PTP announce message
 * @param  ctx: PTP context structure
 * @param  message: Message buffer (output)
 * @retval Message length on success, negative on error
 */
int ptp_create_announce_message(ptp_context_t *ctx, uint8_t *message);

/* Utility functions --------------------------------------------------------*/

/**
 * @brief  Convert timestamp to nanoseconds
 * @param  timestamp: PTP timestamp
 * @retval Nanoseconds since epoch
 */
uint64_t ptp_timestamp_to_ns(ptp_timestamp_t *timestamp);

/**
 * @brief  Convert nanoseconds to PTP timestamp
 * @param  nanoseconds: Nanoseconds since epoch
 * @param  timestamp: PTP timestamp (output)
 * @retval None
 */
void ptp_ns_to_timestamp(uint64_t nanoseconds, ptp_timestamp_t *timestamp);

/**
 * @brief  Get current system time as PTP timestamp
 * @param  timestamp: PTP timestamp (output)
 * @retval None
 */
void ptp_get_current_timestamp(ptp_timestamp_t *timestamp);

/**
 * @brief  Calculate time difference between two timestamps
 * @param  t1: First timestamp
 * @param  t2: Second timestamp
 * @retval Time difference in nanoseconds (t2 - t1)
 */
int64_t ptp_calculate_time_difference(ptp_timestamp_t *t1, ptp_timestamp_t *t2);

/**
 * @brief  Add nanoseconds to timestamp
 * @param  timestamp: Input timestamp
 * @param  nanoseconds: Nanoseconds to add
 * @param  result: Result timestamp (output)
 * @retval None
 */
void ptp_add_nanoseconds(ptp_timestamp_t *timestamp, int64_t nanoseconds, ptp_timestamp_t *result);

/**
 * @brief  Compare two clock identities
 * @param  id1: First clock identity
 * @param  id2: Second clock identity
 * @retval 0 if equal, negative if id1 < id2, positive if id1 > id2
 */
int ptp_compare_clock_id(ptp_clock_id_t *id1, ptp_clock_id_t *id2);

/**
 * @brief  Validate PTP message checksum
 * @param  message: Message buffer
 * @param  length: Message length
 * @retval true if valid, false otherwise
 */
bool ptp_validate_message_checksum(uint8_t *message, uint16_t length);

/**
 * @brief  Calculate PTP message checksum
 * @param  message: Message buffer
 * @param  length: Message length
 * @retval Checksum value
 */
uint16_t ptp_calculate_message_checksum(uint8_t *message, uint16_t length);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Initialize network interface
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
static int ptp_init_network(ptp_context_t *ctx);

/**
 * @brief  Cleanup network interface
 * @param  ctx: PTP context structure
 * @retval 0 on success, negative on error
 */
static int ptp_cleanup_network(ptp_context_t *ctx);

/**
 * @brief  Send PTP message over network
 * @param  ctx: PTP context structure
 * @param  message: Message buffer
 * @param  length: Message length
 * @retval 0 on success, negative on error
 */
static int ptp_send_network_message(ptp_context_t *ctx, uint8_t *message, uint16_t length);

/**
 * @brief  Receive PTP message from network
 * @param  ctx: PTP context structure
 * @param  buffer: Receive buffer
 * @param  length: Buffer length (input/output)
 * @retval 0 on success, negative on error
 */
static int ptp_receive_network_message(ptp_context_t *ctx, uint8_t *buffer, uint16_t *length);

/**
 * @brief  Update PTP state machine
 * @param  ctx: PTP context structure
 * @retval None
 */
static void ptp_update_state_machine(ptp_context_t *ctx);

/**
 * @brief  Run Best Master Clock (BMC) algorithm
 * @param  ctx: PTP context structure
 * @param  announce_msg: Received announce message
 * @retval true if this clock should be master, false otherwise
 */
static bool ptp_run_bmc_algorithm(ptp_context_t *ctx, ptp_announce_msg_t *announce_msg);

/**
 * @brief  Handle hardware timestamp
 * @param  ctx: PTP context structure
 * @param  timestamp_ns: Hardware timestamp in nanoseconds
 * @retval None
 */
static void ptp_handle_hardware_timestamp(ptp_context_t *ctx, uint64_t timestamp_ns);

#ifdef __cplusplus
}
#endif

#endif /* __PTP_STACK_H */
