/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ptp_stack.c
  * @brief          : PTP (IEEE 1588) protocol stack implementation
  * @author         : WiWi Development Team
  * @version        : 1.0.0
  * @date           : January 2025
  ******************************************************************************
  * @attention
  *
  * PTP Protocol Stack Implementation for RANDOM_SYNC_2017MHZ_V2
  * Supports G.8275.1 Telecom Profile
  * Hardware timestamping support for precision timing
  *
  * Based on IEEE 1588-2008 standard
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ptp_stack.h"
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/* Private defines -----------------------------------------------------------*/

/* Network constants */
#define PTP_MULTICAST_GROUP      "224.0.1.129"
#define PTP_EVENT_PORT          319
#define PTP_GENERAL_PORT        320

/* PTP timing constants */
#define PTP_SYNC_INTERVAL       1  // seconds
#define PTP_ANNOUNCE_INTERVAL   2  // seconds
#define PTP_DELAY_REQ_INTERVAL  1  // seconds

/* Private variables ---------------------------------------------------------*/
static int ptp_socket_fd = -1;
static struct sockaddr_in ptp_multicast_addr;
static uint16_t ptp_sequence_id = 0;

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Initialize PTP stack
  * @param  ctx: PTP context structure
  * @param  config: Configuration parameters
  * @retval 0 on success, negative on error
  */
int ptp_stack_init(ptp_context_t *ctx, ptp_config_t *config)
{
  int ret = 0;

  if (ctx == NULL || config == NULL) {
    return -1;
  }

  /* Initialize context */
  memset(ctx, 0, sizeof(ptp_context_t));

  /* Copy configuration */
  memcpy(&ctx->config, config, sizeof(ptp_config_t));

  /* Initialize network */
  ret = ptp_init_network(ctx);
  if (ret < 0) {
    return ret;
  }

  /* Set initial state */
  ctx->port_state = PTP_PORT_STATE_INITIALIZING;
  ctx->initialized = true;

  return 0;
}

/**
  * @brief  Deinitialize PTP stack
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
int ptp_stack_deinit(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return -1;
  }

  /* Cleanup network */
  ptp_cleanup_network(ctx);

  ctx->initialized = false;
  return 0;
}

/**
  * @brief  Start PTP operation
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
int ptp_stack_start(ptp_context_t *ctx)
{
  if (ctx == NULL || !ctx->initialized) {
    return -1;
  }

  /* Set initial state based on clock type */
  switch (ctx->config.clock_type) {
    case PTP_CLOCK_TYPE_BOUNDARY:
      ctx->port_state = PTP_PORT_STATE_MASTER;
      break;
    case PTP_CLOCK_TYPE_ORDINARY:
    default:
      ctx->port_state = PTP_PORT_STATE_SLAVE;
      break;
  }

  return 0;
}

/**
  * @brief  Stop PTP operation
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
int ptp_stack_stop(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return -1;
  }

  ctx->port_state = PTP_PORT_STATE_DISABLED;
  return 0;
}

/**
  * @brief  Process incoming PTP message
  * @param  ctx: PTP context structure
  * @param  buffer: Message buffer
  * @param  length: Message length
  * @retval 0 on success, negative on error
  */
int ptp_stack_process_message(ptp_context_t *ctx, uint8_t *buffer, uint16_t length)
{
  if (ctx == NULL || buffer == NULL || length < sizeof(ptp_header_t)) {
    return -1;
  }

  ptp_header_t *header = (ptp_header_t *)buffer;

  /* Validate PTP version */
  if (header->version_ptp != PTP_VERSION) {
    return -1;
  }

  /* Validate domain */
  if (header->domain_number != ctx->config.domain_number) {
    return 0;  // Ignore messages from different domain
  }

  /* Process message based on type */
  switch (header->message_type) {
    case PTP_MSG_TYPE_SYNC:
      ctx->sync_messages_received++;
      /* Process SYNC message */
      break;

    case PTP_MSG_TYPE_DELAY_REQ:
      ctx->delay_req_messages_received++;
      /* Process DELAY_REQ and send DELAY_RESP */
      break;

    case PTP_MSG_TYPE_FOLLOW_UP:
      /* Process FOLLOW_UP message */
      break;

    case PTP_MSG_TYPE_DELAY_RESP:
      ctx->delay_req_messages_received++;
      /* Process DELAY_RESP message */
      break;

    case PTP_MSG_TYPE_ANNOUNCE:
      ctx->announce_messages_received++;
      /* Process ANNOUNCE message */
      break;

    default:
      return -1;  // Unknown message type
  }

  return 0;
}

/**
  * @brief  Send PTP sync message
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
int ptp_stack_send_sync(ptp_context_t *ctx)
{
  if (ctx == NULL || !ctx->initialized) {
    return -1;
  }

  uint8_t buffer[sizeof(ptp_sync_msg_t)];
  ptp_timestamp_t timestamp;

  /* Create sync message */
  int ret = ptp_create_sync_message(ctx, buffer, &timestamp);
  if (ret < 0) {
    return ret;
  }

  /* Send message */
  ret = ptp_send_network_message(ctx, buffer, ret);
  if (ret < 0) {
    return ret;
  }

  ctx->sync_messages_sent++;
  return 0;
}

/**
  * @brief  Send PTP delay request message
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
int ptp_stack_send_delay_req(ptp_context_t *ctx)
{
  if (ctx == NULL || !ctx->initialized) {
    return -1;
  }

  uint8_t buffer[sizeof(ptp_delay_req_msg_t)];
  ptp_timestamp_t timestamp;

  /* Create delay request message */
  int ret = ptp_create_delay_req_message(ctx, buffer, &timestamp);
  if (ret < 0) {
    return ret;
  }

  /* Send message */
  ret = ptp_send_network_message(ctx, buffer, ret);
  if (ret < 0) {
    return ret;
  }

  ctx->delay_req_messages_sent++;
  return 0;
}

/**
  * @brief  Send PTP announce message
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
int ptp_stack_send_announce(ptp_context_t *ctx)
{
  if (ctx == NULL || !ctx->initialized) {
    return -1;
  }

  uint8_t buffer[sizeof(ptp_announce_msg_t)];

  /* Create announce message */
  int ret = ptp_create_announce_message(ctx, buffer);
  if (ret < 0) {
    return ret;
  }

  /* Send message */
  ret = ptp_send_network_message(ctx, buffer, ret);
  if (ret < 0) {
    return ret;
  }

  ctx->announce_messages_sent++;
  return 0;
}

/**
  * @brief  Get current PTP status
  * @param  ctx: PTP context structure
  * @param  state: Port state (output)
  * @param  offset_ns: Current offset in nanoseconds (output)
  * @param  delay_ns: Current path delay in nanoseconds (output)
  * @retval 0 on success, negative on error
  */
int ptp_stack_get_status(ptp_context_t *ctx, ptp_port_state_t *state,
                        int64_t *offset_ns, int64_t *delay_ns)
{
  if (ctx == NULL || state == NULL || offset_ns == NULL || delay_ns == NULL) {
    return -1;
  }

  *state = ctx->port_state;
  *offset_ns = ctx->master_offset_ns;
  *delay_ns = ctx->path_delay_ns;

  return 0;
}

/**
  * @brief  Set local clock time
  * @param  ctx: PTP context structure
  * @param  timestamp: New timestamp
  * @retval 0 on success, negative on error
  */
int ptp_stack_set_time(ptp_context_t *ctx, ptp_timestamp_t *timestamp)
{
  if (ctx == NULL || timestamp == NULL) {
    return -1;
  }

  /* Set system time (simplified implementation) */
  /* In real implementation, this would set the system clock */

  return 0;
}

/**
  * @brief  Get local clock time
  * @param  ctx: PTP context structure
  * @param  timestamp: Current timestamp (output)
  * @retval 0 on success, negative on error
  */
int ptp_stack_get_time(ptp_context_t *ctx, ptp_timestamp_t *timestamp)
{
  if (ctx == NULL || timestamp == NULL) {
    return -1;
  }

  /* Get current system time */
  ptp_get_current_timestamp(timestamp);

  return 0;
}

/**
  * @brief  Enable hardware timestamping
  * @param  ctx: PTP context structure
  * @param  callback: Hardware timestamp callback function
  * @retval 0 on success, negative on error
  */
int ptp_stack_enable_hardware_timestamping(ptp_context_t *ctx,
                                          void (*callback)(uint64_t timestamp_ns))
{
  if (ctx == NULL) {
    return -1;
  }

  ctx->hardware_ts_enabled = true;
  ctx->hw_ts_callback = callback;

  return 0;
}

/**
  * @brief  Disable hardware timestamping
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
int ptp_stack_disable_hardware_timestamping(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return -1;
  }

  ctx->hardware_ts_enabled = false;
  ctx->hw_ts_callback = NULL;

  return 0;
}

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
                            uint32_t *delay_req_received)
{
  if (ctx == NULL || sync_sent == NULL || sync_received == NULL ||
      delay_req_sent == NULL || delay_req_received == NULL) {
    return -1;
  }

  *sync_sent = ctx->sync_messages_sent;
  *sync_received = ctx->sync_messages_received;
  *delay_req_sent = ctx->delay_req_messages_sent;
  *delay_req_received = ctx->delay_req_messages_received;

  return 0;
}

/**
  * @brief  Create PTP sync message
  * @param  ctx: PTP context structure
  * @param  message: Message buffer (output)
  * @param  timestamp: Origin timestamp
  * @retval Message length on success, negative on error
  */
int ptp_create_sync_message(ptp_context_t *ctx, uint8_t *message, ptp_timestamp_t *timestamp)
{
  if (ctx == NULL || message == NULL || timestamp == NULL) {
    return -1;
  }

  ptp_sync_msg_t *sync_msg = (ptp_sync_msg_t *)message;

  /* Create header */
  ptp_header_t header;
  int ret = ptp_create_header(ctx, &header, PTP_MSG_TYPE_SYNC, sizeof(ptp_sync_msg_t), 0);
  if (ret < 0) {
    return ret;
  }

  memcpy(&sync_msg->header, &header, sizeof(ptp_header_t));
  memcpy(&sync_msg->origin_timestamp, timestamp, sizeof(ptp_timestamp_t));

  return sizeof(ptp_sync_msg_t);
}

/**
  * @brief  Create PTP delay request message
  * @param  ctx: PTP context structure
  * @param  message: Message buffer (output)
  * @param  timestamp: Origin timestamp
  * @retval Message length on success, negative on error
  */
int ptp_create_delay_req_message(ptp_context_t *ctx, uint8_t *message, ptp_timestamp_t *timestamp)
{
  if (ctx == NULL || message == NULL || timestamp == NULL) {
    return -1;
  }

  ptp_delay_req_msg_t *delay_req_msg = (ptp_delay_req_msg_t *)message;

  /* Create header */
  ptp_header_t header;
  int ret = ptp_create_header(ctx, &header, PTP_MSG_TYPE_DELAY_REQ, sizeof(ptp_delay_req_msg_t), 0);
  if (ret < 0) {
    return ret;
  }

  memcpy(&delay_req_msg->header, &header, sizeof(ptp_header_t));
  memcpy(&delay_req_msg->origin_timestamp, timestamp, sizeof(ptp_timestamp_t));

  return sizeof(ptp_delay_req_msg_t);
}

/**
  * @brief  Create PTP follow up message
  * @param  ctx: PTP context structure
  * @param  message: Message buffer (output)
  * @param  timestamp: Precise origin timestamp
  * @retval Message length on success, negative on error
  */
int ptp_create_follow_up_message(ptp_context_t *ctx, uint8_t *message, ptp_timestamp_t *timestamp)
{
  if (ctx == NULL || message == NULL || timestamp == NULL) {
    return -1;
  }

  ptp_follow_up_msg_t *follow_up_msg = (ptp_follow_up_msg_t *)message;

  /* Create header */
  ptp_header_t header;
  int ret = ptp_create_header(ctx, &header, PTP_MSG_TYPE_FOLLOW_UP, sizeof(ptp_follow_up_msg_t), 0);
  if (ret < 0) {
    return ret;
  }

  memcpy(&follow_up_msg->header, &header, sizeof(ptp_header_t));
  memcpy(&follow_up_msg->precise_origin_timestamp, timestamp, sizeof(ptp_timestamp_t));

  return sizeof(ptp_follow_up_msg_t);
}

/**
  * @brief  Create PTP delay response message
  * @param  ctx: PTP context structure
  * @param  message: Message buffer (output)
  * @param  timestamp: Receive timestamp
  * @param  requesting_port: Requesting port identity
  * @retval Message length on success, negative on error
  */
int ptp_create_delay_resp_message(ptp_context_t *ctx, uint8_t *message,
                                 ptp_timestamp_t *timestamp, ptp_port_id_t *requesting_port)
{
  if (ctx == NULL || message == NULL || timestamp == NULL || requesting_port == NULL) {
    return -1;
  }

  ptp_delay_resp_msg_t *delay_resp_msg = (ptp_delay_resp_msg_t *)message;

  /* Create header */
  ptp_header_t header;
  int ret = ptp_create_header(ctx, &header, PTP_MSG_TYPE_DELAY_RESP, sizeof(ptp_delay_resp_msg_t), 0);
  if (ret < 0) {
    return ret;
  }

  memcpy(&delay_resp_msg->header, &header, sizeof(ptp_header_t));
  memcpy(&delay_resp_msg->receive_timestamp, timestamp, sizeof(ptp_timestamp_t));
  memcpy(&delay_resp_msg->requesting_port_id, requesting_port, sizeof(ptp_port_id_t));

  return sizeof(ptp_delay_resp_msg_t);
}

/**
  * @brief  Create PTP announce message
  * @param  ctx: PTP context structure
  * @param  message: Message buffer (output)
  * @retval Message length on success, negative on error
  */
int ptp_create_announce_message(ptp_context_t *ctx, uint8_t *message)
{
  if (ctx == NULL || message == NULL) {
    return -1;
  }

  ptp_announce_msg_t *announce_msg = (ptp_announce_msg_t *)message;

  /* Create header */
  ptp_header_t header;
  int ret = ptp_create_header(ctx, &header, PTP_MSG_TYPE_ANNOUNCE, sizeof(ptp_announce_msg_t), 5);
  if (ret < 0) {
    return ret;
  }

  memcpy(&announce_msg->header, &header, sizeof(ptp_header_t));

  /* Set announce specific fields */
  announce_msg->current_utc_offset = 0;
  announce_msg->grandmaster_priority1 = ctx->config.priority1;
  announce_msg->grandmaster_clock_quality_class = ctx->config.clock_type == PTP_CLOCK_TYPE_BOUNDARY ?
    PTP_CLASS_LOCKED : PTP_CLASS_PRIMARY;
  announce_msg->grandmaster_clock_quality_accuracy = PTP_ACCURACY_100NS;
  announce_msg->grandmaster_clock_quality_offset_scaled_log_variance = 0x4E5D;
  announce_msg->grandmaster_priority2 = ctx->config.priority2;
  memcpy(announce_msg->grandmaster_clock_id.id, ctx->current_dataset.clock_id.id, 8);
  announce_msg->steps_removed = 0;
  announce_msg->time_source = ctx->config.clock_type == PTP_CLOCK_TYPE_BOUNDARY ?
    PTP_SOURCE_PTP : PTP_SOURCE_GNSS;

  return sizeof(ptp_announce_msg_t);
}

/**
  * @brief  Convert timestamp to nanoseconds
  * @param  timestamp: PTP timestamp
  * @retval Nanoseconds since epoch
  */
uint64_t ptp_timestamp_to_ns(ptp_timestamp_t *timestamp)
{
  if (timestamp == NULL) {
    return 0;
  }

  return timestamp->seconds * 1000000000ULL + timestamp->nanoseconds;
}

/**
  * @brief  Convert nanoseconds to PTP timestamp
  * @param  nanoseconds: Nanoseconds since epoch
  * @param  timestamp: PTP timestamp (output)
  * @retval None
  */
void ptp_ns_to_timestamp(uint64_t nanoseconds, ptp_timestamp_t *timestamp)
{
  if (timestamp == NULL) {
    return;
  }

  timestamp->seconds = nanoseconds / 1000000000ULL;
  timestamp->nanoseconds = nanoseconds % 1000000000ULL;
}

/**
  * @brief  Get current system time as PTP timestamp
  * @param  timestamp: PTP timestamp (output)
  * @retval None
  */
void ptp_get_current_timestamp(ptp_timestamp_t *timestamp)
{
  if (timestamp == NULL) {
    return;
  }

  /* Get current system time */
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);

  timestamp->seconds = ts.tv_sec;
  timestamp->nanoseconds = ts.tv_nsec;
}

/**
  * @brief  Calculate time difference between two timestamps
  * @param  t1: First timestamp
  * @param  t2: Second timestamp
  * @retval Time difference in nanoseconds (t2 - t1)
  */
int64_t ptp_calculate_time_difference(ptp_timestamp_t *t1, ptp_timestamp_t *t2)
{
  if (t1 == NULL || t2 == NULL) {
    return 0;
  }

  uint64_t ns1 = ptp_timestamp_to_ns(t1);
  uint64_t ns2 = ptp_timestamp_to_ns(t2);

  return (int64_t)(ns2 - ns1);
}

/**
  * @brief  Add nanoseconds to timestamp
  * @param  timestamp: Input timestamp
  * @param  nanoseconds: Nanoseconds to add
  * @param  result: Result timestamp (output)
  * @retval None
  */
void ptp_add_nanoseconds(ptp_timestamp_t *timestamp, int64_t nanoseconds, ptp_timestamp_t *result)
{
  if (timestamp == NULL || result == NULL) {
    return;
  }

  uint64_t total_ns = ptp_timestamp_to_ns(timestamp) + nanoseconds;

  result->seconds = total_ns / 1000000000ULL;
  result->nanoseconds = total_ns % 1000000000ULL;
}

/**
  * @brief  Compare two clock identities
  * @param  id1: First clock identity
  * @param  id2: Second clock identity
  * @retval 0 if equal, negative if id1 < id2, positive if id1 > id2
  */
int ptp_compare_clock_id(ptp_clock_id_t *id1, ptp_clock_id_t *id2)
{
  if (id1 == NULL || id2 == NULL) {
    return 0;
  }

  return memcmp(id1->id, id2->id, 8);
}

/**
  * @brief  Validate PTP message checksum
  * @param  message: Message buffer
  * @param  length: Message length
  * @retval true if valid, false otherwise
  */
bool ptp_validate_message_checksum(uint8_t *message, uint16_t length)
{
  if (message == NULL || length < 4) {
    return false;
  }

  /* Find checksum in message */
  uint16_t checksum = 0;
  for (uint16_t i = 0; i < length - 2; i++) {
    checksum += message[i];
  }

  /* Extract checksum from message */
  uint16_t message_checksum = (message[length - 2] << 8) | message[length - 1];

  return (checksum == message_checksum);
}

/**
  * @brief  Calculate PTP message checksum
  * @param  message: Message buffer
  * @param  length: Message length
  * @retval Checksum value
  */
uint16_t ptp_calculate_message_checksum(uint8_t *message, uint16_t length)
{
  if (message == NULL || length == 0) {
    return 0;
  }

  uint16_t checksum = 0;
  for (uint16_t i = 0; i < length; i++) {
    checksum += message[i];
  }

  return checksum;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize network interface
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
static int ptp_init_network(ptp_context_t *ctx)
{
  /* Create socket for event messages */
  ptp_socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (ptp_socket_fd < 0) {
    return -1;
  }

  /* Set socket options */
  int opt = 1;
  setsockopt(ptp_socket_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  /* Bind to event port */
  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(PTP_EVENT_PORT);

  if (bind(ptp_socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(ptp_socket_fd);
    return -1;
  }

  /* Join multicast group */
  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(PTP_MULTICAST_GROUP);
  mreq.imr_interface.s_addr = INADDR_ANY;

  if (setsockopt(ptp_socket_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
    close(ptp_socket_fd);
    return -1;
  }

  /* Setup multicast address */
  memset(&ptp_multicast_addr, 0, sizeof(ptp_multicast_addr));
  ptp_multicast_addr.sin_family = AF_INET;
  ptp_multicast_addr.sin_addr.s_addr = inet_addr(PTP_MULTICAST_GROUP);
  ptp_multicast_addr.sin_port = htons(PTP_EVENT_PORT);

  ctx->socket_fd = ptp_socket_fd;
  ctx->event_port = PTP_EVENT_PORT;
  ctx->general_port = PTP_GENERAL_PORT;

  return 0;
}

/**
  * @brief  Cleanup network interface
  * @param  ctx: PTP context structure
  * @retval 0 on success, negative on error
  */
static int ptp_cleanup_network(ptp_context_t *ctx)
{
  if (ptp_socket_fd >= 0) {
    close(ptp_socket_fd);
    ptp_socket_fd = -1;
  }

  return 0;
}

/**
  * @brief  Send PTP message over network
  * @param  ctx: PTP context structure
  * @param  message: Message buffer
  * @param  length: Message length
  * @retval 0 on success, negative on error
  */
static int ptp_send_network_message(ptp_context_t *ctx, uint8_t *message, uint16_t length)
{
  if (ptp_socket_fd < 0 || message == NULL || length == 0) {
    return -1;
  }

  /* Send message to multicast group */
  ssize_t sent = sendto(ptp_socket_fd, message, length, 0,
                       (struct sockaddr *)&ptp_multicast_addr, sizeof(ptp_multicast_addr));

  if (sent < 0) {
    return -1;
  }

  return 0;
}

/**
  * @brief  Receive PTP message from network
  * @param  ctx: PTP context structure
  * @param  buffer: Receive buffer
  * @param  length: Buffer length (input/output)
  * @retval 0 on success, negative on error
  */
static int ptp_receive_network_message(ptp_context_t *ctx, uint8_t *buffer, uint16_t *length)
{
  if (ptp_socket_fd < 0 || buffer == NULL || length == NULL) {
    return -1;
  }

  struct sockaddr_in src_addr;
  socklen_t addr_len = sizeof(src_addr);

  /* Receive message */
  ssize_t received = recvfrom(ptp_socket_fd, buffer, *length, 0,
                             (struct sockaddr *)&src_addr, &addr_len);

  if (received < 0) {
    return -1;
  }

  *length = (uint16_t)received;
  return 0;
}

/**
  * @brief  Update PTP state machine
  * @param  ctx: PTP context structure
  * @retval None
  */
static void ptp_update_state_machine(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return;
  }

  /* Simplified state machine for phase 1 */
  switch (ctx->port_state) {
    case PTP_PORT_STATE_INITIALIZING:
      ctx->port_state = PTP_PORT_STATE_LISTENING;
      break;

    case PTP_PORT_STATE_LISTENING:
      /* Check if we should become master */
      if (ctx->config.clock_type == PTP_CLOCK_TYPE_BOUNDARY) {
        ctx->port_state = PTP_PORT_STATE_MASTER;
      }
      break;

    case PTP_PORT_STATE_MASTER:
      /* Stay as master */
      break;

    case PTP_PORT_STATE_SLAVE:
      /* Stay as slave */
      break;

    default:
      break;
  }
}

/**
  * @brief  Run Best Master Clock (BMC) algorithm
  * @param  ctx: PTP context structure
  * @param  announce_msg: Received announce message
  * @retval true if this clock should be master, false otherwise
  */
static bool ptp_run_bmc_algorithm(ptp_context_t *ctx, ptp_announce_msg_t *announce_msg)
{
  if (ctx == NULL || announce_msg == NULL) {
    return false;
  }

  /* Simplified BMC algorithm for phase 1 */
  /* Compare priority1 */
  if (announce_msg->grandmaster_priority1 < ctx->config.priority1) {
    return false;  // Remote is better
  } else if (announce_msg->grandmaster_priority1 > ctx->config.priority1) {
    return true;   // We are better
  }

  /* Compare priority2 */
  if (announce_msg->grandmaster_priority2 < ctx->config.priority2) {
    return false;  // Remote is better
  } else if (announce_msg->grandmaster_priority2 > ctx->config.priority2) {
    return true;   // We are better
  }

  /* Compare clock class */
  if (announce_msg->grandmaster_clock_quality_class < ctx->config.clock_type) {
    return false;  // Remote is better
  }

  return true;  // We are better or equal
}

/**
  * @brief  Handle hardware timestamp
  * @param  ctx: PTP context structure
  * @param  timestamp_ns: Hardware timestamp in nanoseconds
  * @retval None
  */
static void ptp_handle_hardware_timestamp(ptp_context_t *ctx, uint64_t timestamp_ns)
{
  if (ctx == NULL || !ctx->hardware_ts_enabled || ctx->hw_ts_callback == NULL) {
    return;
  }

  /* Call user callback */
  ctx->hw_ts_callback(timestamp_ns);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
