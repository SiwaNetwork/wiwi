/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : ptp.c
  * @brief          : PTP (IEEE 1588) protocol implementation for RANDOM_SYNC_2017MHZ_V2
  * @author         : WiWi Development Team
  * @version        : 1.0.0
  * @date           : January 2025
  ******************************************************************************
  * @attention
  *
  * PTP Protocol Implementation for STM32H7
  * Supports G.8275.1 Telecom Profile
  * Hardware timestamping support
  *
  * Based on IEEE 1588-2008 standard
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ptp.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/

/* PTP Protocol Constants */
#define PTP_VERSION_PTP             2
#define PTP_VERSION_IEEE1588        2
#define PTP_DEFAULT_DOMAIN          0
#define PTP_DEFAULT_PRIORITY1       128
#define PTP_DEFAULT_PRIORITY2       128

/* PTP Message Sizes */
#define PTP_HEADER_SIZE             sizeof(ptp_header_t)
#define PTP_SYNC_SIZE               sizeof(ptp_sync_msg_t)
#define PTP_DELAY_REQ_SIZE          sizeof(ptp_delay_req_msg_t)
#define PTP_DELAY_RESP_SIZE         sizeof(ptp_delay_resp_msg_t)
#define PTP_ANNOUNCE_SIZE           sizeof(ptp_announce_msg_t)

/* PTP Timing Constants */
#define PTP_SYNC_INTERVAL_DEFAULT   0  // 1 second (2^0)
#define PTP_ANNOUNCE_INTERVAL       1  // 2 seconds (2^1)
#define PTP_DELAY_REQ_INTERVAL      0  // 1 second (2^0)

/* Private variables ---------------------------------------------------------*/
static ptp_context_t *ptp_ctx_global;
static uint16_t sequence_id = 0;

/* PTP Clock Identity */
static uint8_t ptp_clock_identity[8] = {
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77
};

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef ptp_create_header(ptp_header_t *header, uint8_t message_type,
                                         uint16_t message_length, uint8_t control);
static HAL_StatusTypeDef ptp_send_message(ptp_context_t *ctx, uint8_t *message, uint16_t length);
static HAL_StatusTypeDef ptp_receive_message(ptp_context_t *ctx, uint8_t *buffer, uint16_t *length);
static void ptp_update_state_machine(ptp_context_t *ctx);
static int64_t ptp_calculate_clock_offset(ptp_context_t *ctx, ptp_sync_msg_t *sync_msg,
                                         ptp_follow_up_msg_t *follow_up_msg);
static int64_t ptp_calculate_path_delay(ptp_context_t *ctx, ptp_delay_req_msg_t *delay_req,
                                       ptp_delay_resp_msg_t *delay_resp);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Initialize PTP subsystem
  * @param  ctx: PTP context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_init(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Initialize context */
  memset(ctx, 0, sizeof(ptp_context_t));
  ptp_ctx_global = ctx;

  /* Set default values */
  ctx->state = PTP_STATE_INITIALIZING;
  ctx->clock_class = PTP_CLASS_LOCKED;
  ctx->clock_accuracy = PTP_ACCURACY_100NS;
  ctx->clock_source = PTP_SOURCE_GNSS;
  ctx->profile = PTP_PROFILE_G8275_1;

  ctx->domain_number = PTP_DEFAULT_DOMAIN;
  ctx->priority1 = PTP_DEFAULT_PRIORITY1;
  ctx->priority2 = PTP_DEFAULT_PRIORITY2;
  memcpy(ctx->clock_identity, ptp_clock_identity, 8);

  ctx->hardware_timestamping = false;
  ctx->sync_interval = PTP_SYNC_INTERVAL_DEFAULT;

  /* Initialize statistics */
  ctx->sync_count = 0;
  ctx->delay_req_count = 0;
  ctx->announce_count = 0;
  ctx->last_sync_time = 0;
  ctx->last_delay_req_time = 0;

  return HAL_OK;
}

/**
  * @brief  Configure PTP parameters
  * @param  ctx: PTP context structure
  * @param  config: Configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_configure(ptp_context_t *ctx, ptp_config_t *config)
{
  if (ctx == NULL || config == NULL) {
    return HAL_ERROR;
  }

  /* Update configuration */
  ctx->profile = config->profile;
  ctx->domain_number = config->domain_number;
  ctx->priority1 = config->priority1;
  ctx->priority2 = config->priority2;
  ctx->clock_class = config->clock_class;
  ctx->clock_accuracy = config->clock_accuracy;
  ctx->clock_source = config->clock_source;
  ctx->sync_interval = config->sync_interval;
  ctx->hardware_timestamping = config->hardware_timestamping;

  return HAL_OK;
}

/**
  * @brief  Set PTP clock source
  * @param  ctx: PTP context structure
  * @param  source: Clock source
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_set_clock_source(ptp_context_t *ctx, ptp_clock_source_t source)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  ctx->clock_source = source;

  /* Update clock class based on source */
  switch (source) {
    case PTP_SOURCE_GNSS:
      ctx->clock_class = PTP_CLASS_LOCKED;
      ctx->clock_accuracy = PTP_ACCURACY_100NS;
      break;
    case PTP_SOURCE_ATOMIC:
      ctx->clock_class = PTP_CLASS_LOCKED;
      ctx->clock_accuracy = PTP_ACCURACY_25NS;
      break;
    case PTP_SOURCE_PTP:
      ctx->clock_class = PTP_CLASS_LOCKED;
      ctx->clock_accuracy = PTP_ACCURACY_100NS;
      break;
    default:
      ctx->clock_class = PTP_CLASS_FREE_RUNNING;
      ctx->clock_accuracy = PTP_ACCURACY_1US;
      break;
  }

  return HAL_OK;
}

/**
  * @brief  Enable hardware timestamping
  * @param  ctx: PTP context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_enable_hardware_timestamping(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  ctx->hardware_timestamping = true;
  return HAL_OK;
}

/**
  * @brief  Disable hardware timestamping
  * @param  ctx: PTP context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_disable_hardware_timestamping(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  ctx->hardware_timestamping = false;
  return HAL_OK;
}

/**
  * @brief  Get PTP status
  * @param  ctx: PTP context structure
  * @param  status: Status information
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_get_status(ptp_context_t *ctx, ptp_status_t *status)
{
  if (ctx == NULL || status == NULL) {
    return HAL_ERROR;
  }

  status->state = ctx->state;
  status->clock_class = ctx->clock_class;
  status->clock_accuracy = ctx->clock_accuracy;
  status->offset_from_master = ctx->master_offset;
  status->mean_path_delay = ctx->path_delay;
  status->steps_removed = ctx->steps_removed;
  status->last_sync_timestamp = ctx->last_sync_time;
  status->sync_count = ctx->sync_count;
  status->delay_req_count = ctx->delay_req_count;

  return HAL_OK;
}

/**
  * @brief  Process received PTP message
  * @param  ctx: PTP context structure
  * @param  message: PTP message buffer
  * @param  length: Message length
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_process_message(ptp_context_t *ctx, uint8_t *message, uint16_t length)
{
  if (ctx == NULL || message == NULL || length < PTP_HEADER_SIZE) {
    return HAL_ERROR;
  }

  ptp_header_t *header = (ptp_header_t *)message;

  /* Validate PTP version */
  if (header->version_ptp != PTP_VERSION) {
    return HAL_ERROR;
  }

  /* Validate domain */
  if (header->domain_number != ctx->domain_number) {
    return HAL_OK;  // Ignore messages from different domain
  }

  /* Process message based on type */
  switch (header->message_type) {
    case PTP_MSG_SYNC:
      /* Process SYNC message */
      if (length >= PTP_SYNC_SIZE) {
        ptp_sync_msg_t *sync_msg = (ptp_sync_msg_t *)message;
        // Store sync timestamp for later use with FOLLOW_UP
        ctx->last_sync_time = HAL_GetTick();
        ctx->sync_count++;
      }
      break;

    case PTP_MSG_DELAY_REQ:
      /* Process DELAY_REQ message (from slave) */
      if (length >= PTP_DELAY_REQ_SIZE) {
        ptp_delay_req_msg_t *delay_req = (ptp_delay_req_msg_t *)message;
        // Send DELAY_RESP message
        ptp_send_delay_resp(ctx, delay_req);
      }
      break;

    case PTP_MSG_FOLLOW_UP:
      /* Process FOLLOW_UP message */
      if (length >= sizeof(ptp_header_t) + 8) {  // Header + timestamp
        // Calculate offset using stored SYNC timestamp
        uint64_t t2 = ctx->last_sync_time;  // When we received SYNC
        uint64_t t1 = *(uint64_t*)(message + PTP_HEADER_SIZE);  // Precise origin time

        // For now, use simplified calculation
        ctx->master_offset = (int64_t)(t2 - t1);
      }
      break;

    case PTP_MSG_DELAY_RESP:
      /* Process DELAY_RESP message */
      if (length >= PTP_DELAY_RESP_SIZE) {
        ptp_delay_resp_msg_t *delay_resp = (ptp_delay_resp_msg_t *)message;
        // Calculate path delay
        ctx->path_delay = 0;  // Simplified for phase 1
        ctx->delay_req_count++;
      }
      break;

    case PTP_MSG_ANNOUNCE:
      /* Process ANNOUNCE message */
      if (length >= PTP_ANNOUNCE_SIZE) {
        ptp_announce_msg_t *announce = (ptp_announce_msg_t *)message;

        /* Check if this master is better than current */
        if (ptp_is_better_master(ctx, announce)) {
          ctx->steps_removed = announce->steps_removed + 1;
          memcpy(ctx->grandmaster_identity, announce->grandmaster_clock_identity, 8);
          ctx->state = PTP_STATE_SLAVE;
        }

        ctx->announce_count++;
      }
      break;

    default:
      /* Unknown message type */
      break;
  }

  /* Update state machine */
  ptp_update_state_machine(ctx);

  return HAL_OK;
}

/**
  * @brief  Send PTP sync message
  * @param  ctx: PTP context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_send_sync(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  ptp_sync_msg_t sync_msg;
  memset(&sync_msg, 0, sizeof(sync_msg));

  /* Create header */
  ptp_create_header(&sync_msg.header, PTP_MSG_SYNC, PTP_SYNC_SIZE, 0);

  /* Set origin timestamp (current time) */
  uint64_t current_time = HAL_GetTick() * 1000000ULL;  // Convert to nanoseconds
  sync_msg.origin_timestamp_seconds = current_time / 1000000000ULL;
  sync_msg.origin_timestamp_nanoseconds = current_time % 1000000000ULL;

  /* Send message */
  return ptp_send_message(ctx, (uint8_t *)&sync_msg, PTP_SYNC_SIZE);
}

/**
  * @brief  Send PTP delay request
  * @param  ctx: PTP context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_send_delay_req(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  ptp_delay_req_msg_t delay_req_msg;
  memset(&delay_req_msg, 0, sizeof(delay_req_msg));

  /* Create header */
  ptp_create_header(&delay_req_msg.header, PTP_MSG_DELAY_REQ, PTP_DELAY_REQ_SIZE, 0);

  /* Set origin timestamp (current time) */
  uint64_t current_time = HAL_GetTick() * 1000000ULL;  // Convert to nanoseconds
  delay_req_msg.origin_timestamp_seconds = current_time / 1000000000ULL;
  delay_req_msg.origin_timestamp_nanoseconds = current_time % 1000000000ULL;

  /* Send message */
  HAL_StatusTypeDef ret = ptp_send_message(ctx, (uint8_t *)&delay_req_msg, PTP_DELAY_REQ_SIZE);
  if (ret == HAL_OK) {
    ctx->last_delay_req_time = HAL_GetTick();
  }

  return ret;
}

/**
  * @brief  Send PTP announce message
  * @param  ctx: PTP context structure
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_send_announce(ptp_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  ptp_announce_msg_t announce_msg;
  memset(&announce_msg, 0, sizeof(announce_msg));

  /* Create header */
  ptp_create_header(&announce_msg.header, PTP_MSG_ANNOUNCE, PTP_ANNOUNCE_SIZE, 5);

  /* Set announce specific fields */
  announce_msg.current_utc_offset = 0;
  announce_msg.grandmaster_priority1 = ctx->priority1;
  announce_msg.grandmaster_clock_quality_class = ctx->clock_class;
  announce_msg.grandmaster_clock_quality_accuracy = ctx->clock_accuracy;
  announce_msg.grandmaster_clock_quality_offset_scaled_log_variance = 0x4E5D;
  announce_msg.grandmaster_priority2 = ctx->priority2;
  memcpy(announce_msg.grandmaster_clock_identity, ctx->clock_identity, 8);
  announce_msg.steps_removed = 0;
  announce_msg.time_source = ctx->clock_source;

  /* Send message */
  return ptp_send_message(ctx, (uint8_t *)&announce_msg, PTP_ANNOUNCE_SIZE);
}

/**
  * @brief  Set local clock time
  * @param  ctx: PTP context structure
  * @param  seconds: Seconds since epoch
  * @param  nanoseconds: Nanoseconds
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_set_time(ptp_context_t *ctx, uint64_t seconds, uint32_t nanoseconds)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Set system time (simplified implementation) */
  /* In real implementation, this would set the system RTC */

  return HAL_OK;
}

/**
  * @brief  Get local clock time
  * @param  ctx: PTP context structure
  * @param  seconds: Seconds since epoch (output)
  * @param  nanoseconds: Nanoseconds (output)
  * @retval HAL status
  */
HAL_StatusTypeDef ptp_get_time(ptp_context_t *ctx, uint64_t *seconds, uint32_t *nanoseconds)
{
  if (ctx == NULL || seconds == NULL || nanoseconds == NULL) {
    return HAL_ERROR;
  }

  /* Get current system time (simplified implementation) */
  uint64_t current_time_ms = HAL_GetTick();
  *seconds = current_time_ms / 1000;
  *nanoseconds = (current_time_ms % 1000) * 1000000;

  return HAL_OK;
}

/**
  * @brief  Calculate clock offset from master
  * @param  ctx: PTP context structure
  * @param  t1: T1 timestamp (local send time)
  * @param  t2: T2 timestamp (remote receive time)
  * @param  t3: T3 timestamp (remote send time)
  * @param  t4: T4 timestamp (local receive time)
  * @retval Offset in nanoseconds
  */
int64_t ptp_calculate_offset(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4)
{
  /* Standard PTP offset calculation: offset = [(t2 - t1) - (t4 - t3)] / 2 */
  int64_t offset = ((int64_t)(t2 - t1) - (int64_t)(t4 - t3)) / 2;
  return offset;
}

/**
  * @brief  Calculate path delay
  * @param  ctx: PTP context structure
  * @param  t1: T1 timestamp (local send time)
  * @param  t2: T2 timestamp (remote receive time)
  * @param  t3: T3 timestamp (remote send time)
  * @param  t4: T4 timestamp (local receive time)
  * @retval Path delay in nanoseconds
  */
int64_t ptp_calculate_delay(uint64_t t1, uint64_t t2, uint64_t t3, uint64_t t4)
{
  /* Standard PTP delay calculation: delay = [(t4 - t1) - (t3 - t2)] / 2 */
  int64_t delay = ((int64_t)(t4 - t1) - (int64_t)(t3 - t2)) / 2;
  return delay;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Create PTP message header
  * @param  header: Header structure
  * @param  message_type: Message type
  * @param  message_length: Message length
  * @param  control: Control field
  * @retval HAL status
  */
static HAL_StatusTypeDef ptp_create_header(ptp_header_t *header, uint8_t message_type,
                                         uint16_t message_length, uint8_t control)
{
  if (header == NULL) {
    return HAL_ERROR;
  }

  memset(header, 0, sizeof(ptp_header_t));

  header->transport_specific = PTP_TSP_ETHERNET;
  header->message_type = message_type;
  header->version_ptp = PTP_VERSION;
  header->message_length = message_length;
  header->domain_number = ptp_ctx_global->domain_number;
  header->flags = 0;

  /* Set flags based on message type */
  if (message_type == PTP_MSG_SYNC) {
    header->flags |= (1 << 0);  // Two-step flag
  }

  header->correction_field = 0;
  memcpy(header->clock_identity, ptp_clock_identity, 8);
  header->source_port_id = 1;  // Default port
  header->sequence_id = ++sequence_id;
  header->control = control;
  header->log_message_interval = ptp_ctx_global->sync_interval;

  return HAL_OK;
}

/**
  * @brief  Send PTP message
  * @param  ctx: PTP context structure
  * @param  message: Message buffer
  * @param  length: Message length
  * @retval HAL status
  */
static HAL_StatusTypeDef ptp_send_message(ptp_context_t *ctx, uint8_t *message, uint16_t length)
{
  if (ctx == NULL || message == NULL || length == 0) {
    return HAL_ERROR;
  }

  /* In phase 1, we use UART for testing */
  /* In real implementation, this would use Ethernet with hardware timestamping */

  /* For now, just log the message */
  printf("PTP: Sending message type 0x%02X, length %d\r\n", message[1], length);

  return HAL_OK;
}

/**
  * @brief  Receive PTP message
  * @param  ctx: PTP context structure
  * @param  buffer: Receive buffer
  * @param  length: Received length
  * @retval HAL status
  */
static HAL_StatusTypeDef ptp_receive_message(ptp_context_t *ctx, uint8_t *buffer, uint16_t *length)
{
  if (ctx == NULL || buffer == NULL || length == NULL) {
    return HAL_ERROR;
  }

  /* In phase 1, we simulate message reception */
  /* In real implementation, this would use Ethernet interrupt */

  return HAL_OK;
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

  switch (ctx->state) {
    case PTP_STATE_INITIALIZING:
      /* Check if we have GNSS lock */
      if (ctx->clock_source == PTP_SOURCE_GNSS) {
        ctx->state = PTP_STATE_MASTER;  // For phase 1, assume we're master
      } else {
        ctx->state = PTP_STATE_LISTENING;
      }
      break;

    case PTP_STATE_MASTER:
      /* Send periodic SYNC and ANNOUNCE messages */
      break;

    case PTP_STATE_SLAVE:
      /* Process incoming SYNC messages */
      break;

    default:
      /* Other states */
      break;
  }
}

/**
  * @brief  Check if received master is better than current
  * @param  ctx: PTP context structure
  * @param  announce: Received announce message
  * @retval true if better master, false otherwise
  */
static bool ptp_is_better_master(ptp_context_t *ctx, ptp_announce_msg_t *announce)
{
  if (ctx == NULL || announce == NULL) {
    return false;
  }

  /* Simplified BMC algorithm for phase 1 */
  /* In real implementation, this would follow IEEE 1588 BMC rules */

  /* Compare priority1 */
  if (announce->grandmaster_priority1 < ctx->priority1) {
    return true;
  } else if (announce->grandmaster_priority1 > ctx->priority1) {
    return false;
  }

  /* Compare priority2 */
  if (announce->grandmaster_priority2 < ctx->priority2) {
    return true;
  } else if (announce->grandmaster_priority2 > ctx->priority2) {
    return false;
  }

  /* Compare clock class */
  if (announce->grandmaster_clock_quality_class < ctx->clock_class) {
    return true;
  }

  return false;
}

/**
  * @brief  Send PTP delay response
  * @param  ctx: PTP context structure
  * @param  delay_req: Received delay request
  * @retval HAL status
  */
static HAL_StatusTypeDef ptp_send_delay_resp(ptp_context_t *ctx, ptp_delay_req_msg_t *delay_req)
{
  if (ctx == NULL || delay_req == NULL) {
    return HAL_ERROR;
  }

  ptp_delay_resp_msg_t delay_resp;
  memset(&delay_resp, 0, sizeof(delay_resp));

  /* Create header */
  ptp_create_header(&delay_resp.header, PTP_MSG_DELAY_RESP, PTP_DELAY_RESP_SIZE, 0);

  /* Set receive timestamp (when we received the delay request) */
  uint64_t receive_time = HAL_GetTick() * 1000000ULL;  // Convert to nanoseconds
  delay_resp.receive_timestamp_seconds = receive_time / 1000000000ULL;
  delay_resp.receive_timestamp_nanoseconds = receive_time % 1000000000ULL;

  /* Copy requesting clock identity */
  memcpy(delay_resp.requesting_clock_identity, delay_req->header.clock_identity, 8);
  delay_resp.requesting_source_port_id = delay_req->header.source_port_id;

  /* Send message */
  return ptp_send_message(ctx, (uint8_t *)&delay_resp, PTP_DELAY_RESP_SIZE);
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
