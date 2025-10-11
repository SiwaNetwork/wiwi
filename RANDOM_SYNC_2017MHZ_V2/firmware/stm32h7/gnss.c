/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : gnss.c
  * @brief          : GNSS receiver driver for RANDOM_SYNC_2017MHZ_V2
  * @author         : WiWi Development Team
  * @version        : 1.0.0
  * @date           : January 2025
  ******************************************************************************
  * @attention
  *
  * GNSS Receiver Implementation for STM32H7
  * Supports GPS, GLONASS, Galileo, BeiDou
  * NMEA protocol parsing and PPS handling
  *
  * Based on u-blox M8/M9 series GNSS modules
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gnss.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/

/* UART configuration */
#define GNSS_RX_BUFFER_SIZE         256
#define GNSS_TX_BUFFER_SIZE         128

/* NMEA parsing constants */
#define NMEA_START_CHAR             '$'
#define NMEA_END_CHAR               '\n'
#define NMEA_CHECKSUM_DELIMITER     '*'
#define NMEA_FIELD_DELIMITER        ','

/* GNSS command strings */
#define GNSS_CMD_GPS_ONLY           "$PUBX,40,GPS,0,0,0,0*5B\r\n"
#define GNSS_CMD_GLONASS_ONLY       "$PUBX,40,GLO,0,0,0,0*5C\r\n"
#define GNSS_CMD_GALILEO_ONLY       "$PUBX,40,GAL,0,0,0,0*5E\r\n"
#define GNSS_CMD_BEIDOU_ONLY        "$PUBX,40,BDS,0,0,0,0*5F\r\n"
#define GNSS_CMD_MULTI_CONST        "$PUBX,40,ALL,0,0,0,0*5A\r\n"
#define GNSS_CMD_UPDATE_RATE_1HZ    "$PUBX,40,ALL,0,0,0,0*5A\r\n"
#define GNSS_CMD_UPDATE_RATE_2HZ    "$PUBX,40,ALL,0,0,0,0*5A\r\n"
#define GNSS_CMD_UPDATE_RATE_5HZ    "$PUBX,40,ALL,0,0,0,0*5A\r\n"
#define GNSS_CMD_UPDATE_RATE_10HZ   "$PUBX,40,ALL,0,0,0,0*5A\r\n"

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *gnss_uart_handle;
static GPIO_TypeDef *gnss_pps_port;
static uint16_t gnss_pps_pin;
static char gnss_rx_buffer[GNSS_RX_BUFFER_SIZE];
static char gnss_tx_buffer[GNSS_TX_BUFFER_SIZE];
static volatile uint32_t gnss_rx_index = 0;
static volatile bool gnss_message_ready = false;

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static HAL_StatusTypeDef gnss_configure_hardware(gnss_context_t *ctx);
static HAL_StatusTypeDef gnss_send_ublox_command(gnss_context_t *ctx, const char *command);
static HAL_StatusTypeDef gnss_parse_nmea_message(gnss_context_t *ctx, char *message);
static bool gnss_validate_nmea_checksum(const char *message);
static char **gnss_split_nmea_fields(char *message, uint8_t *field_count);
static void gnss_free_fields(char **fields, uint8_t field_count);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Initialize GNSS receiver
  * @param  ctx: GNSS context structure
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_init(gnss_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Store hardware handles */
  gnss_uart_handle = ctx->uart_handle;
  gnss_pps_port = ctx->pps_port;
  gnss_pps_pin = ctx->pps_pin;

  /* Initialize hardware */
  ret = gnss_configure_hardware(ctx);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Initialize context */
  memset(ctx, 0, sizeof(gnss_context_t));
  ctx->status = GNSS_STATUS_IDLE;
  ctx->fix_type = GNSS_FIX_NONE;
  ctx->satellites_used = 0;
  ctx->satellites_visible = 0;
  ctx->hdop = 99.0f;
  ctx->pdop = 99.0f;
  ctx->vdop = 99.0f;
  ctx->pps_received = false;
  ctx->messages_received = 0;
  ctx->checksum_errors = 0;
  ctx->parse_errors = 0;
  ctx->initialized = true;

  /* Start UART reception */
  ret = HAL_UART_Receive_IT(gnss_uart_handle, (uint8_t *)&gnss_rx_buffer[gnss_rx_index], 1);
  if (ret != HAL_OK) {
    return ret;
  }

  /* Configure PPS interrupt */
  if (gnss_pps_port != NULL && gnss_pps_pin != 0) {
    /* Configure PPS pin as interrupt */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gnss_pps_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(gnss_pps_port, &GPIO_InitStruct);

    /* Enable EXTI interrupt */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  }

  return HAL_OK;
}

/**
  * @brief  Configure GNSS parameters
  * @param  ctx: GNSS context structure
  * @param  config: Configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_configure(gnss_context_t *ctx, gnss_config_t *config)
{
  HAL_StatusTypeDef ret = HAL_OK;

  if (ctx == NULL || config == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  /* Update configuration */
  ctx->enable_gps = config->enable_gps;
  ctx->enable_glonass = config->enable_glonass;
  ctx->enable_galileo = config->enable_galileo;
  ctx->enable_beidou = config->enable_beidou;
  ctx->update_rate = config->update_rate;

  /* Configure GNSS constellations */
  if (config->enable_gps && !config->enable_glonass &&
      !config->enable_galileo && !config->enable_beidou) {
    ret = gnss_send_ublox_command(ctx, GNSS_CMD_GPS_ONLY);
  } else if (!config->enable_gps && config->enable_glonass &&
             !config->enable_galileo && !config->enable_beidou) {
    ret = gnss_send_ublox_command(ctx, GNSS_CMD_GLONASS_ONLY);
  } else if (!config->enable_gps && !config->enable_glonass &&
             config->enable_galileo && !config->enable_beidou) {
    ret = gnss_send_ublox_command(ctx, GNSS_CMD_GALILEO_ONLY);
  } else if (!config->enable_gps && !config->enable_glonass &&
             !config->enable_galileo && config->enable_beidou) {
    ret = gnss_send_ublox_command(ctx, GNSS_CMD_BEIDOU_ONLY);
  } else {
    ret = gnss_send_ublox_command(ctx, GNSS_CMD_MULTI_CONST);
  }

  if (ret != HAL_OK) {
    return ret;
  }

  /* Configure update rate */
  switch (config->update_rate) {
    case 1:
      ret = gnss_send_ublox_command(ctx, GNSS_CMD_UPDATE_RATE_1HZ);
      break;
    case 2:
      ret = gnss_send_ublox_command(ctx, GNSS_CMD_UPDATE_RATE_2HZ);
      break;
    case 5:
      ret = gnss_send_ublox_command(ctx, GNSS_CMD_UPDATE_RATE_5HZ);
      break;
    case 10:
      ret = gnss_send_ublox_command(ctx, GNSS_CMD_UPDATE_RATE_10HZ);
      break;
    default:
      ret = HAL_ERROR;
      break;
  }

  return ret;
}

/**
  * @brief  Get GNSS status
  * @param  ctx: GNSS context structure
  * @param  status: Status information
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_get_status(gnss_context_t *ctx, gnss_status_t *status)
{
  if (ctx == NULL || status == NULL) {
    return HAL_ERROR;
  }

  *status = ctx->status;
  return HAL_OK;
}

/**
  * @brief  Get current position
  * @param  ctx: GNSS context structure
  * @param  latitude: Latitude (output)
  * @param  longitude: Longitude (output)
  * @param  altitude: Altitude (output)
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_get_position(gnss_context_t *ctx, double *latitude,
                                   double *longitude, double *altitude)
{
  if (ctx == NULL || latitude == NULL || longitude == NULL || altitude == NULL) {
    return HAL_ERROR;
  }

  if (ctx->fix_type == GNSS_FIX_NONE) {
    return HAL_ERROR;  // No valid fix
  }

  *latitude = ctx->latitude;
  *longitude = ctx->longitude;
  *altitude = ctx->altitude;

  return HAL_OK;
}

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
                               uint8_t *day, uint8_t *hour, uint8_t *minute, uint8_t *second)
{
  if (ctx == NULL || year == NULL || month == NULL || day == NULL ||
      hour == NULL || minute == NULL || second == NULL) {
    return HAL_ERROR;
  }

  *year = ctx->year;
  *month = ctx->month;
  *day = ctx->day;
  *hour = ctx->hour;
  *minute = ctx->minute;
  *second = ctx->second;

  return HAL_OK;
}

/**
  * @brief  Get PPS status
  * @param  ctx: GNSS context structure
  * @param  pps_received: PPS received flag (output)
  * @param  last_pps_time: Last PPS timestamp (output)
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_get_pps_status(gnss_context_t *ctx, bool *pps_received,
                                    uint64_t *last_pps_time)
{
  if (ctx == NULL || pps_received == NULL || last_pps_time == NULL) {
    return HAL_ERROR;
  }

  *pps_received = ctx->pps_received;
  *last_pps_time = ctx->last_pps_time;

  return HAL_OK;
}

/**
  * @brief  Parse NMEA message
  * @param  ctx: GNSS context structure
  * @param  message: NMEA message string
  * @param  length: Message length
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_parse_nmea(gnss_context_t *ctx, char *message, uint16_t length)
{
  if (ctx == NULL || message == NULL || length == 0) {
    return HAL_ERROR;
  }

  /* Validate message format */
  if (message[0] != NMEA_START_CHAR) {
    ctx->parse_errors++;
    return HAL_ERROR;
  }

  /* Check for end of message */
  if (length < 3 || message[length-1] != NMEA_END_CHAR) {
    return HAL_OK;  // Incomplete message
  }

  /* Validate checksum */
  if (!gnss_validate_nmea_checksum(message)) {
    ctx->checksum_errors++;
    return HAL_ERROR;
  }

  /* Update statistics */
  ctx->messages_received++;

  /* Parse message */
  return gnss_parse_nmea_message(ctx, message);
}

/**
  * @brief  Send command to GNSS receiver
  * @param  ctx: GNSS context structure
  * @param  command: Command string
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_send_command(gnss_context_t *ctx, const char *command)
{
  if (ctx == NULL || command == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  return gnss_send_ublox_command(ctx, command);
}

/**
  * @brief  Reset GNSS receiver
  * @param  ctx: GNSS context structure
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_reset(gnss_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Hardware reset if available */
  if (ctx->reset_port != NULL && ctx->reset_pin != 0) {
    HAL_GPIO_WritePin(ctx->reset_port, ctx->reset_pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(ctx->reset_port, ctx->reset_pin, GPIO_PIN_SET);
    HAL_Delay(1000);  // Wait for reset completion
  }

  /* Send reset command */
  return gnss_send_command(ctx, "$PUBX,40,RST,0,0,0,0*5D\r\n");
}

/**
  * @brief  Put GNSS receiver in standby mode
  * @param  ctx: GNSS context structure
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_standby(gnss_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Send standby command */
  return gnss_send_command(ctx, "$PUBX,40,STANDBY,0,0,0,0*5E\r\n");
}

/**
  * @brief  Wake up GNSS receiver from standby
  * @param  ctx: GNSS context structure
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_wakeup(gnss_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Send any command to wake up */
  return gnss_send_command(ctx, "$PUBX,40,ALL,0,0,0,0*5A\r\n");
}

/**
  * @brief  Get GNSS statistics
  * @param  ctx: GNSS context structure
  * @param  messages_received: Messages received counter (output)
  * @param  checksum_errors: Checksum errors counter (output)
  * @param  parse_errors: Parse errors counter (output)
  * @retval HAL status
  */
HAL_StatusTypeDef gnss_get_statistics(gnss_context_t *ctx, uint32_t *messages_received,
                                     uint32_t *checksum_errors, uint32_t *parse_errors)
{
  if (ctx == NULL || messages_received == NULL ||
      checksum_errors == NULL || parse_errors == NULL) {
    return HAL_ERROR;
  }

  *messages_received = ctx->messages_received;
  *checksum_errors = ctx->checksum_errors;
  *parse_errors = ctx->parse_errors;

  return HAL_OK;
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Configure GNSS hardware
  * @param  ctx: GNSS context structure
  * @retval HAL status
  */
static HAL_StatusTypeDef gnss_configure_hardware(gnss_context_t *ctx)
{
  HAL_StatusTypeDef ret = HAL_OK;

  /* Configure UART */
  if (gnss_uart_handle != NULL) {
    ret = HAL_UART_Init(gnss_uart_handle);
    if (ret != HAL_OK) {
      return ret;
    }
  }

  /* Configure PPS GPIO if available */
  if (gnss_pps_port != NULL && gnss_pps_pin != 0) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = gnss_pps_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(gnss_pps_port, &GPIO_InitStruct);
  }

  return HAL_OK;
}

/**
  * @brief  Send u-blox command to GNSS receiver
  * @param  ctx: GNSS context structure
  * @param  command: Command string
  * @retval HAL status
  */
static HAL_StatusTypeDef gnss_send_ublox_command(gnss_context_t *ctx, const char *command)
{
  if (gnss_uart_handle == NULL || command == NULL) {
    return HAL_ERROR;
  }

  /* Copy command to buffer */
  strncpy(gnss_tx_buffer, command, GNSS_TX_BUFFER_SIZE - 1);
  gnss_tx_buffer[GNSS_TX_BUFFER_SIZE - 1] = '\0';

  /* Send command */
  return HAL_UART_Transmit(gnss_uart_handle, (uint8_t *)gnss_tx_buffer,
                          strlen(gnss_tx_buffer), 1000);
}

/**
  * @brief  Parse NMEA message
  * @param  ctx: GNSS context structure
  * @param  message: NMEA message string
  * @retval HAL status
  */
static HAL_StatusTypeDef gnss_parse_nmea_message(gnss_context_t *ctx, char *message)
{
  uint8_t field_count = 0;
  char **fields = gnss_split_nmea_fields(message, &field_count);

  if (fields == NULL || field_count < 2) {
    gnss_free_fields(fields, field_count);
    return HAL_ERROR;
  }

  /* Determine message type and parse accordingly */
  if (strncmp(fields[0], "$GPGGA", 6) == 0 || strncmp(fields[0], "$GNGGA", 6) == 0) {
    return parse_gga_message(ctx, fields, field_count);
  } else if (strncmp(fields[0], "$GPRMC", 6) == 0 || strncmp(fields[0], "$GNRMC", 6) == 0) {
    return parse_rmc_message(ctx, fields, field_count);
  } else if (strncmp(fields[0], "$GPGSA", 6) == 0 || strncmp(fields[0], "$GNGSA", 6) == 0) {
    return parse_gsa_message(ctx, fields, field_count);
  }

  /* Unknown message type */
  gnss_free_fields(fields, field_count);
  return HAL_OK;
}

/**
  * @brief  Validate NMEA checksum
  * @param  message: NMEA message
  * @retval true if valid, false otherwise
  */
static bool gnss_validate_nmea_checksum(const char *message)
{
  if (message == NULL) {
    return false;
  }

  /* Find checksum delimiter */
  const char *checksum_ptr = strchr(message, NMEA_CHECKSUM_DELIMITER);
  if (checksum_ptr == NULL || *(checksum_ptr + 1) == '\0') {
    return false;
  }

  /* Calculate expected checksum */
  uint8_t calculated_checksum = calculate_nmea_checksum(message + 1, checksum_ptr - message - 1);

  /* Parse provided checksum */
  char checksum_str[3];
  strncpy(checksum_str, checksum_ptr + 1, 2);
  checksum_str[2] = '\0';

  uint8_t provided_checksum = (uint8_t)strtol(checksum_str, NULL, 16);

  return (calculated_checksum == provided_checksum);
}

/**
  * @brief  Calculate NMEA checksum
  * @param  message: NMEA message
  * @param  length: Message length
  * @retval Checksum value
  */
static uint8_t calculate_nmea_checksum(const char *message, uint16_t length)
{
  uint8_t checksum = 0;

  for (uint16_t i = 0; i < length; i++) {
    checksum ^= (uint8_t)message[i];
  }

  return checksum;
}

/**
  * @brief  Split NMEA message into fields
  * @param  message: NMEA message
  * @param  field_count: Number of fields (output)
  * @retval Array of field strings
  */
static char **gnss_split_nmea_fields(char *message, uint8_t *field_count)
{
  if (message == NULL || field_count == NULL) {
    return NULL;
  }

  /* Count fields */
  uint8_t count = 0;
  char *ptr = message;
  while ((ptr = strchr(ptr, NMEA_FIELD_DELIMITER)) != NULL) {
    count++;
    ptr++;
  }

  /* Allocate field array */
  char **fields = (char **)malloc((count + 1) * sizeof(char *));
  if (fields == NULL) {
    return NULL;
  }

  /* Split message into fields */
  ptr = message;
  uint8_t field_idx = 0;
  char *token = strtok(ptr, ",");

  while (token != NULL && field_idx < count) {
    fields[field_idx] = token;
    field_idx++;
    token = strtok(NULL, ",");
  }

  *field_count = field_idx;
  return fields;
}

/**
  * @brief  Free field array
  * @param  fields: Field array
  * @param  field_count: Number of fields
  * @retval None
  */
static void gnss_free_fields(char **fields, uint8_t field_count)
{
  if (fields != NULL) {
    free(fields);
  }
}

/**
  * @brief  Parse GGA message (Global Positioning System Fix Data)
  * @param  ctx: GNSS context structure
  * @param  fields: Message fields
  * @param  field_count: Number of fields
  * @retval HAL status
  */
static HAL_StatusTypeDef parse_gga_message(gnss_context_t *ctx, char **fields, uint8_t field_count)
{
  if (field_count < 14) {
    return HAL_ERROR;
  }

  /* Parse latitude */
  if (strlen(fields[2]) > 0) {
    double lat_minutes = atof(fields[2]) / 100.0;
    int lat_degrees = (int)lat_minutes;
    ctx->latitude = lat_degrees + (lat_minutes - lat_degrees) / 0.6;
    if (fields[3][0] == 'S') ctx->latitude = -ctx->latitude;
  }

  /* Parse longitude */
  if (strlen(fields[4]) > 0) {
    double lon_minutes = atof(fields[4]) / 100.0;
    int lon_degrees = (int)lon_minutes;
    ctx->longitude = lon_degrees + (lon_minutes - lon_degrees) / 0.6;
    if (fields[5][0] == 'W') ctx->longitude = -ctx->longitude;
  }

  /* Parse altitude */
  if (strlen(fields[9]) > 0) {
    ctx->altitude = atof(fields[9]);
  }

  /* Parse fix type */
  if (strlen(fields[6]) > 0) {
    int fix_type = atoi(fields[6]);
    switch (fix_type) {
      case 0: ctx->fix_type = GNSS_FIX_NONE; break;
      case 1: ctx->fix_type = GNSS_FIX_2D; break;
      case 2: ctx->fix_type = GNSS_FIX_3D; break;
      case 3: ctx->fix_type = GNSS_FIX_DGPS; break;
      default: ctx->fix_type = GNSS_FIX_NONE; break;
    }
  }

  /* Parse satellites used */
  if (strlen(fields[7]) > 0) {
    ctx->satellites_used = (uint8_t)atoi(fields[7]);
  }

  /* Parse HDOP */
  if (strlen(fields[8]) > 0) {
    ctx->hdop = (float)atof(fields[8]);
  }

  /* Update status */
  if (ctx->fix_type != GNSS_FIX_NONE) {
    ctx->status = GNSS_STATUS_TRACKING;
  } else {
    ctx->status = GNSS_STATUS_SEARCHING;
  }

  return HAL_OK;
}

/**
  * @brief  Parse RMC message (Recommended Minimum Navigation Information)
  * @param  ctx: GNSS context structure
  * @param  fields: Message fields
  * @param  field_count: Number of fields
  * @retval HAL status
  */
static HAL_StatusTypeDef parse_rmc_message(gnss_context_t *ctx, char **fields, uint8_t field_count)
{
  if (field_count < 12) {
    return HAL_ERROR;
  }

  /* Parse time */
  if (strlen(fields[1]) >= 6) {
    ctx->hour = (uint8_t)atoi(&fields[1][0]);
    ctx->minute = (uint8_t)atoi(&fields[1][2]);
    ctx->second = (uint8_t)atoi(&fields[1][4]);
  }

  /* Parse date */
  if (strlen(fields[9]) >= 6) {
    ctx->day = (uint8_t)atoi(&fields[9][0]);
    ctx->month = (uint8_t)atoi(&fields[9][2]);
    ctx->year = (uint16_t)atoi(&fields[9][4]) + 2000;
  }

  /* Parse speed */
  if (strlen(fields[7]) > 0) {
    ctx->speed = (float)atof(fields[7]) * 1.852f;  // Convert knots to km/h
  }

  /* Parse course */
  if (strlen(fields[8]) > 0) {
    ctx->course = (float)atof(fields[8]);
  }

  return HAL_OK;
}

/**
  * @brief  Parse GSA message (GNSS DOP and Active Satellites)
  * @param  ctx: GNSS context structure
  * @param  fields: Message fields
  * @param  field_count: Number of fields
  * @retval HAL status
  */
static HAL_StatusTypeDef parse_gsa_message(gnss_context_t *ctx, char **fields, uint8_t field_count)
{
  if (field_count < 18) {
    return HAL_ERROR;
  }

  /* Parse PDOP */
  if (strlen(fields[15]) > 0) {
    ctx->pdop = (float)atof(fields[15]);
  }

  /* Parse HDOP */
  if (strlen(fields[16]) > 0) {
    ctx->hdop = (float)atof(fields[16]);
  }

  /* Parse VDOP */
  if (strlen(fields[17]) > 0) {
    ctx->vdop = (float)atof(fields[17]);
  }

  return HAL_OK;
}

/**
  * @brief  Convert NMEA coordinate to decimal degrees
  * @param  nmea_coord: NMEA coordinate string
  * @param  hemisphere: Hemisphere character
  * @retval Coordinate in decimal degrees
  */
static double convert_nmea_to_decimal(const char *nmea_coord, char hemisphere)
{
  if (nmea_coord == NULL || strlen(nmea_coord) < 4) {
    return 0.0;
  }

  /* Parse degrees and minutes */
  double minutes = atof(nmea_coord) / 100.0;
  int degrees = (int)minutes;
  double decimal_minutes = minutes - degrees;

  /* Convert to decimal degrees */
  double decimal_degrees = degrees + (decimal_minutes / 0.6);

  /* Apply hemisphere */
  if (hemisphere == 'S' || hemisphere == 'W') {
    decimal_degrees = -decimal_degrees;
  }

  return decimal_degrees;
}

/**
  * @brief  PPS interrupt handler
  * @param  ctx: GNSS context structure
  * @retval None
  */
static void pps_interrupt_handler(gnss_context_t *ctx)
{
  if (ctx != NULL) {
    ctx->pps_received = true;
    ctx->last_pps_time = HAL_GetTick();

    /* Update system time if we have valid time from GNSS */
    if (ctx->year > 2000) {
      /* Set system RTC (simplified implementation) */
    }
  }
}

/**
  * @brief  UART receive complete callback
  * @param  ctx: GNSS context structure
  * @retval None
  */
static void uart_rx_complete_callback(gnss_context_t *ctx)
{
  /* Check for message end */
  if (gnss_rx_buffer[gnss_rx_index] == '\n') {
    /* Process complete message */
    char message[GNSS_RX_BUFFER_SIZE];
    memcpy(message, gnss_rx_buffer, gnss_rx_index + 1);
    message[gnss_rx_index + 1] = '\0';

    gnss_parse_nmea(ctx, message, gnss_rx_index + 1);

    /* Reset buffer */
    gnss_rx_index = 0;
    memset(gnss_rx_buffer, 0, sizeof(gnss_rx_buffer));
    gnss_message_ready = false;
  } else {
    /* Continue receiving */
    gnss_rx_index++;
    if (gnss_rx_index >= GNSS_RX_BUFFER_SIZE - 1) {
      /* Buffer overflow, reset */
      gnss_rx_index = 0;
      memset(gnss_rx_buffer, 0, sizeof(gnss_rx_buffer));
    }
  }

  /* Start next reception */
  HAL_UART_Receive_IT(gnss_uart_handle, (uint8_t *)&gnss_rx_buffer[gnss_rx_index], 1);
}

/* USER CODE BEGIN 1 */

/* UART RX Complete Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == gnss_uart_handle) {
    uart_rx_complete_callback(NULL);  // Simplified for phase 1
  }
}

/* EXTI Callback for PPS */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == gnss_pps_pin) {
    pps_interrupt_handler(NULL);  // Simplified for phase 1
  }
}

/* USER CODE END 1 */
