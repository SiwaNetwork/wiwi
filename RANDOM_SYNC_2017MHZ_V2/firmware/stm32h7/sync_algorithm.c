/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : sync_algorithm.c
  * @brief          : Synchronization algorithms for RANDOM_SYNC_2017MHZ_V2
  * @author         : WiWi Development Team
  * @version        : 1.0.0
  * @date           : January 2025
  ******************************************************************************
  * @attention
  *
  * Synchronization Algorithms Implementation for STM32H7
  * Supports multiple algorithms: Linear, PID, Kalman, Adaptive
  * Designed for sub-nanosecond accuracy requirements
  *
  * Based on PTP synchronization requirements
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "sync_algorithm.h"
#include "main.h"
#include <string.h>
#include <math.h>

/* Private defines -----------------------------------------------------------*/

/* Algorithm constants */
#define MAX_OFFSET_HISTORY          100
#define OFFSET_FILTER_ALPHA         0.1
#define DRIFT_ESTIMATION_WINDOW     10

/* PID controller constants */
#define PID_KP_DEFAULT              0.001
#define PID_KI_DEFAULT              0.00001
#define PID_KD_DEFAULT              0.0001

/* Kalman filter constants */
#define KALMAN_PROCESS_NOISE        1e-6
#define KALMAN_MEASUREMENT_NOISE    1e-3
#define KALMAN_INITIAL_ESTIMATE     0.0
#define KALMAN_INITIAL_COVARIANCE   1.0

/* Private variables ---------------------------------------------------------*/
static int64_t offset_history[MAX_OFFSET_HISTORY];
static uint32_t history_index = 0;
static uint32_t history_count = 0;

/* Exported variables --------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static int64_t calculate_frequency_adjustment(sync_context_t *ctx, int64_t offset_ns);
static void update_offset_history(int64_t offset_ns);
static int64_t get_filtered_offset(void);
static double calculate_offset_variance(int64_t *values, uint32_t count, int64_t mean);

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  Initialize synchronization algorithms
  * @param  ctx: Sync context structure
  * @retval HAL status
  */
HAL_StatusTypeDef sync_init(sync_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Initialize context */
  memset(ctx, 0, sizeof(sync_context_t));

  /* Set default parameters */
  ctx->algorithm_type = SYNC_ALGORITHM_PID;  // Default to PID
  ctx->target_accuracy_ns = DEFAULT_TARGET_ACCURACY_NS;
  ctx->update_interval_ms = DEFAULT_UPDATE_INTERVAL_MS;
  ctx->holdover_timeout_ms = DEFAULT_HOLDOVER_TIMEOUT_MS;

  /* Initialize PID parameters */
  ctx->kp = PID_KP_DEFAULT;
  ctx->ki = PID_KI_DEFAULT;
  ctx->kd = PID_KD_DEFAULT;

  /* Initialize Kalman filter parameters */
  ctx->kalman_q = KALMAN_PROCESS_NOISE;
  ctx->kalman_r = KALMAN_MEASUREMENT_NOISE;
  ctx->kalman_x = KALMAN_INITIAL_ESTIMATE;
  ctx->kalman_p = KALMAN_INITIAL_COVARIANCE;

  /* Reset state */
  ctx->current_offset_ns = 0;
  ctx->current_drift_ppb = 0;
  ctx->last_update_time = 0;
  ctx->in_holdover = false;

  /* Reset statistics */
  ctx->sync_count = 0;
  ctx->adjustment_count = 0;
  ctx->total_offset_ns = 0;
  ctx->max_offset_ns = 0;
  ctx->min_offset_ns = 0;

  /* Initialize offset history */
  memset(offset_history, 0, sizeof(offset_history));
  history_index = 0;
  history_count = 0;

  ctx->initialized = true;

  return HAL_OK;
}

/**
  * @brief  Configure synchronization parameters
  * @param  ctx: Sync context structure
  * @param  config: Configuration parameters
  * @retval HAL status
  */
HAL_StatusTypeDef sync_configure(sync_context_t *ctx, sync_config_t *config)
{
  if (ctx == NULL || config == NULL) {
    return HAL_ERROR;
  }

  if (!ctx->initialized) {
    return HAL_ERROR;
  }

  /* Update configuration */
  ctx->algorithm_type = config->algorithm_type;
  ctx->target_accuracy_ns = config->target_accuracy_ns;
  ctx->update_interval_ms = config->update_interval_ms;
  ctx->holdover_timeout_ms = config->holdover_timeout_ms;

  /* Update PID parameters if using PID algorithm */
  if (config->algorithm_type == SYNC_ALGORITHM_PID) {
    ctx->kp = config->kp;
    ctx->ki = config->ki;
    ctx->kd = config->kd;
  }

  /* Update Kalman parameters if using Kalman algorithm */
  if (config->algorithm_type == SYNC_ALGORITHM_KALMAN) {
    ctx->kalman_q = config->kalman_q;
    ctx->kalman_r = config->kalman_r;
  }

  return HAL_OK;
}

/**
  * @brief  Update synchronization with new measurement
  * @param  ctx: Sync context structure
  * @param  offset_ns: Measured offset in nanoseconds
  * @param  timestamp_ms: Measurement timestamp in milliseconds
  * @retval HAL status
  */
HAL_StatusTypeDef sync_update(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms)
{
  if (ctx == NULL || !ctx->initialized) {
    return HAL_ERROR;
  }

  /* Update offset history */
  update_offset_history(offset_ns);

  /* Update statistics */
  ctx->sync_count++;
  ctx->total_offset_ns += offset_ns;

  if (offset_ns > ctx->max_offset_ns) {
    ctx->max_offset_ns = offset_ns;
  }

  if (offset_ns < ctx->min_offset_ns) {
    ctx->min_offset_ns = offset_ns;
  }

  /* Store current offset */
  ctx->current_offset_ns = offset_ns;
  ctx->last_update_time = timestamp_ms;

  /* Calculate frequency adjustment based on algorithm */
  int64_t adjustment_ppb = calculate_frequency_adjustment(ctx, offset_ns);

  if (adjustment_ppb != 0) {
    ctx->adjustment_count++;
    ctx->current_drift_ppb = adjustment_ppb;
  }

  return HAL_OK;
}

/**
  * @brief  Get current synchronization status
  * @param  ctx: Sync context structure
  * @param  status: Status information (output)
  * @retval HAL status
  */
HAL_StatusTypeDef sync_get_status(sync_context_t *ctx, sync_status_t *status)
{
  if (ctx == NULL || status == NULL) {
    return HAL_ERROR;
  }

  status->synchronized = (abs(ctx->current_offset_ns) <= ctx->target_accuracy_ns);
  status->current_offset_ns = ctx->current_offset_ns;
  status->current_drift_ppb = ctx->current_drift_ppb;
  status->time_since_last_sync_ms = HAL_GetTick() - ctx->last_update_time;
  status->in_holdover = ctx->in_holdover;
  status->holdover_duration_ms = ctx->in_holdover ?
    (HAL_GetTick() - ctx->last_update_time) : 0;

  return HAL_OK;
}

/**
  * @brief  Get current accuracy estimate
  * @param  ctx: Sync context structure
  * @retval Current accuracy in nanoseconds
  */
int64_t sync_get_current_accuracy(sync_context_t *ctx)
{
  if (ctx == NULL || !ctx->initialized) {
    return INT64_MAX;
  }

  /* Return absolute offset as accuracy estimate */
  return abs(ctx->current_offset_ns);
}

/**
  * @brief  Check if system is in holdover mode
  * @param  ctx: Sync context structure
  * @retval true if in holdover, false otherwise
  */
bool sync_is_in_holdover(sync_context_t *ctx)
{
  if (ctx == NULL || !ctx->initialized) {
    return false;
  }

  return ctx->in_holdover;
}

/**
  * @brief  Reset synchronization state
  * @param  ctx: Sync context structure
  * @retval HAL status
  */
HAL_StatusTypeDef sync_reset(sync_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  /* Reset state */
  ctx->current_offset_ns = 0;
  ctx->current_drift_ppb = 0;
  ctx->last_update_time = HAL_GetTick();
  ctx->in_holdover = false;

  /* Reset statistics */
  ctx->sync_count = 0;
  ctx->adjustment_count = 0;
  ctx->total_offset_ns = 0;
  ctx->max_offset_ns = 0;
  ctx->min_offset_ns = 0;

  /* Reset offset history */
  memset(offset_history, 0, sizeof(offset_history));
  history_index = 0;
  history_count = 0;

  /* Reset Kalman filter state */
  ctx->kalman_x = KALMAN_INITIAL_ESTIMATE;
  ctx->kalman_p = KALMAN_INITIAL_COVARIANCE;

  return HAL_OK;
}

/**
  * @brief  Enable holdover mode
  * @param  ctx: Sync context structure
  * @retval HAL status
  */
HAL_StatusTypeDef sync_enable_holdover(sync_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  ctx->in_holdover = true;
  return HAL_OK;
}

/**
  * @brief  Disable holdover mode
  * @param  ctx: Sync context structure
  * @retval HAL status
  */
HAL_StatusTypeDef sync_disable_holdover(sync_context_t *ctx)
{
  if (ctx == NULL) {
    return HAL_ERROR;
  }

  ctx->in_holdover = false;
  return HAL_OK;
}

/**
  * @brief  Get synchronization statistics
  * @param  ctx: Sync context structure
  * @param  sync_count: Number of sync operations (output)
  * @param  adjustment_count: Number of adjustments made (output)
  * @param  average_offset: Average offset (output)
  * @retval HAL status
  */
HAL_StatusTypeDef sync_get_statistics(sync_context_t *ctx, uint32_t *sync_count,
                                     uint32_t *adjustment_count, int64_t *average_offset)
{
  if (ctx == NULL || sync_count == NULL || adjustment_count == NULL || average_offset == NULL) {
    return HAL_ERROR;
  }

  *sync_count = ctx->sync_count;
  *adjustment_count = ctx->adjustment_count;

  if (ctx->sync_count > 0) {
    *average_offset = ctx->total_offset_ns / ctx->sync_count;
  } else {
    *average_offset = 0;
  }

  return HAL_OK;
}

/* Algorithm implementations ---------------------------------------------*/

/**
  * @brief  Linear synchronization algorithm
  * @param  ctx: Sync context structure
  * @param  offset_ns: Measured offset
  * @param  timestamp_ms: Measurement timestamp
  * @retval Frequency adjustment in ppb
  */
int64_t sync_algorithm_linear(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms)
{
  if (ctx == NULL) {
    return 0;
  }

  /* Simple proportional control */
  int64_t adjustment_ppb = (offset_ns * ctx->kp * 1000) / ctx->update_interval_ms;

  /* Clamp to reasonable range */
  adjustment_ppb = clamp_int64(adjustment_ppb, -1000, 1000);

  return adjustment_ppb;
}

/**
  * @brief  PID synchronization algorithm
  * @param  ctx: Sync context structure
  * @param  offset_ns: Measured offset
  * @param  timestamp_ms: Measurement timestamp
  * @retval Frequency adjustment in ppb
  */
int64_t sync_algorithm_pid(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms)
{
  static int64_t integral = 0;
  static int64_t previous_error = 0;
  static uint32_t last_timestamp = 0;

  if (ctx == NULL) {
    return 0;
  }

  /* Calculate time delta */
  uint32_t dt_ms = timestamp_ms - last_timestamp;
  if (dt_ms == 0) {
    dt_ms = ctx->update_interval_ms;
  }
  last_timestamp = timestamp_ms;

  /* PID calculation */
  int64_t error = offset_ns;

  /* Proportional term */
  double p_term = ctx->kp * error;

  /* Integral term */
  integral += error * dt_ms;
  integral = clamp_int64(integral, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);
  double i_term = ctx->ki * integral;

  /* Derivative term */
  int64_t derivative = (error - previous_error) / dt_ms;
  double d_term = ctx->kd * derivative;

  /* Calculate output */
  int64_t output = (int64_t)(p_term + i_term + d_term);

  /* Clamp output */
  output = clamp_int64(output, PID_OUTPUT_MIN, PID_OUTPUT_MAX);

  /* Update state */
  previous_error = error;

  return output;
}

/**
  * @brief  Kalman filter synchronization algorithm
  * @param  ctx: Sync context structure
  * @param  offset_ns: Measured offset
  * @param  timestamp_ms: Measurement timestamp
  * @retval Frequency adjustment in ppb
  */
int64_t sync_algorithm_kalman(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms)
{
  if (ctx == NULL) {
    return 0;
  }

  /* Kalman filter prediction step */
  double x_pred = ctx->kalman_x;
  double p_pred = ctx->kalman_p + ctx->kalman_q;

  /* Kalman filter update step */
  double k_gain = p_pred / (p_pred + ctx->kalman_r);
  ctx->kalman_x = x_pred + k_gain * (offset_ns - x_pred);
  ctx->kalman_p = (1 - k_gain) * p_pred;

  /* Convert state estimate to frequency adjustment */
  int64_t adjustment_ppb = (int64_t)(ctx->kalman_x * 1000);  // Convert to ppb

  /* Clamp to reasonable range */
  adjustment_ppb = clamp_int64(adjustment_ppb, -1000, 1000);

  return adjustment_ppb;
}

/**
  * @brief  Adaptive synchronization algorithm
  * @param  ctx: Sync context structure
  * @param  offset_ns: Measured offset
  * @param  timestamp_ms: Measurement timestamp
  * @retval Frequency adjustment in ppb
  */
int64_t sync_algorithm_adaptive(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms)
{
  if (ctx == NULL) {
    return 0;
  }

  /* Get filtered offset for stability */
  int64_t filtered_offset = get_filtered_offset();

  /* Calculate variance for adaptive behavior */
  double variance = calculate_offset_variance(offset_history,
                                           (history_count < MAX_OFFSET_HISTORY) ? history_count : MAX_OFFSET_HISTORY,
                                           filtered_offset);

  /* Adaptive algorithm selection based on variance */
  if (variance < 100) {
    /* Low variance - use Kalman filter for precision */
    return sync_algorithm_kalman(ctx, offset_ns, timestamp_ms);
  } else if (variance < 1000) {
    /* Medium variance - use PID for balance */
    return sync_algorithm_pid(ctx, offset_ns, timestamp_ms);
  } else {
    /* High variance - use linear for stability */
    return sync_algorithm_linear(ctx, offset_ns, timestamp_ms);
  }
}

/* Utility functions --------------------------------------------------------*/

/**
  * @brief  Convert nanoseconds to frequency adjustment in ppb
  * @param  offset_ns: Time offset in nanoseconds
  * @param  interval_ms: Measurement interval in milliseconds
  * @retval Frequency adjustment in ppb
  */
int64_t ns_to_ppb(int64_t offset_ns, uint32_t interval_ms)
{
  if (interval_ms == 0) {
    return 0;
  }

  /* Convert: offset_ns / interval_ms * 1000 ppb */
  return (offset_ns * 1000) / interval_ms;
}

/**
  * @brief  Convert frequency adjustment in ppb to nanoseconds
  * @param  adjustment_ppb: Frequency adjustment in ppb
  * @param  interval_ms: Time interval in milliseconds
  * @retval Time offset in nanoseconds
  */
int64_t ppb_to_ns(int64_t adjustment_ppb, uint32_t interval_ms)
{
  if (interval_ms == 0) {
    return 0;
  }

  /* Convert: adjustment_ppb * interval_ms / 1000 ns */
  return (adjustment_ppb * interval_ms) / 1000;
}

/**
  * @brief  Clamp value to specified range
  * @param  value: Input value
  * @param  min: Minimum value
  * @param  max: Maximum value
  * @retval Clamped value
  */
int64_t clamp_int64(int64_t value, int64_t min, int64_t max)
{
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

/**
  * @brief  Calculate moving average
  * @param  values: Array of values
  * @param  count: Number of values
  * @retval Average value
  */
int64_t calculate_moving_average(int64_t *values, uint32_t count)
{
  if (values == NULL || count == 0) {
    return 0;
  }

  int64_t sum = 0;
  for (uint32_t i = 0; i < count; i++) {
    sum += values[i];
  }

  return sum / count;
}

/**
  * @brief  Calculate standard deviation
  * @param  values: Array of values
  * @param  count: Number of values
  * @param  mean: Mean value
  * @retval Standard deviation
  */
double calculate_standard_deviation(int64_t *values, uint32_t count, int64_t mean)
{
  if (values == NULL || count == 0) {
    return 0.0;
  }

  double sum_squared_diff = 0.0;
  for (uint32_t i = 0; i < count; i++) {
    int64_t diff = values[i] - mean;
    sum_squared_diff += (double)diff * (double)diff;
  }

  return sqrt(sum_squared_diff / count);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Calculate frequency adjustment based on algorithm
  * @param  ctx: Sync context structure
  * @param  offset_ns: Measured offset in nanoseconds
  * @retval Frequency adjustment in ppb
  */
static int64_t calculate_frequency_adjustment(sync_context_t *ctx, int64_t offset_ns)
{
  if (ctx == NULL) {
    return 0;
  }

  switch (ctx->algorithm_type) {
    case SYNC_ALGORITHM_LINEAR:
      return sync_algorithm_linear(ctx, offset_ns, HAL_GetTick());

    case SYNC_ALGORITHM_PID:
      return sync_algorithm_pid(ctx, offset_ns, HAL_GetTick());

    case SYNC_ALGORITHM_KALMAN:
      return sync_algorithm_kalman(ctx, offset_ns, HAL_GetTick());

    case SYNC_ALGORITHM_ADAPTIVE:
      return sync_algorithm_adaptive(ctx, offset_ns, HAL_GetTick());

    default:
      return 0;
  }
}

/**
  * @brief  Update offset history for filtering
  * @param  offset_ns: New offset value
  * @retval None
  */
static void update_offset_history(int64_t offset_ns)
{
  offset_history[history_index] = offset_ns;
  history_index = (history_index + 1) % MAX_OFFSET_HISTORY;

  if (history_count < MAX_OFFSET_HISTORY) {
    history_count++;
  }
}

/**
  * @brief  Get filtered offset value
  * @retval Filtered offset in nanoseconds
  */
static int64_t get_filtered_offset(void)
{
  if (history_count == 0) {
    return 0;
  }

  /* Use exponential moving average for filtering */
  static int64_t filtered_offset = 0;
  int64_t current_offset = offset_history[history_index == 0 ? MAX_OFFSET_HISTORY - 1 : history_index - 1];

  filtered_offset = (int64_t)(OFFSET_FILTER_ALPHA * current_offset +
                             (1.0 - OFFSET_FILTER_ALPHA) * filtered_offset);

  return filtered_offset;
}

/**
  * @brief  Calculate offset variance for adaptive algorithm
  * @param  values: Array of offset values
  * @param  count: Number of values
  * @param  mean: Mean offset value
  * @retval Variance value
  */
static double calculate_offset_variance(int64_t *values, uint32_t count, int64_t mean)
{
  if (values == NULL || count == 0) {
    return 0.0;
  }

  double sum_squared_diff = 0.0;
  for (uint32_t i = 0; i < count; i++) {
    int64_t diff = values[i] - mean;
    sum_squared_diff += (double)diff * (double)diff;
  }

  return sum_squared_diff / count;
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
