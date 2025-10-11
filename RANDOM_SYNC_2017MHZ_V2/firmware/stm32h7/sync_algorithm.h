#ifndef __SYNC_ALGORITHM_H
#define __SYNC_ALGORITHM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/* Sync Algorithm Types */
typedef enum {
    SYNC_ALGORITHM_NONE,
    SYNC_ALGORITHM_LINEAR,
    SYNC_ALGORITHM_PID,
    SYNC_ALGORITHM_KALMAN,
    SYNC_ALGORITHM_ADAPTIVE
} sync_algorithm_type_t;

/* Sync Context Structure */
typedef struct {
    /* Algorithm type */
    sync_algorithm_type_t algorithm_type;

    /* Target parameters */
    int64_t target_accuracy_ns;      // Target accuracy in nanoseconds
    uint32_t update_interval_ms;     // Update interval in milliseconds
    uint32_t holdover_timeout_ms;    // Holdover timeout in milliseconds

    /* Current state */
    int64_t current_offset_ns;       // Current offset from master (ns)
    int64_t current_drift_ppb;       // Current frequency drift (ppb)
    uint32_t last_update_time;       // Last update timestamp

    /* Statistics */
    uint32_t sync_count;
    uint32_t adjustment_count;
    int64_t total_offset_ns;
    int64_t max_offset_ns;
    int64_t min_offset_ns;

    /* Algorithm parameters */
    double kp;  // Proportional gain
    double ki;  // Integral gain
    double kd;  // Derivative gain

    /* Kalman filter state (for Kalman algorithm) */
    double kalman_x;    // State estimate
    double kalman_p;    // Error covariance
    double kalman_q;    // Process noise covariance
    double kalman_r;    // Measurement noise covariance

    bool initialized;
    bool in_holdover;
} sync_context_t;

/* Sync Configuration Structure */
typedef struct {
    sync_algorithm_type_t algorithm_type;
    int64_t target_accuracy_ns;
    uint32_t update_interval_ms;
    uint32_t holdover_timeout_ms;

    /* PID parameters (for PID algorithm) */
    double kp;  // Proportional gain
    double ki;  // Integral gain
    double kd;  // Derivative gain

    /* Kalman parameters (for Kalman algorithm) */
    double kalman_q;  // Process noise covariance
    double kalman_r;  // Measurement noise covariance
} sync_config_t;

/* Sync Status Structure */
typedef struct {
    bool synchronized;
    int64_t current_offset_ns;
    int64_t current_drift_ppb;
    uint32_t time_since_last_sync_ms;
    bool in_holdover;
    uint32_t holdover_duration_ms;
} sync_status_t;

/* Exported constants --------------------------------------------------------*/

/* Default sync parameters */
#define DEFAULT_TARGET_ACCURACY_NS    1000   // 1 μs
#define DEFAULT_UPDATE_INTERVAL_MS    1000   // 1 second
#define DEFAULT_HOLDOVER_TIMEOUT_MS   3600000 // 1 hour

/* PID controller limits */
#define PID_INTEGRAL_MAX              1000000
#define PID_INTEGRAL_MIN              -1000000
#define PID_OUTPUT_MAX                1000
#define PID_OUTPUT_MIN                -1000

/* Kalman filter parameters */
#define KALMAN_DEFAULT_Q              1e-6
#define KALMAN_DEFAULT_R              1e-3

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Initialize synchronization algorithms
 * @param  ctx: Sync context structure
 * @retval HAL status
 */
HAL_StatusTypeDef sync_init(sync_context_t *ctx);

/**
 * @brief  Configure synchronization parameters
 * @param  ctx: Sync context structure
 * @param  config: Configuration parameters
 * @retval HAL status
 */
HAL_StatusTypeDef sync_configure(sync_context_t *ctx, sync_config_t *config);

/**
 * @brief  Update synchronization with new measurement
 * @param  ctx: Sync context structure
 * @param  offset_ns: Measured offset in nanoseconds
 * @param  timestamp_ms: Measurement timestamp in milliseconds
 * @retval HAL status
 */
HAL_StatusTypeDef sync_update(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms);

/**
 * @brief  Get current synchronization status
 * @param  ctx: Sync context structure
 * @param  status: Status information (output)
 * @retval HAL status
 */
HAL_StatusTypeDef sync_get_status(sync_context_t *ctx, sync_status_t *status);

/**
 * @brief  Get current accuracy estimate
 * @param  ctx: Sync context structure
 * @retval Current accuracy in nanoseconds
 */
int64_t sync_get_current_accuracy(sync_context_t *ctx);

/**
 * @brief  Check if system is in holdover mode
 * @param  ctx: Sync context structure
 * @retval true if in holdover, false otherwise
 */
bool sync_is_in_holdover(sync_context_t *ctx);

/**
 * @brief  Reset synchronization state
 * @param  ctx: Sync context structure
 * @retval HAL status
 */
HAL_StatusTypeDef sync_reset(sync_context_t *ctx);

/**
 * @brief  Enable holdover mode
 * @param  ctx: Sync context structure
 * @retval HAL status
 */
HAL_StatusTypeDef sync_enable_holdover(sync_context_t *ctx);

/**
 * @brief  Disable holdover mode
 * @param  ctx: Sync context structure
 * @retval HAL status
 */
HAL_StatusTypeDef sync_disable_holdover(sync_context_t *ctx);

/**
 * @brief  Get synchronization statistics
 * @param  ctx: Sync context structure
 * @param  sync_count: Number of sync operations (output)
 * @param  adjustment_count: Number of adjustments made (output)
 * @param  average_offset: Average offset (output)
 * @retval HAL status
 */
HAL_StatusTypeDef sync_get_statistics(sync_context_t *ctx, uint32_t *sync_count,
                                     uint32_t *adjustment_count, int64_t *average_offset);

/* Algorithm-specific functions ---------------------------------------------*/

/**
 * @brief  Linear synchronization algorithm
 * @param  ctx: Sync context structure
 * @param  offset_ns: Measured offset
 * @param  timestamp_ms: Measurement timestamp
 * @retval Frequency adjustment in ppb
 */
int64_t sync_algorithm_linear(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms);

/**
 * @brief  PID synchronization algorithm
 * @param  ctx: Sync context structure
 * @param  offset_ns: Measured offset
 * @param  timestamp_ms: Measurement timestamp
 * @retval Frequency adjustment in ppb
 */
int64_t sync_algorithm_pid(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms);

/**
 * @brief  Kalman filter synchronization algorithm
 * @param  ctx: Sync context structure
 * @param  offset_ns: Measured offset
 * @param  timestamp_ms: Measurement timestamp
 * @retval Frequency adjustment in ppb
 */
int64_t sync_algorithm_kalman(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms);

/**
 * @brief  Adaptive synchronization algorithm
 * @param  ctx: Sync context structure
 * @param  offset_ns: Measured offset
 * @param  timestamp_ms: Measurement timestamp
 * @retval Frequency adjustment in ppb
 */
int64_t sync_algorithm_adaptive(sync_context_t *ctx, int64_t offset_ns, uint32_t timestamp_ms);

/* Utility functions --------------------------------------------------------*/

/**
 * @brief  Convert nanoseconds to frequency adjustment in ppb
 * @param  offset_ns: Time offset in nanoseconds
 * @param  interval_ms: Measurement interval in milliseconds
 * @retval Frequency adjustment in ppb
 */
int64_t ns_to_ppb(int64_t offset_ns, uint32_t interval_ms);

/**
 * @brief  Convert frequency adjustment in ppb to nanoseconds
 * @param  adjustment_ppb: Frequency adjustment in ppb
 * @param  interval_ms: Time interval in milliseconds
 * @retval Time offset in nanoseconds
 */
int64_t ppb_to_ns(int64_t adjustment_ppb, uint32_t interval_ms);

/**
 * @brief  Clamp value to specified range
 * @param  value: Input value
 * @param  min: Minimum value
 * @param  max: Maximum value
 * @retval Clamped value
 */
int64_t clamp_int64(int64_t value, int64_t min, int64_t max);

/**
 * @brief  Calculate moving average
 * @param  values: Array of values
 * @param  count: Number of values
 * @retval Average value
 */
int64_t calculate_moving_average(int64_t *values, uint32_t count);

/**
 * @brief  Calculate standard deviation
 * @param  values: Array of values
 * @param  count: Number of values
 * @param  mean: Mean value
 * @retval Standard deviation
 */
double calculate_standard_deviation(int64_t *values, uint32_t count, int64_t mean);

#ifdef __cplusplus
}
#endif

#endif /* __SYNC_ALGORITHM_H */
