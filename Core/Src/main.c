/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RXBUFFERSIZE  256U

/* UART command format:
 *   0\r\n              toggle periodic debug output
 *   1\r\n              red LED toggle
 *   2 <position>\r\n  3508 pull the slide to the trigger position
 *   3 <position>\r\n  lock the trigger, then return the 3508 to this position
 *   4\r\n              release the trigger and launch
 *   5 <position>\r\n  Yaw position
 *   6 <position>\r\n  M2006 position
 *   7\r\n              move M2006 to the mechanical stop and zero there
 */
#define UART_CMD_LOAD              0x01U
#define UART_CMD_DEBUG_TOGGLE      0x02U
#define UART_CMD_FIRE              0x04U
#define UART_CMD_SET_YAW           0x08U
#define UART_CMD_LED_TOGGLE        0x10U
#define UART_CMD_SET_TRIGGER       0x20U
#define UART_CMD_ZERO_TRIGGER      0x40U
#define UART_CMD_SET_RETURN        0x80U

/* DART launcher motor configuration copied from the source project. */
#define YAW_ANGLE_MIN              10000L
#define YAW_ANGLE_MAX              110000L
#define YAW_ANGLE_DEFAULT          40000L
#define LOAD_POSITION_MIN          (-100000L)
#define LOAD_POSITION_MAX          1070000L
#define LAUNCHER_LOAD_DEFAULT      367878L
#define LAUNCHER_HOME_DEFAULT      55000L

#define MOTOR_FEEDBACK_TIMEOUT_MS  500U
#define YAW_TARGET_TOLERANCE       1000L
#define YAW_VELOCITY_TOLERANCE     120.0f
#define LOAD_TARGET_TOLERANCE      5000L
#define TRIGGER_TARGET_TOLERANCE   10000L
#define TRIGGER_VELOCITY_TOLERANCE 100.0f
#define TARGET_VELOCITY_TOLERANCE  30.0f
#define LOAD_STALL_ERROR_THRESHOLD 50000L
#define LOAD_STALL_VELOCITY_THRESHOLD 100.0f
#define LOAD_STALL_DETECT_MS       200U
#define LOAD_STALL_BOOST_MS        500U
#define LOAD_STALL_MAX_ATTEMPTS    3U
#define LOAD_STALL_BOOST_CURRENT   16384
#define POSITION_STABLE_TIME_MS    100U
#define MOTION_TIMEOUT_MS          10000U
#define YAW_MOTION_TIMEOUT_MS      10000U
#define TRIGGER_MOTION_TIMEOUT_MS  120000U
#define SERVO_LOCK_WAIT_MS         1000U
#define FIRE_SERVO_DELAY_MS        1000U
#define FIRE_TRIGGER_TIME_MS       3000U
#define MOTOR_CONTROL_PERIOD_MS    1U
#define DEBUG_PRINT_PERIOD_MS      100U

/* Servo PWM: timer counter is 1 MHz, period is 10000 counts (100 Hz). */
#define SERVO_MIN_PULSE             500U
#define SERVO_MAX_PULSE             2500U
#define SERVO_MAX_ANGLE             180U
#define SERVO_LOCK_0                55U
#define SERVO_LOCK_1                0U
#define SERVO_AVOID_0               95U
#define SERVO_AVOID_1               35U

/* M2006 homing values copied from the source project. */
#define TRIGGER_HOME_VELOCITY       (-5000.0f)
#define TRIGGER_HOME_CURRENT_LIMIT  6000
#define TRIGGER_HOME_CURRENT_GATE   2000
#define TRIGGER_HOME_CONFIRM_MS     100U
#define TRIGGER_HOME_TIMEOUT_MS     120000U

#define UART7_RESPONSE_QUEUE_LEN    8U
#define UART7_RESPONSE_MAX_LEN      256U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t RxBuffer[RXBUFFERSIZE];
uint8_t aRxBuffer;
volatile uint16_t Uart7_Rx_Cnt;
volatile uint8_t g_command_flags;
volatile int32_t g_pending_yaw_target;
volatile int32_t g_pending_load_target;
volatile int32_t g_pending_return_target;
volatile int32_t g_pending_trigger_target;
static uint8_t g_yaw_command_waiting;
static uint8_t g_load_command_waiting;
static uint8_t g_load_command_number;
static uint8_t g_trigger_command_waiting;
static uint8_t g_trigger_homing_enabled;
static uint8_t g_trigger_position_ready;
static uint8_t g_trigger_zero_set;
static uint8_t g_trigger_control_enabled;
static int32_t g_trigger_zero_offset;
static volatile uint8_t g_uart7_tx_busy;
static uint8_t g_uart7_tx_buffer[UART7_RESPONSE_MAX_LEN];
static uint8_t g_uart7_response_queue[UART7_RESPONSE_QUEUE_LEN]
                                     [UART7_RESPONSE_MAX_LEN];
static uint8_t g_uart7_response_length[UART7_RESPONSE_QUEUE_LEN];
static volatile uint8_t g_uart7_response_head;
static volatile uint8_t g_uart7_response_tail;
static uint8_t g_debug_enabled;
static uint32_t g_debug_next_ms;
static int16_t g_debug_trigger_current;
static int16_t g_debug_load_current[2];
static int16_t g_debug_yaw_current;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct
{
  int16_t angle;
  int16_t velocity;
  int32_t angle_with_rounds;
  int16_t last_angle;
  float filtered_velocity;
  uint8_t first_feedback;
  uint8_t online;
  uint32_t last_update_ms;
} RM_MotorFeedback_t;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float integral;
  float last_error;
  float integral_limit;
  float p_limit;
  float i_limit;
  float output_limit;
} PID_t;

typedef enum
{
  LAUNCHER_IDLE = 0,
  LAUNCHER_LOADING,
  LAUNCHER_LOCKING,
  LAUNCHER_RETURNING,
  LAUNCHER_ARMED,
  LAUNCHER_FIRE_WAIT,
  LAUNCHER_FIRING,
  LAUNCHER_FAULT
} LauncherState_t;

static RM_MotorFeedback_t g_yaw_feedback;
static RM_MotorFeedback_t g_load_feedback[2];
static RM_MotorFeedback_t g_trigger_feedback;

static PID_t g_yaw_angle_pid;
static PID_t g_yaw_velocity_pid;
static PID_t g_load_angle_pid[2];
static PID_t g_load_velocity_pid[2];
static PID_t g_trigger_angle_pid;
static PID_t g_trigger_velocity_pid;

static int32_t g_yaw_target = YAW_ANGLE_DEFAULT;
static int32_t g_load_target = LAUNCHER_LOAD_DEFAULT;
static int32_t g_return_target = LAUNCHER_HOME_DEFAULT;
static int32_t g_trigger_target;
static uint8_t g_yaw_control_enabled;
static uint8_t g_load_control_enabled;
static int16_t g_last_load_current[2];
static uint32_t g_fire_start_ms;
static uint32_t g_lock_start_ms;
static uint32_t g_yaw_motion_start_ms;
static uint32_t g_yaw_stable_start_ms;
static uint32_t g_load_motion_start_ms;
static uint32_t g_load_stable_start_ms;
static uint32_t g_trigger_motion_start_ms;
static uint32_t g_trigger_stable_start_ms;
static uint32_t g_trigger_homing_start_ms;
static uint32_t g_trigger_homing_stable_start_ms;
static uint32_t g_load_stall_start_ms[2];
static uint32_t g_load_stall_boost_start_ms[2];
static uint8_t g_load_stall_boost_active[2];
static uint8_t g_load_stall_attempts[2];
static LauncherState_t g_launcher_state = LAUNCHER_IDLE;

static void launcher_init(void);
static void launcher_apply_commands(void);
static void launcher_control_step(void);
static void launcher_send_motor_currents(int16_t trigger_current,
                                         int16_t load0_current,
                                         int16_t load1_current,
                                         int16_t yaw_current);
static void load_stall_reset(void);
static int16_t load_apply_stall_boost(uint8_t motor_index,
                                      const RM_MotorFeedback_t *motor,
                                      int32_t target,
                                      int16_t normal_current,
                                      uint32_t now);
static void set_trigger_servo_angle(uint8_t servo_index, uint16_t angle);
static void parse_uart_command(const uint8_t *frame, uint16_t length);
static uint8_t is_ascii_space(uint8_t c);
static void uart7_queue_response(const char *message);
static void uart7_response_service(void);
static void uart7_debug_service(void);
static void decode_rm_feedback(RM_MotorFeedback_t *motor,
                               const uint8_t *data,
                               uint8_t reverse_angle);
static float pid_update(PID_t *pid, float target, float current);
static void pid_reset(PID_t *pid);
static int16_t position_controller(RM_MotorFeedback_t *motor,
                                   PID_t *angle_pid,
                                   PID_t *velocity_pid,
                                   int32_t target,
                                   int32_t position_offset,
                                   int16_t output_limit);
static uint8_t motor_is_online(const RM_MotorFeedback_t *motor,
                               uint32_t now);
static uint8_t motor_at_target(const RM_MotorFeedback_t *motor,
                               int32_t target,
                               int32_t tolerance,
                               int32_t position_offset);
static uint8_t motor_at_target_and_stopped(const RM_MotorFeedback_t *motor,
                                           int32_t target,
                                           int32_t tolerance,
                                           int32_t position_offset);
static uint8_t yaw_at_target_and_stopped(const RM_MotorFeedback_t *motor,
                                         int32_t target);
static uint8_t trigger_at_target_and_stopped(const RM_MotorFeedback_t *motor,
                                             int32_t target,
                                             int32_t position_offset);

static void can1_start_receive_all(void)
{
  CAN_FilterTypeDef filter = {0};

  /* CAN1 is shared by all launcher motors and other devices.  Receive every
   * standard/extended frame, then dispatch only the IDs we own in software. */
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1,
                                   CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

static void pid_config(PID_t *pid, float kp, float ki, float kd,
                       float integral_limit, float p_limit, float i_limit,
                       float output_limit)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->p_limit = p_limit;
  pid->i_limit = i_limit;
  pid->output_limit = output_limit;
  pid_reset(pid);
}

static void pid_reset(PID_t *pid)
{
  pid->integral = 0.0f;
  pid->last_error = 0.0f;
}

static float clamp_float(float value, float min_value, float max_value)
{
  if (value < min_value)
  {
    return min_value;
  }
  if (value > max_value)
  {
    return max_value;
  }
  return value;
}

static float pid_update(PID_t *pid, float target, float current)
{
  float error = target - current;
  float derivative;
  float output;

  pid->integral = clamp_float(pid->integral + error,
                              -pid->integral_limit,
                              pid->integral_limit);
  derivative = error - pid->last_error;
  pid->last_error = error;
  output = clamp_float(pid->kp * error, -pid->p_limit, pid->p_limit) +
           clamp_float(pid->ki * pid->integral,
                       -pid->i_limit, pid->i_limit) +
           pid->kd * derivative;
  return clamp_float(output, -pid->output_limit, pid->output_limit);
}

static uint8_t motor_is_online(const RM_MotorFeedback_t *motor,
                               uint32_t now)
{
  return (uint8_t)(motor->online &&
                   ((uint32_t)(now - motor->last_update_ms) <=
                    MOTOR_FEEDBACK_TIMEOUT_MS));
}

static uint8_t motor_at_target(const RM_MotorFeedback_t *motor,
                               int32_t target,
                               int32_t tolerance,
                               int32_t position_offset)
{
  int32_t error = motor->angle_with_rounds - position_offset - target;
  if (error < 0)
  {
    error = -error;
  }
  return (uint8_t)(error <= tolerance);
}

static uint8_t motor_at_target_and_stopped(const RM_MotorFeedback_t *motor,
                                           int32_t target,
                                           int32_t tolerance,
                                           int32_t position_offset)
{
  return (uint8_t)(motor_at_target(motor, target, tolerance, position_offset) &&
                   (motor->filtered_velocity <= TARGET_VELOCITY_TOLERANCE) &&
                   (motor->filtered_velocity >= -TARGET_VELOCITY_TOLERANCE));
}

static uint8_t yaw_at_target_and_stopped(const RM_MotorFeedback_t *motor,
                                         int32_t target)
{
  return (uint8_t)(motor_at_target(motor, target, YAW_TARGET_TOLERANCE, 0) &&
                   (motor->filtered_velocity <= YAW_VELOCITY_TOLERANCE) &&
                   (motor->filtered_velocity >= -YAW_VELOCITY_TOLERANCE));
}

static uint8_t trigger_at_target_and_stopped(const RM_MotorFeedback_t *motor,
                                             int32_t target,
                                             int32_t position_offset)
{
  return (uint8_t)(motor_at_target(motor, target,
                                   TRIGGER_TARGET_TOLERANCE,
                                   position_offset) &&
                   (motor->filtered_velocity <= TRIGGER_VELOCITY_TOLERANCE) &&
                   (motor->filtered_velocity >= -TRIGGER_VELOCITY_TOLERANCE));
}

static void load_stall_reset(void)
{
  g_load_stall_start_ms[0] = 0U;
  g_load_stall_start_ms[1] = 0U;
  g_load_stall_boost_start_ms[0] = 0U;
  g_load_stall_boost_start_ms[1] = 0U;
  g_load_stall_boost_active[0] = 0U;
  g_load_stall_boost_active[1] = 0U;
  g_load_stall_attempts[0] = 0U;
  g_load_stall_attempts[1] = 0U;
}

static int16_t load_apply_stall_boost(uint8_t motor_index,
                                      const RM_MotorFeedback_t *motor,
                                      int32_t target,
                                      int16_t normal_current,
                                      uint32_t now)
{
  int64_t position_error = (int64_t)target -
                            (int64_t)motor->angle_with_rounds;
  uint8_t stalled;

  if (position_error < 0)
  {
    position_error = -position_error;
  }

  stalled = (uint8_t)((position_error > LOAD_STALL_ERROR_THRESHOLD) &&
                      (motor->filtered_velocity <=
                       LOAD_STALL_VELOCITY_THRESHOLD) &&
                      (motor->filtered_velocity >=
                       -LOAD_STALL_VELOCITY_THRESHOLD) &&
                      (normal_current != 0));

  if (g_load_stall_boost_active[motor_index] != 0U)
  {
    /* Stop the boost early if the motor has started moving or reached the
     * position-error threshold.  Otherwise limit one boost to 500 ms. */
    if ((!stalled) ||
        ((uint32_t)(now - g_load_stall_boost_start_ms[motor_index]) >=
         LOAD_STALL_BOOST_MS))
    {
      g_load_stall_boost_active[motor_index] = 0U;
      g_load_stall_boost_start_ms[motor_index] = 0U;
      return normal_current;
    }

    return (normal_current > 0) ? LOAD_STALL_BOOST_CURRENT :
                                  (int16_t)-LOAD_STALL_BOOST_CURRENT;
  }

  if (!stalled ||
      (g_load_stall_attempts[motor_index] >= LOAD_STALL_MAX_ATTEMPTS))
  {
    g_load_stall_start_ms[motor_index] = 0U;
    return normal_current;
  }

  if (g_load_stall_start_ms[motor_index] == 0U)
  {
    g_load_stall_start_ms[motor_index] = now;
  }
  else if ((uint32_t)(now - g_load_stall_start_ms[motor_index]) >=
           LOAD_STALL_DETECT_MS)
  {
    g_load_stall_attempts[motor_index]++;
    g_load_stall_boost_active[motor_index] = 1U;
    g_load_stall_boost_start_ms[motor_index] = now;
    g_load_stall_start_ms[motor_index] = 0U;
    return (normal_current > 0) ? LOAD_STALL_BOOST_CURRENT :
                                  (int16_t)-LOAD_STALL_BOOST_CURRENT;
  }

  return normal_current;
}

static int16_t position_controller(RM_MotorFeedback_t *motor,
                                   PID_t *angle_pid,
                                   PID_t *velocity_pid,
                                   int32_t target,
                                   int32_t position_offset,
                                   int16_t output_limit)
{
  float velocity_target;
  float output;

  if (!motor->online)
  {
    pid_reset(angle_pid);
    pid_reset(velocity_pid);
    return 0;
  }

  velocity_target = pid_update(angle_pid,
                               (float)target,
                               (float)motor->angle_with_rounds -
                                   (float)position_offset);
  /* The angle PID output limit is the velocity target limit in the source
   * project.  Do not use one fixed limit for all motor types. */
  velocity_target = clamp_float(velocity_target,
                                -angle_pid->output_limit,
                                angle_pid->output_limit);
  output = pid_update(velocity_pid,
                      velocity_target,
                      motor->filtered_velocity);
  output = clamp_float(output, -(float)output_limit, (float)output_limit);
  return (int16_t)output;
}

static void decode_rm_feedback(RM_MotorFeedback_t *motor,
                               const uint8_t *data,
                               uint8_t reverse_angle)
{
  int16_t raw_angle = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
  int16_t raw_velocity = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  uint32_t now = HAL_GetTick();

  if (reverse_angle)
  {
    raw_angle = (int16_t)(8191 - raw_angle);
    raw_velocity = (int16_t)(-raw_velocity);
  }

  if (!motor->first_feedback)
  {
    motor->angle = raw_angle;
    motor->last_angle = raw_angle;
    motor->angle_with_rounds = raw_angle;
    motor->filtered_velocity = (float)raw_velocity;
    motor->first_feedback = 1U;
  }
  else
  {
    if ((raw_angle < 2048) && (motor->last_angle > 6144))
    {
      motor->angle_with_rounds += 8192;
    }
    else if ((raw_angle > 6144) && (motor->last_angle < 2048))
    {
      motor->angle_with_rounds -= 8192;
    }
    motor->angle = raw_angle;
    motor->last_angle = raw_angle;
    motor->filtered_velocity = (float)raw_velocity * 0.6f +
                                motor->filtered_velocity * 0.4f;
  }

  motor->velocity = raw_velocity;
  motor->last_update_ms = now;
  motor->online = 1U;
}

static uint32_t servo_compare_from_angle(uint16_t angle)
{
  if (angle > SERVO_MAX_ANGLE)
  {
    angle = SERVO_MAX_ANGLE;
  }
  return SERVO_MIN_PULSE +
         ((uint32_t)angle * (SERVO_MAX_PULSE - SERVO_MIN_PULSE)) /
             SERVO_MAX_ANGLE;
}

static void set_trigger_servo_angle(uint8_t servo_index, uint16_t angle)
{
  if (servo_index == 0U)
  {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
                          servo_compare_from_angle(angle));
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,
                          servo_compare_from_angle(angle));
  }
}

static uint8_t is_ascii_space(uint8_t c)
{
  return (uint8_t)(c == (uint8_t)' ' || c == (uint8_t)'\t');
}

static void uart7_queue_response(const char *message)
{
  uint8_t head = g_uart7_response_head;
  uint8_t next = (uint8_t)((head + 1U) % UART7_RESPONSE_QUEUE_LEN);
  uint16_t length = (uint16_t)strlen(message);

  if (next == g_uart7_response_tail)
  {
    return;
  }
  if (length >= UART7_RESPONSE_MAX_LEN)
  {
    length = UART7_RESPONSE_MAX_LEN - 1U;
  }
  memcpy(g_uart7_response_queue[head], message, length);
  g_uart7_response_queue[head][length] = 0U;
  g_uart7_response_length[head] = (uint8_t)length;
  g_uart7_response_head = next;
}

static void uart7_response_service(void)
{
  uint8_t tail;
  uint8_t length;

  if (g_uart7_tx_busy != 0U)
  {
    return;
  }
  tail = g_uart7_response_tail;
  if (tail == g_uart7_response_head)
  {
    uart7_debug_service();
    return;
  }
  length = g_uart7_response_length[tail];
  memcpy(g_uart7_tx_buffer, g_uart7_response_queue[tail], length);
  if (HAL_UART_Transmit_IT(&huart7, g_uart7_tx_buffer, length) == HAL_OK)
  {
    g_uart7_response_tail =
        (uint8_t)((tail + 1U) % UART7_RESPONSE_QUEUE_LEN);
    g_uart7_tx_busy = 1U;
  }
}

static void uart7_debug_service(void)
{
  uint32_t now = HAL_GetTick();
  int length;

  if ((g_debug_enabled == 0U) ||
      ((int32_t)(now - g_debug_next_ms) < 0))
  {
    return;
  }

  /* Keep all values integral in the diagnostic line.  This avoids the large
   * printf floating-point implementation and keeps UART7 responsive. */
  length = snprintf((char *)g_uart7_tx_buffer, sizeof(g_uart7_tx_buffer),
                    "DBG S=%u Y=%u L=%u T=%u H=%u | "
                    "Y a=%ld t=%ld v=%ld i=%d on=%u | "
                    "L0 a=%ld t=%ld v=%ld i=%d on=%u | "
                    "L1 a=%ld t=%ld v=%ld i=%d on=%u | "
                    "M a=%ld t=%ld v=%ld i=%d on=%u\r\n",
                    (unsigned)g_launcher_state,
                    (unsigned)g_yaw_control_enabled,
                    (unsigned)g_load_control_enabled,
                    (unsigned)g_trigger_control_enabled,
                    (unsigned)g_trigger_homing_enabled,
                    (long)g_yaw_feedback.angle_with_rounds,
                    (long)g_yaw_target,
                    (long)g_yaw_feedback.filtered_velocity,
                    (int)g_debug_yaw_current,
                    (unsigned)g_yaw_feedback.online,
                    (long)g_load_feedback[0].angle_with_rounds,
                    (long)g_load_target,
                    (long)g_load_feedback[0].filtered_velocity,
                    (int)g_debug_load_current[0],
                    (unsigned)g_load_feedback[0].online,
                    (long)g_load_feedback[1].angle_with_rounds,
                    (long)g_load_target,
                    (long)g_load_feedback[1].filtered_velocity,
                    (int)g_debug_load_current[1],
                    (unsigned)g_load_feedback[1].online,
                    (long)((int64_t)g_trigger_feedback.angle_with_rounds -
                           (int64_t)g_trigger_zero_offset),
                    (long)g_trigger_target,
                    (long)g_trigger_feedback.filtered_velocity,
                    (int)g_debug_trigger_current,
                    (unsigned)g_trigger_feedback.online);
  if (length <= 0)
  {
    return;
  }
  if ((uint32_t)length >= sizeof(g_uart7_tx_buffer))
  {
    length = (int)(sizeof(g_uart7_tx_buffer) - 1U);
  }

  if (HAL_UART_Transmit_IT(&huart7, g_uart7_tx_buffer,
                           (uint16_t)length) == HAL_OK)
  {
    g_uart7_tx_busy = 1U;
    g_debug_next_ms = now + DEBUG_PRINT_PERIOD_MS;
  }
}

static uint8_t parse_uint32_parameter(const uint8_t *frame,
                                      uint16_t length,
                                      uint16_t *index,
                                      uint32_t maximum,
                                      uint32_t *value)
{
  uint32_t digit;
  uint8_t digit_found = 0U;
  uint8_t separator_found = 0U;

  while ((*index < length) && is_ascii_space(frame[*index]))
  {
    separator_found = 1U;
    (*index)++;
  }
  if (!separator_found)
  {
    return 0U;
  }
  while ((*index < length) && (frame[*index] >= (uint8_t)'0') &&
         (frame[*index] <= (uint8_t)'9'))
  {
    digit = (uint32_t)(frame[*index] - (uint8_t)'0');
    if ((*value > (maximum / 10U)) ||
        ((*value == (maximum / 10U)) && (digit > (maximum % 10U))))
    {
      return 0U;
    }
    *value = *value * 10U + digit;
    digit_found = 1U;
    (*index)++;
  }
  while ((*index < length) && is_ascii_space(frame[*index]))
  {
    (*index)++;
  }
  return (uint8_t)(digit_found && (*index == length));
}

static uint8_t parse_int32_parameter(const uint8_t *frame,
                                     uint16_t length,
                                     uint16_t *index,
                                     int32_t *value)
{
  uint32_t magnitude = 0U;
  uint32_t digit;
  uint32_t maximum;
  uint8_t negative = 0U;
  uint8_t digit_found = 0U;
  uint8_t separator_found = 0U;

  while ((*index < length) && is_ascii_space(frame[*index]))
  {
    separator_found = 1U;
    (*index)++;
  }
  if (!separator_found)
  {
    return 0U;
  }

  if ((*index < length) && (frame[*index] == (uint8_t)'-'))
  {
    negative = 1U;
    (*index)++;
  }
  else if ((*index < length) && (frame[*index] == (uint8_t)'+'))
  {
    (*index)++;
  }

  maximum = negative ? 2147483648UL : 2147483647UL;
  while ((*index < length) && (frame[*index] >= (uint8_t)'0') &&
         (frame[*index] <= (uint8_t)'9'))
  {
    digit = (uint32_t)(frame[*index] - (uint8_t)'0');
    if ((magnitude > (maximum / 10U)) ||
        ((magnitude == (maximum / 10U)) &&
         (digit > (maximum % 10U))))
    {
      return 0U;
    }
    magnitude = magnitude * 10U + digit;
    digit_found = 1U;
    (*index)++;
  }
  while ((*index < length) && is_ascii_space(frame[*index]))
  {
    (*index)++;
  }
  if (!digit_found || (*index != length))
  {
    return 0U;
  }

  if (negative)
  {
    if (magnitude == 2147483648UL)
    {
      *value = (-2147483647L - 1L);
    }
    else
    {
      *value = -(int32_t)magnitude;
    }
  }
  else
  {
    *value = (int32_t)magnitude;
  }
  return 1U;
}

static uint8_t parse_command_without_parameter(const uint8_t *frame,
                                               uint16_t length,
                                               uint16_t index)
{
  while ((index < length) && is_ascii_space(frame[index]))
  {
    index++;
  }
  return (uint8_t)(index == length);
}

static void parse_uart_command(const uint8_t *frame, uint16_t length)
{
  uint16_t index = 0U;
  uint32_t value = 0U;
  int32_t signed_value;
  uint8_t command;

  while ((index < length) && is_ascii_space(frame[index]))
  {
    index++;
  }
  if (index >= length)
  {
    return;
  }

  command = frame[index++];
  if (command == (uint8_t)'0')
  {
    if (parse_command_without_parameter(frame, length, index))
    {
      g_command_flags |= UART_CMD_DEBUG_TOGGLE;
    }
  }
  else if (command == (uint8_t)'1')
  {
    if (parse_command_without_parameter(frame, length, index))
    {
      g_command_flags |= UART_CMD_LED_TOGGLE;
    }
  }
  else if (command == (uint8_t)'2')
  {
    if (parse_int32_parameter(frame, length, &index, &signed_value) &&
        (signed_value >= LOAD_POSITION_MIN) &&
        (signed_value <= LOAD_POSITION_MAX))
    {
      g_pending_load_target = signed_value;
      g_command_flags |= UART_CMD_LOAD;
    }
  }
  else if (command == (uint8_t)'3')
  {
    if (parse_int32_parameter(frame, length, &index, &signed_value) &&
        (signed_value >= LOAD_POSITION_MIN) &&
        (signed_value <= LOAD_POSITION_MAX))
    {
      g_pending_return_target = signed_value;
      g_command_flags |= UART_CMD_SET_RETURN;
    }
  }
  else if (command == (uint8_t)'4')
  {
    if (parse_command_without_parameter(frame, length, index))
    {
      g_command_flags |= UART_CMD_FIRE;
    }
  }
  else if (command == (uint8_t)'5')
  {
    if (parse_uint32_parameter(frame, length, &index,
                               (uint32_t)YAW_ANGLE_MAX, &value) &&
        (value >= (uint32_t)YAW_ANGLE_MIN))
    {
      g_pending_yaw_target = (int32_t)value;
      g_command_flags |= UART_CMD_SET_YAW;
    }
  }
  else if (command == (uint8_t)'6')
  {
    if (parse_int32_parameter(frame, length, &index, &signed_value))
    {
      g_pending_trigger_target = signed_value;
      g_command_flags |= UART_CMD_SET_TRIGGER;
    }
  }
  else if (command == (uint8_t)'7')
  {
    if (parse_command_without_parameter(frame, length, index))
    {
      g_command_flags |= UART_CMD_ZERO_TRIGGER;
    }
  }
}

static void launcher_init(void)
{
  /* The source values are a useful starting point, but are too aggressive
   * for this bare-metal position loop and make Yaw hunt around the target.
   * Reduce the velocity-loop integral/derivative and the outer-loop gain. */
  pid_config(&g_yaw_velocity_pid, 10.0f, 0.1f, 0.05f,
             360000.0f, 4300.0f, 13000.0f, 16384.0f);
  pid_config(&g_yaw_angle_pid, 0.25f, 0.0f, 0.005f,
             200000.0f, 10000.0f, 200.0f, 300.0f);

  pid_config(&g_load_velocity_pid[0], 25.0f, 0.1f, 0.01f,
             100000.0f, 16384.0f, 6000.0f, 16384.0f);
  pid_config(&g_load_velocity_pid[1], 25.0f, 0.1f, 0.01f,
             100000.0f, 16384.0f, 6000.0f, 16384.0f);
  /* Allow the 3508 position loop to request the source project's loading
   * speed.  The inner velocity loop still limits the CAN current to 16384. */
  pid_config(&g_load_angle_pid[0], 0.1f, 0.01f, 0.01f,
             100.0f, 4000.0f, 100.0f, 4000.0f);
  pid_config(&g_load_angle_pid[1], 0.1f, 0.01f, 0.01f,
             100.0f, 4000.0f, 100.0f, 4000.0f);

  /* M2006/trigger uses a gentler position loop than the source project.
   * Its feedback and current directions are reversed in the CAN callback /
   * send routine below.  The smaller integral and derivative terms prevent
   * repeated corrections around the target. */
  pid_config(&g_trigger_velocity_pid, 8.0f, 0.05f, 0.005f,
             10000.0f, 5000.0f, 5000.0f, 16384.0f);
  pid_config(&g_trigger_angle_pid, 0.06f, 0.0f, 0.005f,
             100.0f, 10000.0f, 10000.0f, 8000.0f);

  g_yaw_target = YAW_ANGLE_DEFAULT;
  g_load_target = LAUNCHER_LOAD_DEFAULT;
  g_return_target = LAUNCHER_HOME_DEFAULT;
  g_trigger_target = 0;
  g_yaw_control_enabled = 0U;
  g_load_control_enabled = 0U;
  g_yaw_command_waiting = 0U;
  g_load_command_waiting = 0U;
  g_load_command_number = 0U;
  g_trigger_homing_enabled = 0U;
  g_trigger_command_waiting = 0U;
  g_trigger_position_ready = 0U;
  g_trigger_zero_set = 0U;
  g_trigger_control_enabled = 0U;
  g_trigger_zero_offset = 0;
  g_trigger_homing_start_ms = 0U;
  g_trigger_homing_stable_start_ms = 0U;
  load_stall_reset();
  g_last_load_current[0] = 0;
  g_last_load_current[1] = 0;
  g_uart7_tx_busy = 0U;
  g_uart7_response_head = 0U;
  g_uart7_response_tail = 0U;
  g_debug_enabled = 0U;
  g_debug_next_ms = 0U;
  g_debug_trigger_current = 0;
  g_debug_load_current[0] = 0;
  g_debug_load_current[1] = 0;
  g_debug_yaw_current = 0;
  g_launcher_state = LAUNCHER_IDLE;

  /* Start with the trigger locked.  Command 2 explicitly moves it to the
   * avoid position before the slide starts moving. */
  set_trigger_servo_angle(0U, SERVO_LOCK_0);
  set_trigger_servo_angle(1U, SERVO_LOCK_1);
  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void launcher_apply_commands(void)
{
  uint8_t flags;
  int32_t yaw_target;
  int32_t load_target;
  int32_t return_target;
  int32_t trigger_target;
  uint32_t now = HAL_GetTick();

  __disable_irq();
  flags = g_command_flags;
  yaw_target = g_pending_yaw_target;
  load_target = g_pending_load_target;
  return_target = g_pending_return_target;
  trigger_target = g_pending_trigger_target;
  g_command_flags = 0U;
  __enable_irq();

  if ((flags & UART_CMD_DEBUG_TOGGLE) != 0U)
  {
    g_debug_enabled ^= 1U;
    g_debug_next_ms = now;
    if (g_debug_enabled != 0U)
    {
      uart7_queue_response("CMD 0 DEBUG ON\r\n");
    }
    else
    {
      uart7_queue_response("CMD 0 DEBUG OFF\r\n");
    }
  }

  if ((flags & UART_CMD_LED_TOGGLE) != 0U)
  {
    /* Command 1 preserves the previous red LED toggle function. */
    HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
    uart7_queue_response("CMD 1 DONE\r\n");
  }

  if ((flags & UART_CMD_ZERO_TRIGGER) != 0U)
  {
    if (g_trigger_homing_enabled)
    {
      /* Ignore a repeated homing command while homing is already active. */
    }
    else if (((g_launcher_state == LAUNCHER_IDLE) ||
         (g_launcher_state == LAUNCHER_FAULT)) &&
        motor_is_online(&g_trigger_feedback, now))
    {
      if (g_trigger_command_waiting != 0U)
      {
        g_trigger_command_waiting = 0U;
        uart7_queue_response("CMD 6 CANCELED\r\n");
      }
      /* Match the source project: run toward the mechanical stop in velocity
       * mode, then zero the multi-turn position after a current stall is
       * detected.  CMD 7 is acknowledged only after that operation finishes. */
      g_trigger_homing_enabled = 1U;
      g_trigger_homing_start_ms = now;
      g_trigger_homing_stable_start_ms = 0U;
      g_trigger_zero_set = 0U;
      g_trigger_position_ready = 0U;
      g_trigger_target = 0;
      g_trigger_control_enabled = 0U;
      g_trigger_command_waiting = 0U;
      pid_reset(&g_trigger_angle_pid);
      pid_reset(&g_trigger_velocity_pid);
    }
    else
    {
      uart7_queue_response("CMD 7 NOT READY\r\n");
    }
  }

  if ((flags & UART_CMD_SET_YAW) != 0U)
  {
    if (g_yaw_command_waiting != 0U)
    {
      uart7_queue_response("CMD 5 CANCELED\r\n");
    }
    g_yaw_target = yaw_target;
    g_yaw_control_enabled = 1U;
    g_yaw_command_waiting = 1U;
    g_yaw_motion_start_ms = now;
    g_yaw_stable_start_ms = 0U;
    pid_reset(&g_yaw_angle_pid);
    pid_reset(&g_yaw_velocity_pid);
  }

  if ((flags & UART_CMD_SET_TRIGGER) != 0U)
  {
    if (g_trigger_homing_enabled)
    {
      /* CMD 6 is intentionally ignored while CMD 7 is homing. */
    }
    else if (g_trigger_zero_set &&
        ((g_launcher_state == LAUNCHER_IDLE) ||
         (g_launcher_state == LAUNCHER_FAULT)) &&
        motor_is_online(&g_trigger_feedback, now))
    {
      if (g_trigger_command_waiting != 0U)
      {
        uart7_queue_response("CMD 6 CANCELED\r\n");
      }
      g_trigger_target = trigger_target;
      g_trigger_control_enabled = 1U;
      g_trigger_command_waiting = 1U;
      g_trigger_position_ready = 0U;
      g_trigger_motion_start_ms = now;
      g_trigger_stable_start_ms = 0U;
      pid_reset(&g_trigger_angle_pid);
      pid_reset(&g_trigger_velocity_pid);
      g_launcher_state = LAUNCHER_IDLE;
    }
    else
    {
      uart7_queue_response("CMD 6 NOT READY\r\n");
    }
  }

  if ((flags & UART_CMD_LOAD) != 0U)
  {
    if (g_trigger_position_ready &&
        ((g_launcher_state == LAUNCHER_IDLE) ||
         (g_launcher_state == LAUNCHER_LOADING) ||
         (g_launcher_state == LAUNCHER_LOCKING) ||
         (g_launcher_state == LAUNCHER_RETURNING) ||
         (g_launcher_state == LAUNCHER_ARMED) ||
         (g_launcher_state == LAUNCHER_FIRE_WAIT) ||
         (g_launcher_state == LAUNCHER_FIRING) ||
         (g_launcher_state == LAUNCHER_FAULT)))
    {
      if (g_load_command_waiting != 0U)
      {
        if (g_load_command_number == 2U)
        {
          uart7_queue_response("CMD 2 CANCELED\r\n");
        }
        else
        {
          uart7_queue_response("CMD 3 CANCELED\r\n");
        }
      }
      g_load_target = load_target;
      g_load_control_enabled = 1U;
      g_load_command_waiting = 1U;
      g_load_command_number = 2U;
      g_load_motion_start_ms = now;
      g_load_stable_start_ms = 0U;
      g_last_load_current[0] = 0;
      g_last_load_current[1] = 0;
      load_stall_reset();
      g_launcher_state = LAUNCHER_LOADING;
      set_trigger_servo_angle(0U, SERVO_AVOID_0);
      set_trigger_servo_angle(1U, SERVO_AVOID_1);
      pid_reset(&g_load_angle_pid[0]);
      pid_reset(&g_load_angle_pid[1]);
      pid_reset(&g_load_velocity_pid[0]);
      pid_reset(&g_load_velocity_pid[1]);
    }
    else
    {
      uart7_queue_response("CMD 2 NOT READY\r\n");
    }
  }

  if ((flags & UART_CMD_SET_RETURN) != 0U)
  {
    if ((g_launcher_state == LAUNCHER_LOADING) ||
        (g_launcher_state == LAUNCHER_LOCKING) ||
        (g_launcher_state == LAUNCHER_RETURNING) ||
        (g_launcher_state == LAUNCHER_FAULT))
    {
      if (g_load_command_waiting != 0U)
      {
        if (g_load_command_number == 2U)
        {
          uart7_queue_response("CMD 2 CANCELED\r\n");
        }
        else
        {
          uart7_queue_response("CMD 3 CANCELED\r\n");
        }
      }
      g_return_target = return_target;
      /* The operator is responsible for confirming that cmd 2 has reached
       * the desired trigger position.  Cmd 3 therefore starts immediately;
       * it must not wait for cmd 2's position-complete condition.  A new cmd
       * 3 also replaces an active lock/return operation. */
      g_load_command_waiting = 1U;
      g_load_command_number = 3U;
      g_load_motion_start_ms = now;
      g_load_stable_start_ms = 0U;
      g_lock_start_ms = now;
      g_launcher_state = LAUNCHER_LOCKING;
      set_trigger_servo_angle(0U, SERVO_LOCK_0);
      set_trigger_servo_angle(1U, SERVO_LOCK_1);
    }
    else
    {
      uart7_queue_response("CMD 3 NOT READY\r\n");
    }
  }

  if ((flags & UART_CMD_FIRE) != 0U)
  {
    if ((g_launcher_state == LAUNCHER_ARMED) ||
        (g_launcher_state == LAUNCHER_RETURNING) ||
        (g_launcher_state == LAUNCHER_FAULT))
    {
      /* The operator may manually confirm the return position.  Therefore
       * cmd 4 is also accepted during return or after a position timeout. */
      g_load_control_enabled = 0U;
      g_load_command_waiting = 0U;
      g_last_load_current[0] = 0;
      g_last_load_current[1] = 0;
      g_fire_start_ms = now;
      g_launcher_state = LAUNCHER_FIRE_WAIT;
    }
    else
    {
      uart7_queue_response("CMD 4 NOT READY\r\n");
    }
  }
}

static void launcher_control_step(void)
{
  uint32_t now = HAL_GetTick();
  int16_t trigger_current = 0;
  int16_t trigger_homing_current = 0;
  int16_t yaw_current = 0;
  int16_t load0_current = 0;
  int16_t load1_current = 0;
  uint8_t yaw_stable;
  uint8_t load_stable;
  uint8_t trigger_stable;

  /* Once Yaw is close and slow, stop applying alternating corrections.  Yaw
   * has its own wider velocity deadband because the encoder velocity
   * feedback is noisy near zero; the tighter generic threshold is retained
   * for the launcher mechanisms. */
  yaw_stable = (uint8_t)(motor_is_online(&g_yaw_feedback, now) &&
                         yaw_at_target_and_stopped(&g_yaw_feedback,
                                                   g_yaw_target));
  if (g_yaw_control_enabled && motor_is_online(&g_yaw_feedback, now))
  {
    if (yaw_stable)
    {
      pid_reset(&g_yaw_angle_pid);
      pid_reset(&g_yaw_velocity_pid);
    }
    else
    {
      yaw_current = position_controller(&g_yaw_feedback,
                                         &g_yaw_angle_pid,
                                         &g_yaw_velocity_pid,
                                         g_yaw_target,
                                         0,
                                         16384);
    }
  }
  else
  {
    pid_reset(&g_yaw_angle_pid);
    pid_reset(&g_yaw_velocity_pid);
  }

  if (g_yaw_command_waiting != 0U)
  {
    if (yaw_stable)
    {
      if (g_yaw_stable_start_ms == 0U)
      {
        g_yaw_stable_start_ms = now;
      }
      else if ((uint32_t)(now - g_yaw_stable_start_ms) >=
               POSITION_STABLE_TIME_MS)
      {
        g_yaw_command_waiting = 0U;
        uart7_queue_response("CMD 5 DONE\r\n");
      }
    }
    else
    {
      g_yaw_stable_start_ms = 0U;
    }
    if ((g_yaw_command_waiting != 0U) &&
        ((uint32_t)(now - g_yaw_motion_start_ms) >= YAW_MOTION_TIMEOUT_MS))
    {
      g_yaw_command_waiting = 0U;
      g_yaw_control_enabled = 0U;
      g_yaw_stable_start_ms = 0U;
      pid_reset(&g_yaw_angle_pid);
      pid_reset(&g_yaw_velocity_pid);
      uart7_queue_response("CMD 5 TIMEOUT\r\n");
    }
  }

  /* M2006 uses the source project's reverse angle/velocity/current
   * convention.  Command 7 has priority and performs the source project's
   * velocity-to-stall homing before command 6 position control is allowed. */
  trigger_stable = (uint8_t)(g_trigger_zero_set &&
                             motor_is_online(&g_trigger_feedback, now) &&
                             trigger_at_target_and_stopped(&g_trigger_feedback,
                                                           g_trigger_target,
                                                           g_trigger_zero_offset));
  if (g_trigger_homing_enabled)
  {
    if (motor_is_online(&g_trigger_feedback, now))
    {
      trigger_homing_current = (int16_t)clamp_float(
          pid_update(&g_trigger_velocity_pid, TRIGGER_HOME_VELOCITY,
                     g_trigger_feedback.filtered_velocity),
          -(float)TRIGGER_HOME_CURRENT_LIMIT,
          (float)TRIGGER_HOME_CURRENT_LIMIT);
      trigger_current = (int16_t)-trigger_homing_current;

      if ((trigger_homing_current >= TRIGGER_HOME_CURRENT_GATE) ||
          (trigger_homing_current <= -TRIGGER_HOME_CURRENT_GATE))
      {
        if (g_trigger_homing_stable_start_ms == 0U)
        {
          g_trigger_homing_stable_start_ms = now;
        }
        else if ((uint32_t)(now - g_trigger_homing_stable_start_ms) >=
                 TRIGGER_HOME_CONFIRM_MS)
        {
          g_trigger_homing_enabled = 0U;
          g_trigger_zero_offset = g_trigger_feedback.angle_with_rounds;
          g_trigger_zero_set = 1U;
          g_trigger_position_ready = 0U;
          g_trigger_target = 0;
          g_trigger_control_enabled = 0U;
          g_trigger_homing_stable_start_ms = 0U;
          trigger_current = 0;
          pid_reset(&g_trigger_velocity_pid);
          uart7_queue_response("CMD 7 DONE\r\n");
        }
      }
      else
      {
        g_trigger_homing_stable_start_ms = 0U;
      }
    }
    else
    {
      pid_reset(&g_trigger_velocity_pid);
    }

    if (g_trigger_homing_enabled &&
         ((uint32_t)(now - g_trigger_homing_start_ms) >=
         TRIGGER_HOME_TIMEOUT_MS))
    {
      g_trigger_homing_enabled = 0U;
      g_trigger_homing_stable_start_ms = 0U;
      g_trigger_zero_set = 0U;
      g_trigger_position_ready = 0U;
      trigger_current = 0;
      pid_reset(&g_trigger_velocity_pid);
      uart7_queue_response("CMD 7 TIMEOUT\r\n");
    }
  }
  else if (g_trigger_control_enabled && g_trigger_zero_set &&
      motor_is_online(&g_trigger_feedback, now))
  {
    if (trigger_stable)
    {
      pid_reset(&g_trigger_angle_pid);
      pid_reset(&g_trigger_velocity_pid);
    }
    else
    {
      trigger_current = (int16_t)-position_controller(&g_trigger_feedback,
                                                        &g_trigger_angle_pid,
                                                        &g_trigger_velocity_pid,
                                                        g_trigger_target,
                                                        g_trigger_zero_offset,
                                                        16384);
    }
  }
  else
  {
    pid_reset(&g_trigger_angle_pid);
    pid_reset(&g_trigger_velocity_pid);
  }

  if (g_trigger_command_waiting != 0U)
  {
    if (trigger_stable)
    {
      if (g_trigger_stable_start_ms == 0U)
      {
        g_trigger_stable_start_ms = now;
      }
      else if ((uint32_t)(now - g_trigger_stable_start_ms) >=
               POSITION_STABLE_TIME_MS)
      {
        g_trigger_command_waiting = 0U;
        g_trigger_position_ready = 1U;
        uart7_queue_response("CMD 6 DONE\r\n");
      }
    }
    else
    {
      g_trigger_stable_start_ms = 0U;
    }
    if ((g_trigger_command_waiting != 0U) &&
        ((uint32_t)(now - g_trigger_motion_start_ms) >=
         TRIGGER_MOTION_TIMEOUT_MS))
    {
      g_trigger_command_waiting = 0U;
      g_trigger_position_ready = 0U;
      g_trigger_control_enabled = 0U;
      g_trigger_stable_start_ms = 0U;
      pid_reset(&g_trigger_angle_pid);
      pid_reset(&g_trigger_velocity_pid);
      uart7_queue_response("CMD 6 TIMEOUT\r\n");
    }
  }

  /* cmd 3 first holds the trigger for a full second, then starts the 3508
   * return motion using the position supplied with cmd 3. */
  if ((g_launcher_state == LAUNCHER_LOCKING) &&
      ((uint32_t)(now - g_lock_start_ms) >= SERVO_LOCK_WAIT_MS))
  {
    g_load_target = g_return_target;
    g_load_motion_start_ms = now;
    g_load_stable_start_ms = 0U;
    pid_reset(&g_load_angle_pid[0]);
    pid_reset(&g_load_angle_pid[1]);
    pid_reset(&g_load_velocity_pid[0]);
    pid_reset(&g_load_velocity_pid[1]);
    g_launcher_state = LAUNCHER_RETURNING;
  }

  if (g_load_control_enabled &&
      motor_is_online(&g_load_feedback[0], now) &&
      motor_is_online(&g_load_feedback[1], now))
  {
    load0_current = position_controller(&g_load_feedback[0],
                                        &g_load_angle_pid[0],
                                        &g_load_velocity_pid[0],
                                        g_load_target,
                                        0,
                                        16384);
    /* Motor ID 3 is installed in the reverse direction, matching the source
     * project's angle_reverse=true configuration. */
    load1_current = (int16_t)-position_controller(&g_load_feedback[1],
                                                   &g_load_angle_pid[1],
                                                   &g_load_velocity_pid[1],
                                                   g_load_target,
                                                   0,
                                                   16384);

    /* During command 2 only, give each 3508 an independent, finite boost if
     * it is far from the target but has remained nearly stationary.  The
     * current direction is taken from that motor's normal PID output, so the
     * reverse-mounted motor keeps its existing sign convention. */
    if ((g_launcher_state == LAUNCHER_LOADING) &&
        (g_load_command_waiting != 0U) &&
        (g_load_command_number == 2U))
    {
      load0_current = load_apply_stall_boost(0U, &g_load_feedback[0],
                                             g_load_target, load0_current,
                                             now);
      load1_current = load_apply_stall_boost(1U, &g_load_feedback[1],
                                             g_load_target, load1_current,
                                             now);
    }

    g_last_load_current[0] = load0_current;
    g_last_load_current[1] = load1_current;
  }
  else if (g_load_control_enabled)
  {
    /* Keep the last M3508 command during a feedback gap; zero would release
     * the spring-loaded slide. */
    load0_current = g_last_load_current[0];
    load1_current = g_last_load_current[1];
    pid_reset(&g_load_angle_pid[0]);
    pid_reset(&g_load_angle_pid[1]);
    pid_reset(&g_load_velocity_pid[0]);
    pid_reset(&g_load_velocity_pid[1]);
  }
  else
  {
    g_last_load_current[0] = 0;
    g_last_load_current[1] = 0;
  }

  load_stable = (uint8_t)(motor_is_online(&g_load_feedback[0], now) &&
                          motor_is_online(&g_load_feedback[1], now) &&
                          motor_at_target_and_stopped(&g_load_feedback[0],
                                                      g_load_target,
                                                      LOAD_TARGET_TOLERANCE,
                                                      0) &&
                          motor_at_target_and_stopped(&g_load_feedback[1],
                                                      g_load_target,
                                                      LOAD_TARGET_TOLERANCE,
                                                      0));
  if ((g_load_command_waiting != 0U) &&
      ((g_launcher_state == LAUNCHER_LOADING) ||
       (g_launcher_state == LAUNCHER_RETURNING)))
  {
    if (load_stable)
    {
      if (g_load_stable_start_ms == 0U)
      {
        g_load_stable_start_ms = now;
      }
      else if ((uint32_t)(now - g_load_stable_start_ms) >=
               POSITION_STABLE_TIME_MS)
      {
        g_load_command_waiting = 0U;
        if (g_launcher_state == LAUNCHER_LOADING)
        {
          uart7_queue_response("CMD 2 DONE\r\n");
        }
        else if (g_launcher_state == LAUNCHER_RETURNING)
        {
          g_launcher_state = LAUNCHER_ARMED;
          uart7_queue_response("CMD 3 DONE\r\n");
        }
      }
    }
    else
    {
      g_load_stable_start_ms = 0U;
    }

    if ((g_load_command_waiting != 0U) &&
        ((g_launcher_state == LAUNCHER_LOADING) ||
         (g_launcher_state == LAUNCHER_RETURNING)) &&
        ((uint32_t)(now - g_load_motion_start_ms) >= MOTION_TIMEOUT_MS))
    {
      uint8_t timeout_command = g_load_command_number;

      /* Keep position control active in the fault state: the spring-loaded
       * mechanism must remain held after a timeout. */
      g_load_command_waiting = 0U;
      g_load_stable_start_ms = 0U;
      g_launcher_state = LAUNCHER_FAULT;
      if (timeout_command == 2U)
      {
        uart7_queue_response("CMD 2 TIMEOUT\r\n");
      }
      else
      {
        uart7_queue_response("CMD 3 TIMEOUT\r\n");
      }
    }
  }

  if ((g_launcher_state == LAUNCHER_FIRE_WAIT) &&
      ((uint32_t)(now - g_fire_start_ms) >= FIRE_SERVO_DELAY_MS))
  {
    /* 3508 output has already been disconnected.  Only now move the servo
     * into the trigger/release (avoid) position. */
    set_trigger_servo_angle(0U, SERVO_AVOID_0);
    set_trigger_servo_angle(1U, SERVO_AVOID_1);
    g_fire_start_ms = now;
    g_launcher_state = LAUNCHER_FIRING;
  }

  if ((g_launcher_state == LAUNCHER_FIRING) &&
      ((uint32_t)(now - g_fire_start_ms) >= FIRE_TRIGGER_TIME_MS))
  {
    set_trigger_servo_angle(0U, SERVO_LOCK_0);
    set_trigger_servo_angle(1U, SERVO_LOCK_1);
    g_launcher_state = LAUNCHER_IDLE;
    uart7_queue_response("CMD 4 DONE\r\n");
  }

  g_debug_trigger_current = trigger_current;
  g_debug_load_current[0] = load0_current;
  g_debug_load_current[1] = load1_current;
  g_debug_yaw_current = yaw_current;

  launcher_send_motor_currents(trigger_current, load0_current,
                               load1_current, yaw_current);
}

static void launcher_send_motor_currents(int16_t trigger_current,
                                         int16_t load0_current,
                                         int16_t load1_current,
                                         int16_t yaw_current)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t data[8] = {0};
  uint32_t mailbox;

  tx_header.StdId = 0x200U;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;
  data[0] = (uint8_t)((uint16_t)trigger_current >> 8);
  data[1] = (uint8_t)trigger_current;
  data[2] = (uint8_t)((uint16_t)load0_current >> 8);
  data[3] = (uint8_t)load0_current;
  data[4] = (uint8_t)((uint16_t)load1_current >> 8);
  data[5] = (uint8_t)load1_current;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
  {
    (void)HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
  }

  tx_header.StdId = 0x1FEU;
  data[0] = 0U;
  data[1] = 0U;
  data[2] = 0U;
  data[3] = 0U;
  data[4] = 0U;
  data[5] = 0U;
  data[6] = (uint8_t)((uint16_t)yaw_current >> 8);
  data[7] = (uint8_t)yaw_current;

  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U)
  {
    (void)HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  launcher_init();
  can1_start_receive_all();
  HAL_UART_Receive_IT(&huart7, &aRxBuffer, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    launcher_apply_commands();

    /* SysTick is the 1 ms bare-metal time base.  Keep the control loop
     * non-blocking so UART reception remains responsive. */
    {
      static uint32_t last_control_ms = 0U;
      uint32_t now = HAL_GetTick();
      if ((uint32_t)(now - last_control_ms) >= MOTOR_CONTROL_PERIOD_MS)
      {
        last_control_ms = now;
        launcher_control_step();
      }
    }
    uart7_response_service();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if (hcan != &hcan1)
  {
    return;
  }

  /* Drain the FIFO because CAN1 is shared with other devices. */
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) !=
        HAL_OK)
    {
      break;
    }

    if (rx_header.IDE != CAN_ID_STD)
    {
      continue;
    }

    switch (rx_header.StdId)
    {
      case 0x201U:
        decode_rm_feedback(&g_trigger_feedback, rx_data, 1U);
        break;
      case 0x202U:
        decode_rm_feedback(&g_load_feedback[0], rx_data, 0U);
        break;
      case 0x203U:
        decode_rm_feedback(&g_load_feedback[1], rx_data, 1U);
        break;
      case 0x208U:
        decode_rm_feedback(&g_yaw_feedback, rx_data, 0U);
        break;
      default:
        /* Other CAN1 devices are intentionally left untouched. */
        break;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if ((huart == NULL) || (huart->Instance != UART7))
	{
		return;
	}

	if (Uart7_Rx_Cnt >= (RXBUFFERSIZE - 1U))
	{
		Uart7_Rx_Cnt = 0;
		memset(RxBuffer, 0x00, sizeof(RxBuffer));
	}
	else
	{
		RxBuffer[Uart7_Rx_Cnt++] = aRxBuffer;

		if ((Uart7_Rx_Cnt >= 2U) &&
		    (RxBuffer[Uart7_Rx_Cnt - 1U] == 0x0A) &&
		    (RxBuffer[Uart7_Rx_Cnt - 2U] == 0x0D))
		{
			parse_uart_command(RxBuffer, (uint16_t)(Uart7_Rx_Cnt - 2U));

			Uart7_Rx_Cnt = 0;
			memset(RxBuffer, 0x00, sizeof(RxBuffer));
		}
	}

  HAL_UART_Receive_IT(&huart7, &aRxBuffer, 1);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == UART7))
  {
    g_uart7_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == UART7))
  {
    /* Let the main loop retry a pending acknowledgement after an error. */
    g_uart7_tx_busy = 0U;
  }
}









/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart7, (uint8_t *)&ch, 1, 0xffff);//具体是huart1还是hlpuart1看初始化的代码
  return ch;
}
 
/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart7, &ch, 1, 0xffff);
  return ch;
}








/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
