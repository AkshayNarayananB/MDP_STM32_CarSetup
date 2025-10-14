#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "oled.h"
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

char buf[64];
float gyro_bias;
//int16_t MAG_BIAS_X = -68;
//int16_t MAG_BIAS_Y = 4;
//int16_t MAG_BIAS_Z = 149;
#define MAG_BIAS_X  17.83697
#define MAG_BIAS_Y -90.91626
#define MAG_BIAS_Z  41.17581

float MAG_SOFT[3][3] = {
    { 3.53823628e-04, -3.72760336e-06,  2.35228020e-05 },
    {-3.72760336e-06,  3.57798362e-04, -1.10186884e-05 },
    { 2.35228020e-05, -1.10186884e-05,  3.69198217e-04 }
};


// Define the window where deceleration begins (TUNE THIS)
const float SLOWDOWN_DEGREES = 10.0f; // Start slowing down 10 degrees before target
const int MIN_TURN_PWM = 500;          // Minimum speed to maintain movement (Tune this)

static inline void Servo_WriteUS(uint16_t us)
{
  if (us < 500)
    us = 500;
  if (us > 2500)
    us = 2500;
  __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, us);
}

/**
 * @brief Map steering direction (deg) to servo PWM microsecond value
 *
 * User input range:
 *   -45 = Full Left  (500 µs)
 *     0 = Straight   (1150 µs)
 *   +45 = Full Right (2400 µs)
 *
 * @param steer_angle Steering angle in degrees [-45 to +45]
 * @return uint16_t Pulse width in microseconds
 */
uint16_t Steering_ToUS(int16_t steer_angle)
{
    if (steer_angle < -45) steer_angle = -45;
    if (steer_angle >  45) steer_angle =  45;

    // Linear interpolation
    // slope = (2400 - 500) / (45 - (-45)) = 1900 / 90 ≈ 21.111 µs per degree
    // but we want exact 0° = 1150, so adjust baseline

    int32_t us = 1150 + (int32_t)steer_angle * ( (2400 - 500) / 90 );

    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, (uint16_t)us);
    return (uint16_t)us;
}

/* USER CODE BEGIN 0 */
#define COMMAND_SIZE 5
#define QUEUE_DEPTH 10 // Max 10 commands waiting

// Command Structure
typedef struct {
    char buffer[COMMAND_SIZE + 1]; // +1 for null terminator
} Command_t;

// Queue Structure (Simple Circular Buffer)
typedef struct {
    Command_t commands[QUEUE_DEPTH];
    volatile int head; // Points to the next spot to enqueue (write)
    volatile int tail; // Points to the next spot to dequeue (read)
    volatile int count; // Number of items currently in the queue
} CommandQueue_t;

// Global Variables
CommandQueue_t cmdQueue;
char rx_buffer[COMMAND_SIZE]; // Temporary buffer for the UART interrupt
/* USER CODE END 0 */

#define CMD_BUF_LEN 64
uint8_t cmd_buf[CMD_BUF_LEN];
uint32_t cmd_index = 0;
uint8_t commandReady = 0;
#define FORWARD 1
#define REVERSE 0
// IR global variables
volatile uint16_t raw4, raw5;
volatile uint32_t mv4, mv5;
volatile float dist4, dist5;
volatile float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
// --- ICM20948 address / WHO_AM_I ---

// ... existing ICM-20948 definitions ...
#define WHO_AM_I 0x00
#define WHO_AM_I_VAL 0xEA
#define REG_BANK_SEL 0x7F

// *** MAGNETOMETER (AK09916) REGISTERS AND MODES ***
#define AK09916_I2C_ADDR (0x0C) // AK09916 7-bit I2C Address
#define AK_CNTL2 0x31          // AK09916 Control Register 2
#define AK_HXL 0x11            // AK09916 X-Low data register (start of read)
#define AK_MODE_CONT_100HZ 0x08 // Continuous measurement at 100 Hz

// ICM Bank 0 Key Registers
#define ICM_REG_USER_CTRL 0x03
#define ICM_REG_INT_PIN_CFG 0x0F
#define ICM_REG_EXT_SENS_DATA_00 0x33 // Where the magnetometer data is stored

// ICM Bank 3 Key Registers for I2C Master Setup
#define ICM_BANK_3 0x30
#define ICM_REG_I2C_SLV0_ADDR 0x03
#define ICM_REG_I2C_SLV0_REG 0x04
#define ICM_REG_I2C_SLV0_CTRL 0x05
#define ICM_REG_I2C_SLV4_ADDR 0x10
#define ICM_REG_I2C_SLV4_REG 0x11
#define ICM_REG_I2C_SLV4_DO 0x12
#define ICM_REG_I2C_SLV4_CTRL 0x13
// *************************************************

#define ICM_ADDR_68 (0x68 << 1)         // AD0 = 0
#define ICM_ADDR_69 (0x69 << 1)         // AD0 = 1
static uint16_t ICM_ADDR = ICM_ADDR_69; // will be auto-detected
char buf[64];

uint8_t ch;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_ADC1_Init(void);

uint32_t counter = 0;          // Timer 2 counter
int16_t count = 0;             // Convert counter to signed value
int16_t no_of_tick = 50;       // number of tick used in SysTick to calculate speed, in msec
int16_t speed = 0;             // speed in term of number of edges detected per Systick
int16_t rpm = 0;               // speed in rpm number of count/sec * 60 sec  divide by 260 count per round
int start = 0;                 // use to start stop the motor
int32_t pwmVal = 0;            // pwm value to control motor speed
int32_t pwmVal_raw = 0;        // pwm value before clamping - for debugging
int16_t pwmMax = (7200 - 200); // Maximum PWM value = 7200 keep the maximum value to 7000
int16_t pwmMin = 250;          // offset value to compensate for deadzone
int err;                       // status for checking return

int encoder_A = 0; // encoders reading of Drive A (from complement of TIM2->CNT)
int encoder_D = 0; // encoders reading of Drive D (from TIM5->CNT)

int16_t position = 0;                        // position of the motor (1 rotation = 260 count)
extern int16_t oldpos;                       // // see SysTick_Handler in stm32f4xx_it.c
int16_t angle = 0;                           // angle of rotation, in degree resolution = 360 degree/260 tick
int16_t target_angle = 0;                    // target angle of rotation,
int16_t position_target;                     // target position
int16_t direction;                           // motor direction 0 or 1
int16_t error;                               // error between target and actual
int32_t error_area = 0;                      // area under error - to calculate I for PI implementation
int32_t error_old, error_change, error_rate; // to calculate D for PID control
int32_t millisOld, millisNow, dt;            // to calculate I and D for PID control

// --- Motor Forward Global Variables -- //

static uint32_t mf_last_time = 0;
static float mf_heading = 0.0f;
static float mf_target_heading = 0.0f;
static uint8_t mf_initialized = 0;

static float mf_integral = 0.0f;
static float mf_last_error = 0.0f;
static float mf_gz_filtered = 0.0f;

#define COUNTS_PER_REV     774.0f //265.0f   // your measured value (you already use 265)
#define WHEEL_CIRCUM_CM     21.0f

static inline float counts_to_cm(int32_t delta_counts) {
  return ((float)delta_counts / COUNTS_PER_REV) * WHEEL_CIRCUM_CM;
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  counter = __HAL_TIM_GET_COUNTER(htim);
  count = (int16_t)counter;
  position = count / 2; // x2 encoding
  angle = count / 2;    // x2 encoding
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // see EXTI0_IRQHandler() in stm32f4xx_it.c for interrupt
  if (GPIO_Pin == USER_PB_Pin)
  {
    // toggle LED
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12); // LED - A12
    if (start == 0)
    {
      start = 1;
      // reset all value to Zero
      TIM2->CNT = 0; // Timer Counter Value
      speed = 0;
      position = 0; // see SysTick_Handler in stm32f4xx_it.c
      oldpos = 0;   // see SysTick_Handler in stm32f4xx_it.c
      angle = 0;
      pwmVal = 0;
    }
    else
      start = 0;
  }
}

void MotorDrive_enable(void)
{
  // Enable PWM through TIM4-CH1/CH4 to drive the DC motor - Rev D board
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // on Motor drive A interface
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // on Motor drive A interface
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // on Motor drive D interface
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // on Motor drive D interface
}

void Motor_stop(void)
{
  // Set both IN1 and IN2 pins = '1'
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwmMax);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwmMax);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmMax);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwmMax);
}

static void Motor_set_left_pwm(int pwm, int direction)
{
    // Clamp to valid PWM range
    if (pwm > pwmMax)  pwm = pwmMax;
    if (pwm < pwmMin)  pwm = pwmMin;

    if (direction == FORWARD) {
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, pwm);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
    } else { // REVERSE
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
        __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwm);
    }
}

static void Motor_set_right_pwm(int pwm, int direction)
{
    // Clamp to valid PWM range
    if (pwm > pwmMax)  pwm = pwmMax;
    if (pwm < pwmMin)  pwm = pwmMin;

    if (direction == FORWARD) {
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, pwm);
    } else { // REVERSE
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwm);
        __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
    }
}

static void Motor_set_pwm(int left_pwm, int right_pwm)
{
  // Clamp to valid PWM range
  if (left_pwm > pwmMax)  left_pwm = pwmMax;
  if (left_pwm < pwmMin)  left_pwm = pwmMin;
  if (right_pwm > pwmMax) right_pwm = pwmMax;
  if (right_pwm < pwmMin) right_pwm = pwmMin;

  // Left motor (TIM4, CH3/CH4)
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, left_pwm);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);

  // Right motor (TIM1, CH3/CH4)
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, right_pwm);
}

void Motor_forward_reset(void)
{
    mf_last_time = 0;
    mf_heading = 0.0f;
    mf_target_heading = 0.0f;
    mf_initialized = 0;

    mf_integral = 0.0f;
    mf_last_error = 0.0f;
    mf_gz_filtered = 0.0f;
}

void run_straight_to_distance_cm_backward_MAG(float target_cm, int base_pwm)
{
    // --- Static variables persist across loop calls ---
    static float heading_integral_bwd = 0.0f;
    static float heading_last_error_bwd = 0.0f;
    static float speed_integral_bwd = 0.0f;
    static float speed_last_error_bwd = 0.0f;
    static float fused_heading_bwd = 0.0f;    // <-- fused heading
    static uint32_t last_time_bwd = 0;
    static int32_t prev_left_counts_bwd = 0;
    static int32_t prev_right_counts_bwd = 0;

    // --- One-time initialization ---
    static bool initialized_bwd = false;
    if (!initialized_bwd) {
        heading_integral_bwd = 0.0f;
        heading_last_error_bwd = 0.0f;
        speed_integral_bwd = 0.0f;
        speed_last_error_bwd = 0.0f;
        fused_heading_bwd = 0.0f;

        last_time_bwd = HAL_GetTick();
        prev_left_counts_bwd = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        prev_right_counts_bwd = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        initialized_bwd = true;
    }

    // --- Reset encoder counts ---
    int32_t start_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    float travelled_cm = 0.0f;

    // --- PID gains ---
    float Kp_h = 40.0f, Ki_h = 5.0f, Kd_h = 3.5f;
    float Kp_e = 0.5f, Ki_e = 0.04f, Kd_e = 0.1f;

    char buf[50];

    while (1)
    {
        int32_t cur_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t diff = cur_pos - start_pos;
        travelled_cm = counts_to_cm(abs(diff));
        float remaining = target_cm - travelled_cm;

        if (remaining <= 1.0f) {  // tolerance
            Motor_stop();
            initialized_bwd = false;
            break;
        }

        // --- Δt calculation ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time_bwd) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_time_bwd = now;

        // --- Read calibrated magnetometer ---
        int16_t mx_raw, my_raw, mz_raw;
        ICM20948_ReadMagRaw(&mx_raw, &my_raw, &mz_raw);

        float mx = (float)mx_raw;
        float my = (float)my_raw;
        float mz = (float)mz_raw;
        apply_mag_calibration(&mx, &my, &mz);

        // --- Read gyro ---
        int16_t gz_raw;
        ICM20948_ReadRaw(NULL, NULL, NULL, NULL, NULL, &gz_raw);
        float gz_dps = ((float)gz_raw / 131.0f) - gyro_bias;

        // --- Complementary filter ---
        float mag_heading = atan2f(my, mx);   // radians
        fused_heading_bwd += gz_dps * (3.14159265f / 180.0f) * dt; // gyro integration
        float alpha = 0.98f;
        fused_heading_bwd = alpha*fused_heading_bwd + (1.0f-alpha)*mag_heading;

        if (fused_heading_bwd > 3.14159265f) fused_heading_bwd -= 2.0f*3.14159265f;
        if (fused_heading_bwd < -3.14159265f) fused_heading_bwd += 2.0f*3.14159265f;

        // --- Heading PID ---
        float heading_error = 0.0f - fused_heading_bwd;  // target is straight
        heading_integral_bwd += heading_error * dt;
        float heading_derivative = (heading_error - heading_last_error_bwd) / dt;
        heading_last_error_bwd = heading_error;
        float gyro_correction = Kp_h*heading_error + Ki_h*heading_integral_bwd + Kd_h*heading_derivative;

        // --- Encoder PID ---
        int32_t left_counts  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t right_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        int32_t left_delta  = left_counts - prev_left_counts_bwd;
        int32_t right_delta = right_counts - prev_right_counts_bwd;
        prev_left_counts_bwd  = left_counts;
        prev_right_counts_bwd = right_counts;

        float speed_error = (float)(left_delta - right_delta);
        speed_integral_bwd += speed_error * dt;
        float speed_derivative = (speed_error - speed_last_error_bwd) / dt;
        speed_last_error_bwd = speed_error;
        float encoder_correction = Kp_e*speed_error + Ki_e*speed_integral_bwd + Kd_e*speed_derivative;

        // --- PWM scaling ---
        int pwm_backward = base_pwm;
        if (remaining < 5.0f) pwm_backward = (int)(0.6f*base_pwm);
        if (remaining < 1.0f) pwm_backward = pwmMin + 60;

        // --- Apply corrections ---
        int left_pwm  = pwm_backward + (int)(0.7*gyro_correction + 0.3*encoder_correction);
        int right_pwm = pwm_backward + (int)(-0.7*gyro_correction - 0.3*encoder_correction);

        if (left_pwm > pwmMax) left_pwm = pwmMax;
        if (left_pwm < pwmMin) left_pwm = pwmMin;
        if (right_pwm > pwmMax) right_pwm = pwmMax;
        if (right_pwm < pwmMin) right_pwm = pwmMin;

        if (heading_integral_bwd > 100) heading_integral_bwd = 100;
        if (heading_integral_bwd < -100) heading_integral_bwd = -100;
        if (speed_integral_bwd > 50) speed_integral_bwd = 50;
        if (speed_integral_bwd < -50) speed_integral_bwd = -50;

        Motor_set_pwm_reverse(left_pwm, right_pwm);

        // --- Optional OLED updates ---
        OLED_Clear();
        sprintf(buf, "Target: %.1fcm", target_cm);
        OLED_ShowString(0, 0, (uint8_t *)buf);
        sprintf(buf, "Travel: %.1fcm", travelled_cm);
        OLED_ShowString(0, 20, (uint8_t *)buf);
        float left_rpm = (abs(left_delta)/COUNTS_PER_REV)/dt*60.0f;
        float right_rpm = (abs(right_delta)/COUNTS_PER_REV)/dt*60.0f;
        sprintf(buf, "L: %.1f", left_rpm);
        OLED_ShowString(0, 40, (uint8_t *)buf);
        sprintf(buf, "R: %.1f", right_rpm);
        OLED_ShowString(60, 40, (uint8_t *)buf);
        OLED_Refresh_Gram();

        HAL_Delay(10);
    }

    // --- Final display ---
    OLED_Clear();
    sprintf(buf, "Target: %.1fcm", target_cm);
    OLED_ShowString(0, 0, (uint8_t *)buf);
    sprintf(buf, "Travel: %.1fcm", travelled_cm);
    OLED_ShowString(0, 20, (uint8_t *)buf);
    float err_pct = (travelled_cm - target_cm)/target_cm*100.0f;
    sprintf(buf, "Error: %.1f%%", err_pct);
    OLED_ShowString(0, 40, (uint8_t *)buf);
    OLED_Refresh_Gram();
}


/**
 * @brief Moves the robot straight backward for a target distance using cascaded PID control.
 * @param target_cm The total distance (in cm) the robot should travel backward.
 * @param base_pwm The base speed (PWM magnitude) to apply to the motors.
 */
void run_straight_to_distance_cm_backward(float target_cm, int base_pwm)
{
    // --- Static variables persist across loop calls (Renamed to prevent collision with forward function) ---
    static float heading_integral_bwd = 0.0f;
    static float heading_last_error_bwd = 0.0f;
    static float speed_integral_bwd = 0.0f;
    static float speed_last_error_bwd = 0.0f;
    static float gz_filtered_bwd = 0.0f;
    static uint32_t last_time_bwd = 0;
    static int32_t prev_left_counts_bwd = 0;
    static int32_t prev_right_counts_bwd = 0;

    // --- One-time initialization ---
    static bool initialized_bwd = false;
    if (!initialized_bwd) {
        // Reset PID state when starting a new movement
        heading_integral_bwd = 0.0f;
        heading_last_error_bwd = 0.0f;
        speed_integral_bwd = 0.0f;
        speed_last_error_bwd = 0.0f;
        gz_filtered_bwd = 0.0f;

        last_time_bwd = HAL_GetTick();
        prev_left_counts_bwd = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        prev_right_counts_bwd = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        initialized_bwd = true;
    }

    // --- Reset encoder counts and distance tracking (Uses the current encoder counts) ---
    int32_t start_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    float travelled_cm = 0.0f;

    // --- Control parameters (Exact match to original function) ---
    const float tol_cm = 1.0f;
    const float slow_down_cm = 5.0f;
    const float creep_cm = 1.0f;

    // --- PID gains (tune these!) ---
    // Heading PID (Outer loop)
    float Kp_h = 40.0f;
    float Ki_h = 5.0f;
    float Kd_h = 3.5f;

    // Speed PID (Inner loop)
    float Kp_e = 0.5f;
    float Ki_e = 0.04f;
    float Kd_e = 0.1f;

    char buf[50];

    while (1)
    {
        // --- Distance travelled ---
        int32_t cur_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t diff = cur_pos - start_pos;
        // abs() is used to track distance regardless of encoder direction (forward or backward)
        travelled_cm = counts_to_cm(abs(diff));

        float remaining = target_cm - travelled_cm;

        if (remaining <= tol_cm) {
            Motor_stop();
            // Reset initialization flag so next call (forward or backward) runs initialization
            initialized_bwd = false;
            break;
        }

        // --- Δt calculation ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time_bwd) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_time_bwd = now;

        // --- Gyro Read & Heading Calculation (Outer Loop) ---
        int16_t gz_raw;
        // Assuming ICM20948_ReadRaw and gyro_bias are defined globally
        ICM20948_ReadRaw(NULL, NULL, NULL, NULL, NULL, &gz_raw);
        float gz_dps = gz_raw / 131.0f - gyro_bias;
        gz_filtered_bwd = 0.9f * gz_filtered_bwd + 0.1f * gz_dps;
        float heading = 0.0f; // Target heading is 0.0f (straight line)
        heading += gz_filtered_bwd * dt;

        float heading_error = 0.0f - heading;
        heading_integral_bwd += heading_error * dt;
        float heading_derivative = (heading_error - heading_last_error_bwd) / dt;
        heading_last_error_bwd = heading_error;

        float gyro_correction = Kp_h * heading_error + Ki_h * heading_integral_bwd + Kd_h * heading_derivative;

        // --- Encoder Read & Speed Correction (Inner Loop) ---
        int32_t left_counts  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t right_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

        int32_t left_delta  = left_counts - prev_left_counts_bwd;
        int32_t right_delta = right_counts - prev_right_counts_bwd;
        prev_left_counts_bwd  = left_counts;
        prev_right_counts_bwd = right_counts;

        // Note: speed_error is calculated as (Left - Right) delta counts
        float speed_error = (float)(left_delta - right_delta);

        speed_integral_bwd += speed_error * dt;
        float speed_derivative = (speed_error - speed_last_error_bwd) / dt;
        speed_last_error_bwd = speed_error;

        float encoder_correction = Kp_e * speed_error + Ki_e * speed_integral_bwd + Kd_e * speed_derivative;

        // --- PWM scaling (Magnitude control) ---
        int pwm_backward = base_pwm;
        if (remaining < slow_down_cm) pwm_backward = (int)(0.6f * base_pwm);
        if (remaining < creep_cm) pwm_backward = pwmMin + 60; // Assuming pwmMin is available

        // --- Apply Corrections (SIGNS FLIPPED FOR BACKWARD MOTION) ---
        // Original Forward: Left = PWM + (+G - E), Right = PWM + (-G + E)
        // Backward: Left = PWM + (-G + E), Right = PWM + (+G - E)

        //int left_pwm  = pwm_backward + (int)(-0.7 * gyro_correction + 0.3 * encoder_correction);
        //int right_pwm = pwm_backward + (int)( 0.7 * gyro_correction - 0.3 * encoder_correction);

        // Gemini Input, reverse signs for reverse directed drift
        // Suggested fix: Signs of gyro_correction flipped for backward steering logic
        int left_pwm  = pwm_backward + (int)( 0.7 * gyro_correction + 0.3 * encoder_correction);  // Added correction (slowing down left in reverse)
        int right_pwm = pwm_backward + (int)(-0.7 * gyro_correction - 0.3 * encoder_correction); // Subtracted correction (speeding up right in reverse)

        // --- Clamp PWM ---
        // Ensure the magnitude of PWM stays within the allowed range
        if (left_pwm > pwmMax) left_pwm = pwmMax;
        if (left_pwm < pwmMin) left_pwm = pwmMin;
        if (right_pwm > pwmMax) right_pwm = pwmMax;
        if (right_pwm < pwmMin) right_pwm = pwmMin;

        // --- Anti-windup clamping for integrals ---
        if (heading_integral_bwd > 100) heading_integral_bwd = 100;
        if (heading_integral_bwd < -100) heading_integral_bwd = -100;

        if (speed_integral_bwd > 50) speed_integral_bwd = 50;
        if (speed_integral_bwd < -50) speed_integral_bwd = -50;


        Motor_set_pwm_reverse(left_pwm, right_pwm); // Use the reverse motor function

        // --- Optional OLED updates (Exact match to original) ---
        OLED_Clear(); // Assuming OLED functions exist
        sprintf(buf, "Target: %.1fcm", target_cm);
        OLED_ShowString(0, 0, (uint8_t *)buf);
        sprintf(buf, "Travel: %.1fcm", travelled_cm);
        OLED_ShowString(0, 20, (uint8_t *)buf);

        // Calculate RPM using the provided COUNTS_PER_REV
        float left_rpm = (abs(left_delta) / COUNTS_PER_REV) / dt * 60.0f;
        float right_rpm = (abs(right_delta) / COUNTS_PER_REV) / dt * 60.0f;

        // Display live RPM
        sprintf(buf, "L: %.1f", left_rpm);
        OLED_ShowString(0, 40, (uint8_t *)buf);
        sprintf(buf, "R: %.1f", right_rpm);
        OLED_ShowString(60, 40, (uint8_t *)buf);

        OLED_Refresh_Gram();

        HAL_Delay(10);
    }

    // --- Final OLED display (Exact match to original) ---
    OLED_Clear();
    sprintf(buf, "Target: %.1fcm", target_cm);
    OLED_ShowString(0, 0, (uint8_t *)buf);
    sprintf(buf, "Travel: %.1fcm", travelled_cm);
    OLED_ShowString(0, 20, (uint8_t *)buf);
    float err_pct = (travelled_cm - target_cm) / target_cm * 100.0f;
    sprintf(buf, "Error: %.1f%%", err_pct);
    OLED_ShowString(0, 40, (uint8_t *)buf);
    OLED_Refresh_Gram();
}

/**
 * @brief Sets the PWM for both motors to drive them in reverse.
 * * This function reverses direction by swapping the PWM application channels
 * compared to the forward motion function.
 */
void Motor_set_pwm_reverse(int left_pwm, int right_pwm)
{
    // Clamp PWM (magnitude only)
    if (left_pwm > pwmMax)  left_pwm = pwmMax;
    if (left_pwm < pwmMin)  left_pwm = pwmMin;
    if (right_pwm > pwmMax) right_pwm = pwmMax;
    if (right_pwm < pwmMin) right_pwm = pwmMin;

    // Left motor (TIM4, CH3/CH4) - REVERSE
    // Forward was CH3=PWM, CH4=0. Reverse is CH3=0, CH4=PWM.
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, left_pwm);

    // Right motor (TIM1, CH3/CH4) - REVERSE
    // Forward was CH3=0, CH4=PWM. Reverse is CH3=PWM, CH4=0.
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, right_pwm);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);
}

void run_straight_to_distance_cm_MAG(float target_cm, int base_pwm)
{
    // --- Static variables persist across loop calls ---
    static float heading_integral = 0.0f;
    static float heading_last_error = 0.0f;
    static float speed_integral = 0.0f;
    static float speed_last_error = 0.0f;
    static float fused_heading = 0.0f;  // fused heading with gyro+mag
    static uint32_t last_time = 0;
    static int32_t prev_left_counts = 0;
    static int32_t prev_right_counts = 0;

    // --- One-time initialization ---
    static bool initialized = false;
    if (!initialized) {
        heading_integral = 0.0f;
        heading_last_error = 0.0f;
        speed_integral = 0.0f;
        speed_last_error = 0.0f;
        fused_heading = 0.0f;

        last_time = HAL_GetTick();
        prev_left_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        prev_right_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        initialized = true;
    }

    int32_t start_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    float travelled_cm = 0.0f;

    const float tol_cm = 1.0f;
    const float slow_down_cm = 5.0f;
    const float creep_cm = 1.0f;

    float Kp_h = 40.0f, Ki_h = 5.0f, Kd_h = 3.5f;
    float Kp_e = 0.5f, Ki_e = 0.04f, Kd_e = 0.1f;

    char buf[50];

    while (1) {
        int32_t cur_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t diff = cur_pos - start_pos;
        travelled_cm = counts_to_cm(abs(diff));
        float remaining = target_cm - travelled_cm;

        if (remaining <= tol_cm) {
            Motor_stop();
            initialized = false;
            break;
        }

        // --- Δt calculation ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_time = now;

        // --- Read calibrated magnetometer ---
        int16_t mx_raw, my_raw, mz_raw;
        ICM20948_ReadMagRaw(&mx_raw, &my_raw, &mz_raw);
        float mx = (float)mx_raw;
        float my = (float)my_raw;
        float mz = (float)mz_raw;
        apply_mag_calibration(&mx, &my, &mz);

        // --- Read gyro ---
        int16_t gz_raw;
        ICM20948_ReadRaw(NULL, NULL, NULL, NULL, NULL, &gz_raw);
        float gz_dps = ((float)gz_raw / 131.0f) - gyro_bias;

        // --- Complementary filter ---
        float mag_heading = atan2f(my, mx);  // radians
        fused_heading += gz_dps * (3.14159265f / 180.0f) * dt; // gyro integration
        float alpha = 0.98f;
        fused_heading = alpha * fused_heading + (1.0f - alpha) * mag_heading;

        if (fused_heading > 3.14159265f) fused_heading -= 2.0f * 3.14159265f;
        if (fused_heading < -3.14159265f) fused_heading += 2.0f * 3.14159265f;

        // --- Heading PID ---
        float heading_error = 0.0f - fused_heading;
        heading_integral += heading_error * dt;
        float heading_derivative = (heading_error - heading_last_error) / dt;
        heading_last_error = heading_error;
        float gyro_correction = Kp_h * heading_error + Ki_h * heading_integral + Kd_h * heading_derivative;

        // --- Encoder PID ---
        int32_t left_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t right_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
        int32_t left_delta = left_counts - prev_left_counts;
        int32_t right_delta = right_counts - prev_right_counts;
        prev_left_counts = left_counts;
        prev_right_counts = right_counts;

        float speed_error = (float)(left_delta - right_delta);
        speed_integral += speed_error * dt;
        float speed_derivative = (speed_error - speed_last_error) / dt;
        speed_last_error = speed_error;
        float encoder_correction = Kp_e * speed_error + Ki_e * speed_integral + Kd_e * speed_derivative;

        // --- PWM scaling ---
        int pwm_forward = base_pwm;
        if (remaining < slow_down_cm) pwm_forward = (int)(0.6f * base_pwm);
        if (remaining < creep_cm) pwm_forward = pwmMin + 60;

        // --- Apply corrections ---
        int left_pwm = pwm_forward + (int)(0.7 * gyro_correction - 0.3 * encoder_correction);
        int right_pwm = pwm_forward + (int)(-0.7 * gyro_correction + 0.3 * encoder_correction);

        // --- Clamp PWM ---
        if (left_pwm > pwmMax) left_pwm = pwmMax;
        if (left_pwm < pwmMin) left_pwm = pwmMin;
        if (right_pwm > pwmMax) right_pwm = pwmMax;
        if (right_pwm < pwmMin) right_pwm = pwmMin;

        if (heading_integral > 500) heading_integral = 500;
        if (heading_integral < -500) heading_integral = -500;
        if (speed_integral > 50) speed_integral = 50;
        if (speed_integral < -50) speed_integral = -50;

        Motor_set_pwm(left_pwm, right_pwm);

        // --- Optional OLED updates ---
        OLED_Clear();
        sprintf(buf, "Target: %.1fcm", target_cm);
        OLED_ShowString(0, 0, (uint8_t *)buf);
        sprintf(buf, "Travel: %.1fcm", travelled_cm);
        OLED_ShowString(0, 20, (uint8_t *)buf);

        float left_rpm = (abs(left_delta) / COUNTS_PER_REV) / dt * 60.0f;
        float right_rpm = (abs(right_delta) / COUNTS_PER_REV) / dt * 60.0f;
        sprintf(buf, "L: %.1f", left_rpm);
        OLED_ShowString(0, 40, (uint8_t *)buf);
        sprintf(buf, "R: %.1f", right_rpm);
        OLED_ShowString(60, 40, (uint8_t *)buf);
        OLED_Refresh_Gram();

        HAL_Delay(10);
    }

    // --- Final OLED display ---
    OLED_Clear();
    sprintf(buf, "Target: %.1fcm", target_cm);
    OLED_ShowString(0, 0, (uint8_t *)buf);
    sprintf(buf, "Travel: %.1fcm", travelled_cm);
    OLED_ShowString(0, 20, (uint8_t *)buf);
    float err_pct = (travelled_cm - target_cm) / target_cm * 100.0f;
    sprintf(buf, "Error: %.1f%%", err_pct);
    OLED_ShowString(0, 40, (uint8_t *)buf);
    OLED_Refresh_Gram();
}




void run_straight_to_distance_cm(float target_cm, int base_pwm)
{
    // --- Static variables persist across loop calls ---
    static float heading_integral = 0.0f;
    static float heading_last_error = 0.0f;
    static float speed_integral = 0.0f;
    static float speed_last_error = 0.0f;
    static float gz_filtered = 0.0f;
    static uint32_t last_time = 0;
    static int32_t prev_left_counts = 0;
    static int32_t prev_right_counts = 0;

    // --- One-time initialization ---
   static bool initialized = false;
   if (!initialized) {
    // Reset PID state when starting a new movement
	   heading_integral = 0.0f;
       heading_last_error = 0.0f;
       speed_integral = 0.0f;
       speed_last_error = 0.0f;
       gz_filtered = 0.0f;

       last_time = HAL_GetTick();
       prev_left_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
       prev_right_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
       initialized = true;
    }

    // --- Reset encoder counts and distance tracking ---
    int32_t start_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    float travelled_cm = 0.0f;

    // --- Control parameters ---
    const float tol_cm = 1.0f;
    const float slow_down_cm = 5.0f;
    const float creep_cm = 1.0f;

    // --- PID gains (tune these!) ---
    // Heading PID (Outer loop)
    float Kp_h = 40.0f;
    float Ki_h = 5.0f;
    float Kd_h = 3.5f;

    // Speed PID (Inner loop)
    float Kp_e = 0.5f;
    float Ki_e = 0.04f;
    float Kd_e = 0.1f;

    char buf[50];
    float heading = 0.0f;

    while (1)
    {
        // --- Distance travelled ---
        int32_t cur_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t diff = cur_pos - start_pos;
        travelled_cm = counts_to_cm(abs(diff));

        float remaining = target_cm - travelled_cm;

        if (remaining <= tol_cm) {
            Motor_stop();
            initialized = false;
            break;
        }

        // --- Δt calculation ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_time = now;

        // --- Gyro Read & Heading Calculation (Outer Loop) ---
        int16_t gz_raw;
        ICM20948_ReadRaw(NULL, NULL, NULL, NULL, NULL, &gz_raw);
        float gz_dps = gz_raw / 131.0f - gyro_bias;
        gz_filtered = 0.9f * gz_filtered + 0.1f * gz_dps;
        heading += gz_filtered * dt;

        float heading_error = 0.0f - heading;
        heading_integral += heading_error * dt;
        float heading_derivative = (heading_error - heading_last_error) / dt;
        heading_last_error = heading_error;

        float gyro_correction = Kp_h * heading_error + Ki_h * heading_integral + Kd_h * heading_derivative;

        // --- Encoder Read & Speed Correction (Inner Loop) ---
        int32_t left_counts  = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t right_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);

        int32_t left_delta  = left_counts - prev_left_counts;
        int32_t right_delta = right_counts - prev_right_counts;
        prev_left_counts  = left_counts;
        prev_right_counts = right_counts;

        float speed_error = (float)(left_delta - right_delta);

        speed_integral += speed_error * dt;
        float speed_derivative = (speed_error - speed_last_error) / dt;
        speed_last_error = speed_error;

        float encoder_correction = Kp_e * speed_error + Ki_e * speed_integral + Kd_e * speed_derivative;

        // --- PWM scaling ---
        int pwm_forward = base_pwm;
        if (remaining < slow_down_cm) pwm_forward = (int)(0.6f * base_pwm);
        if (remaining < creep_cm) pwm_forward = pwmMin + 60;

        // --- Apply Corrections ---
        int left_pwm  = pwm_forward + (int)(0.7 * gyro_correction - 0.3 * encoder_correction);
        int right_pwm = pwm_forward + (int)(-0.7 * gyro_correction + 0.3 * encoder_correction);

        // --- Clamp PWM ---
        if (left_pwm > pwmMax) left_pwm = pwmMax;
        if (left_pwm < pwmMin) left_pwm = pwmMin;
        if (right_pwm > pwmMax) right_pwm = pwmMax;
        if (right_pwm < pwmMin) right_pwm = pwmMin;

        if (heading_integral > 500) heading_integral = 500;
        if (heading_integral < -500) heading_integral = -500;

        if (speed_integral > 50) speed_integral = 50;
        if (speed_integral < -50) speed_integral = -50;


        Motor_set_pwm(left_pwm, right_pwm);

        // --- Optional OLED updates ---
        OLED_Clear();
        sprintf(buf, "Target: %.1fcm", target_cm);
        OLED_ShowString(0, 0, (uint8_t *)buf);
        sprintf(buf, "Travel: %.1fcm", travelled_cm);
        OLED_ShowString(0, 20, (uint8_t *)buf);

        // Calculate RPM using the provided COUNTS_PER_REV
        float left_rpm = (abs(left_delta) / COUNTS_PER_REV) / dt * 60.0f;
        float right_rpm = (abs(right_delta) / COUNTS_PER_REV) / dt * 60.0f;

        // Display live RPM
        sprintf(buf, "L: %.1f", left_rpm);
        OLED_ShowString(0, 40, (uint8_t *)buf);
        sprintf(buf, "R: %.1f", right_rpm);
        OLED_ShowString(60, 40, (uint8_t *)buf);

        OLED_Refresh_Gram();

        HAL_Delay(10);
    }

    // --- Final OLED display ---
    OLED_Clear();
    sprintf(buf, "Target: %.1fcm", target_cm);
    OLED_ShowString(0, 0, (uint8_t *)buf);
    sprintf(buf, "Travel: %.1fcm", travelled_cm);
    OLED_ShowString(0, 20, (uint8_t *)buf);
    float err_pct = (travelled_cm - target_cm) / target_cm * 100.0f;
    sprintf(buf, "Error: %.1f%%", err_pct);
    OLED_ShowString(0, 40, (uint8_t *)buf);
    OLED_Refresh_Gram();
}

/**
 * @brief Turns the robot by a specified angle.
 *
 * This function uses a gyroscope to track the robot's heading and
 * performs a controlled arc until the target angle is reached.
 *
 * @param target_angle The total angle to turn in degrees.
 * Positive for right turns, negative for left turns.
 * (e.g., 90.0f to 360.0f or -90.0f to -360.0f)
 * @param base_pwm The base PWM value for the motors.
 * @param steer_angle The steering angle in degrees [-45 to +45].
 * A positive value for right turns, negative for left turns.
 */

void turn_by_angle_degrees(float target_angle, int base_pwm, float steer_angle)
{
    // --- PID state variables ---
    float heading = 0.0f;
    float gz_filtered = 0.0f;
    uint32_t last_time = HAL_GetTick();

    // Raw sensor variables for reading
    int16_t ax, ay, az, gx, gy, gz;
    float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;

    // --- Ensure steer_angle aligns with target_angle direction ---
    // If target is positive (right turn), ensure steer_angle is positive.
    // If target is negative (left turn), ensure steer_angle is negative.
    if (target_angle > 0 && steer_angle < 0) {
        steer_angle = -steer_angle;
    } else if (target_angle < 0 && steer_angle > 0) {
        steer_angle = -steer_angle;
    }

    // Ensure the steer_angle is within the safe range
    if (steer_angle > 45.0f) steer_angle = 45.0f;
    if (steer_angle < -45.0f) steer_angle = -45.0f;

    // Set the steering angle
    Steering_ToUS(steer_angle);
    Motor_set_pwm(base_pwm, base_pwm);

    // Prepare OLED display
    OLED_Clear();
    char buf[32];
    sprintf(buf, "Target: %.1f deg", target_angle);
    OLED_ShowString(0, 0, (uint8_t *)buf);
    OLED_Refresh_Gram();

    while (1)
    {
        // --- Read and scale gyroscope data ---
        ICM20948_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
        gz_dps = gz / 131.0f;

        // --- Δt calculation ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_time = now;

        // --- Gyro filtering & heading integration ---
        gz_filtered = 0.9f * gz_filtered + 0.1f * gz_dps;
        heading += gz_filtered * dt;

        // --- Check for completion ---
        if (fabs(heading) >= fabs(target_angle))
        {
            Motor_stop();
            Steering_ToUS(0.0);
            break;
        }

        // --- Update current heading on OLED ---
        sprintf(buf, "Current: %.1f deg", heading);
        OLED_ShowString(0, 20, (uint8_t *)buf);
        OLED_Refresh_Gram();

        HAL_Delay(10); // ~100 Hz loop
    }

    // --- Final OLED display after completion ---
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Turn Complete!");
    sprintf(buf, "Final Angle: %.1f", heading);
    OLED_ShowString(0, 20, (uint8_t *)buf);
    OLED_Refresh_Gram();
}

void turn_by_angle_degrees_backwards(float target_angle, int base_pwm, float steer_angle)
{
    // --- State Variables ---
    float heading = 0.0f;
    float gz_filtered = 0.0f;
    uint32_t last_time = HAL_GetTick();

    // Variable for impulse rejection (holds the last known good reading)
    static float gz_last_unbiased = 0.0f;

    // Raw sensor variables for reading
    int16_t gz;
    float gz_dps;

    // --- Control Parameters (TUNE THESE!) ---
    // *** NEW: Deadband Threshold (TUNE THIS!) ***
    // Set slightly higher than your observed stationary noise (e.g., 3.5 DPS)
    const float GYRO_NOISE_THRESHOLD = 3.5f;

    // Max allowed change in DPS for impulse rejection
    const float MAX_DPS_CHANGE = 150.0f;

    // Tolerance for final stop (MUST be small since there's no deceleration)
    const float COMPLETION_TOLERANCE = 0.1f;

    // NOTE: gyro_bias is assumed to be a globally defined/accessible float in DPS.

    // --- Ensure steer_angle aligns with target_angle direction ---
    if (target_angle > 0 && steer_angle < 0) {
        steer_angle = -steer_angle;
    } else if (target_angle < 0 && steer_angle > 0) {
        steer_angle = -steer_angle;
    }
    if (steer_angle > 45.0f) steer_angle = 45.0f;
    if (steer_angle < -45.0f) steer_angle = -45.0f;
    Steering_ToUS(steer_angle);

    // Prepare OLED display (Omitted for brevity)
    OLED_Clear();
    char buf[32];
    sprintf(buf, "Target: %.1f deg", target_angle);
    OLED_ShowString(0, 0, (uint8_t *)buf);
    OLED_Refresh_Gram();

    // --- Start Motor at Constant Speed ---
    Motor_set_pwm_reverse(base_pwm, base_pwm);

    while (1)
    {
        // --- Read and scale gyroscope data ---
        ICM20948_ReadRaw(NULL, NULL, NULL, NULL, NULL, &gz);
        gz_dps = gz / 131.0f;
        gz_dps = gz_dps - gyro_bias;

        // --- Δt calculation ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_time = now;

        // ------------------------------------------------------------------
        // *** FIX 1: SOFTWARE DEAD-BAND ***
        // ------------------------------------------------------------------
        if (fabs(gz_dps) < GYRO_NOISE_THRESHOLD) {
            gz_dps = 0.0f;
        }

        // ------------------------------------------------------------------
        // *** FIX 2: IMPULSE REJECTION FILTER ***
        // ------------------------------------------------------------------
        float delta_gz = gz_dps - gz_last_unbiased;

        if (fabs(delta_gz) > MAX_DPS_CHANGE)
        {
            gz_dps = gz_last_unbiased;
        }
        gz_last_unbiased = gz_dps;

        // --- Gyro filtering & heading integration ---
        gz_filtered = 0.95f * gz_filtered + 0.05f * gz_dps;
        heading += gz_filtered * dt;

        // --- Motor Control (Constant Speed) ---
        // Motor PWM is set outside the loop and remains constant (base_pwm)

        // --- Check for completion ---
        float angle_left_to_turn = fabs(target_angle) - fabs(heading);

        if (angle_left_to_turn <= COMPLETION_TOLERANCE)
        {
            Motor_stop();
            Steering_ToUS(0.0);
            break;
        }

        // --- Update current heading on OLED ---
        sprintf(buf, "Current: %.1f deg", heading);
        OLED_ShowString(0, 20, (uint8_t *)buf);
        OLED_Refresh_Gram();

        HAL_Delay(10); // ~100 Hz loop
    }

    // --- Final OLED display after completion ---
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Turn Complete!");
    sprintf(buf, "Final Angle: %.1f", heading);
    OLED_ShowString(0, 20, (uint8_t *)buf);
    OLED_Refresh_Gram();
}


/* USER CODE BEGIN 4 */

/**
 * @brief Initialize the command queue.
 */
void Queue_Init(void)
{
    cmdQueue.head = 0;
    cmdQueue.tail = 0;
    cmdQueue.count = 0;
}

/**
 * @brief Enqueue a received command.
 * @param cmd Pointer to the command buffer to enqueue.
 * @retval 1 if successful, 0 if queue is full.
 */
int Queue_Enqueue(char *cmd)
{
    if (cmdQueue.count < QUEUE_DEPTH) {
        // Copy the command (5 chars)
        memcpy(cmdQueue.commands[cmdQueue.head].buffer, cmd, COMMAND_SIZE);
        // Null-terminate the command string for safety/printing
        cmdQueue.commands[cmdQueue.head].buffer[COMMAND_SIZE] = '\0';

        cmdQueue.head = (cmdQueue.head + 1) % QUEUE_DEPTH;
        cmdQueue.count++;
        return 1;
    }
    return 0; // Queue is full
}

/**
 * @brief Dequeue the next command to be executed.
 * @param cmd Pointer to a Command_t struct where the dequeued command will be stored.
 * @retval 1 if successful, 0 if queue is empty.
 */
int Queue_Dequeue(Command_t *cmd)
{
    if (cmdQueue.count > 0) {
        // Copy the command from the queue
        memcpy(cmd->buffer, cmdQueue.commands[cmdQueue.tail].buffer, COMMAND_SIZE + 1);

        cmdQueue.tail = (cmdQueue.tail + 1) % QUEUE_DEPTH;
        cmdQueue.count--;
        return 1;
    }
    return 0; // Queue is empty
}

/**
 * @brief Executes the command stored in the queue.
 * Commands are 5 characters long and follow specific formats:
 * - fwXXX or bwXXX (e.g., fw120, bw080) where XXX is distance (up to 120, 3 digits).
 * - rhtfw, rhtbw, lftfw, lftbw (Fixed direction/turn commands).
 * @param cmd The command struct to execute.
 */
void Execute_Command(Command_t *cmd)
{
    // Safety check for null terminator (already added in Queue_Enqueue, but good practice)
    cmd->buffer[COMMAND_SIZE] = '\0';

    // -----------------------------------------------------------
    // COMMANDS WITH DISTANCE: fwXXX, bwXXX
    // -----------------------------------------------------------
    if (strncmp(cmd->buffer, "fw", 2) == 0 || strncmp(cmd->buffer, "bw", 2) == 0)
    {
        uint16_t distance = 0;
        char distance_str[4]; // To hold "XXX" + null terminator

        // Check if the command includes a 3-digit distance (e.g., "fw120")
        if (COMMAND_SIZE >= 5)
        {
            strncpy(distance_str, &cmd->buffer[2], 3);
            distance_str[3] = '\0'; // Null-terminate the string
            distance = (uint16_t)atoi(distance_str);
        }

//        char debug_msg[30];
//        sprintf(debug_msg, "CMD: %s D:%u", cmd->buffer, distance);
//        OLED_ShowString(0,0, debug_msg);
//        OLED_Refresh_Gram();

        if (strncmp(cmd->buffer, "fw", 2) == 0)
        {
            run_straight_to_distance_cm_MAG(distance,3000);
        }
        else // Must be "bw"
        {
            run_straight_to_distance_cm_backward_MAG(distance,3000);
        }
    }
    // -----------------------------------------------------------
    // FIXED 5-CHAR COMMANDS: rhtfw, rhtbw, lftfw, lftbw
    // -----------------------------------------------------------
    else if (strcmp(cmd->buffer, "rhtfw") == 0)
    {
    	  turn_by_angle_degrees(33.25, 2000, 20.0); //right forward
    }
    else if (strcmp(cmd->buffer, "rhtbw") == 0)
    {
        turn_by_angle_degrees_backwards(33.25, 2000, 20.0); //right backward
    }
    else if (strcmp(cmd->buffer, "lftfw") == 0)
    {
    	  turn_by_angle_degrees(-33.25, 2000, 20.0); //left
    }
    else if (strcmp(cmd->buffer, "lftbw") == 0)
    {
  	  turn_by_angle_degrees_backwards(-30.25, 2000, -18.0); //back left
    }
    else if (strcmp(cmd->buffer, "rboot") == 0){
    	HAL_NVIC_SystemReset(); //system reset -> program starts again from main
    }
    // -----------------------------------------------------------
    // UNKNOWN COMMAND
    // -----------------------------------------------------------
    else
    {
        // Optionally, handle unknown commands (e.g., log an error)
    }
}

/**
 * @brief Starts the non-blocking (interrupt-driven) UART reception.
 */
void Start_UART_Reception(void)
{
    // Start receiving exactly COMMAND_SIZE (5) bytes into the temporary rx_buffer
    HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buffer, COMMAND_SIZE);
}
/* USER CODE END 4 */

void Calibrate_Encoder_Counts(void)
{
    // Reset both encoder counters
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Encoder A
    __HAL_TIM_SET_COUNTER(&htim5, 0);  // Encoder D

    int16_t countsA = 0;
    int16_t countsD = 0;

    OLED_Clear();
    OLED_ShowString(0, 0, "Rotate wheel 1 rev");
    OLED_ShowString(0, 10, "A:");
    OLED_ShowString(0, 20, "D:");
    OLED_Refresh_Gram();

    while (1)
    {
        // Read live encoder counts
        countsA = __HAL_TIM_GET_COUNTER(&htim2);
        countsD = __HAL_TIM_GET_COUNTER(&htim5);

        // Display live values on OLED
        sprintf(buf, "%6d", countsA);
        OLED_ShowString(30, 10, (uint8_t *)buf);

        sprintf(buf, "%6d", countsD);
        OLED_ShowString(30, 20, (uint8_t *)buf);

        OLED_Refresh_Gram();

        // Also send over UART for logging
        sprintf(buf, "Counts A = %d | Counts D = %d\r\n", countsA, countsD);
        HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);

        HAL_Delay(100);  // update ~10 times/sec
    }
}



void Motor_forward(int pwmVal)
{
  // --- Static variables persist across calls ---
  static uint32_t last_time = 0;
  static float heading = 0.0f;
  static float target_heading = 0.0f;
  static uint8_t initialized = 0;

  static float integral = 0.0f;
  static float last_error = 0.0f;
  static float gz_filtered = 0.0f;

  // --- Timing ---
  uint32_t now = HAL_GetTick();
  float dt = (now - last_time) / 1000.0f; // ms -> seconds
  if (dt <= 0) dt = 0.001f;               // protect against div by 0
  last_time = now;

  if (!initialized) {
    target_heading = heading;   // lock current heading
    initialized = 1;
  }

  // --- Filter gyro Z-axis (yaw rate) ---
  float alpha = 0.9f; // closer to 1 = smoother
  gz_filtered = alpha * gz_filtered + (1.0f - alpha) * gz_dps;

  // --- Update heading from filtered gyro ---
  heading += gz_filtered * dt;   // integrate deg/s over time

  // --- Compute heading error ---
  float heading_error = target_heading - heading;
  integral += heading_error * dt;                // I term
  float derivative = (heading_error - last_error) / dt; // D term
  last_error = heading_error;

  // --- PID controller gains ---
  float Kp_h = 30.0f;   // proportional gain
  float Ki_h = 4.0f;    // integral gain
  float Kd_h = 3.5f;    // derivative gain

  // --- Control law (PID) ---
  int correction = (int)(Kp_h * heading_error +
                         Ki_h * integral +
                         Kd_h * derivative);

  // --- Clamp correction ---
  if (correction > 2000) correction = 2000;
  if (correction < -2000) correction = -2000;

  // --- Motor offset compensation (baseline bias) ---
  int left_offset  = -200;
  int right_offset = 0;

  // --- Apply correction + offsets ---
  int left_pwm  = pwmVal + left_offset  + correction;
  int right_pwm = pwmVal + right_offset - correction;

  // Clamp to valid PWM range
  if (left_pwm > pwmMax)  left_pwm = pwmMax;
  if (left_pwm < pwmMin)  left_pwm = pwmMin;
  if (right_pwm > pwmMax) right_pwm = pwmMax;
  if (right_pwm < pwmMin) right_pwm = pwmMin;

  // --- Send to motors ---
  // Motor A (left)
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, left_pwm);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);

  // Motor D (right)
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, right_pwm);

  // --- Debug (optional) ---
  // sprintf(buf, "Err=%.2f Corr=%d L=%d R=%d\r\n",
  //         heading_error, correction, left_pwm, right_pwm);
  // HAL_UART_Transmit(&huart3, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}



void Motor_reverse(int pwmVal)
{
  // Motor A: PWM on CH4, CH3 = 0
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, pwmVal);

  // Motor D: PWM on CH3, CH4 = 0
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, pwmVal);
  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_4, 0);

  sprintf(buf, "PWM = %4dR ", pwmVal);
  OLED_ShowString(0, 20, buf);
}

void serial_uart()
{
  // send various values to serial port @ usart 3 for display
  angle = (int)(position * 360 / 265); // Hall Sensor = 26 poles/13 pulses, DC motor = 20x13 = 260 pulses per revolution
                                       //  measured value = 265 pulses per revolution
  sprintf(buf, "%5d", angle);
  OLED_ShowString(60, 10, buf);
  // also send to serial port
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';                                      // comma separator
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", target_angle);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", error);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", pwmVal);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port
  OLED_ShowString(40, 20, buf);

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", error_area);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", error_change);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%5d", error_rate);
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port

  buf[0] = ',';
  HAL_UART_Transmit(&huart3, buf, 1, HAL_MAX_DELAY); // Send to Uart3 USB port

  sprintf(buf, "%4d ", speed);                       // RPM speed of the DC motor
  HAL_UART_Transmit(&huart3, buf, 5, HAL_MAX_DELAY); // Send to Uart3 USB port
  OLED_ShowString(40, 30, buf);
  OLED_Refresh_Gram();

  buf[0] = '\r';
  buf[1] = '\n';                                     // move to next line on serial port
  HAL_UART_Transmit(&huart3, buf, 2, HAL_MAX_DELAY); // Send through USB port
}

int _write(int file, char *ptr, int len)
{ // redirect printf to USART
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  for (int i = 0; i < len; i++)
  {
    ITM_SendChar(ptr[i]);
  }
  return len;
}

static uint16_t adc_read_channel(ADC_HandleTypeDef *hadc, uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES; // stable for GP2Y0A21YK
  HAL_ADC_ConfigChannel(hadc, &sConfig);

  // Throw away first sample after switch
      HAL_ADC_Start(hadc);
      HAL_ADC_PollForConversion(hadc, 10);
      (void)HAL_ADC_GetValue(hadc);

      HAL_ADC_Start(hadc);
      if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) return 0;
      uint16_t v = (uint16_t)HAL_ADC_GetValue(hadc);
      HAL_ADC_Stop(hadc);
      return v;
}

static inline float dist_cm_from_mv_5(uint32_t mv)
{
  const float A = 104.0f;         // mV
  const float B = 23730.0f;       // mV·cm
  if (mv <= (uint32_t)(A + 1.0f)) // avoid divide-by-zero / nonsense
    return 80.0f;                 // clamp to far limit
  float d = B / ((float)mv - A);
  // optional clamping to expected range (adjust to your setup)
  if (d < 4.0f)
    d = 4.0f;
  if (d > 80.0f)
    d = 80.0f;
  return d;
}

static inline float dist_cm_from_mv_4(uint32_t mv)
{
  const float A = 45.22f;         // mV
  const float B = 22380.0f;       // mV·cm
  if (mv <= (uint32_t)(A + 1.0f)) // avoid divide-by-zero / nonsense
    return 80.0f;                 // clamp to far limit
  float d = B / ((float)mv - A);
  // optional clamping to expected range (adjust to your setup)
  if (d < 4.0f)
    d = 4.0f;
  if (d > 80.0f)
    d = 80.0f;
  return d;
}

// IMU configuration
static void ICM20948_SelectBank(uint8_t bank)
{
  uint8_t d[2] = {REG_BANK_SEL, (uint8_t)(bank << 4)};
  HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, d, 2, HAL_MAX_DELAY);
}

static void ICM20948_WriteReg(uint8_t bank, uint8_t reg, uint8_t val)
{
  ICM20948_SelectBank(bank);
  uint8_t d[2] = {reg, val};
  HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, d, 2, HAL_MAX_DELAY);
}

static void ICM20948_ReadRegs(uint8_t bank, uint8_t reg, uint8_t *data, uint8_t len)
{
  ICM20948_SelectBank(bank);
  HAL_I2C_Master_Transmit(&hi2c2, ICM_ADDR, &reg, 1, HAL_MAX_DELAY);
  HAL_I2C_Master_Receive(&hi2c2, ICM_ADDR, data, len, HAL_MAX_DELAY);
}

// quick raw read (no bank select) used only by detector
static HAL_StatusTypeDef icm_read_raw(uint16_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
  HAL_StatusTypeDef s;
  s = HAL_I2C_Master_Transmit(&hi2c2, addr, &reg, 1, 100);
  if (s != HAL_OK)
    return s;
  return HAL_I2C_Master_Receive(&hi2c2, addr, data, len, 100);
}

// try 0x68 then 0x69, set ICM_ADDR if WHO_AM_I == 0xEA
int ICM20948_Detect(void)
{
  uint8_t who = 0;

  if (icm_read_raw(ICM_ADDR_68, WHO_AM_I, &who, 1) == HAL_OK && who == WHO_AM_I_VAL)
  {
    ICM_ADDR = ICM_ADDR_68;
    return 0;
  }
  if (icm_read_raw(ICM_ADDR_69, WHO_AM_I, &who, 1) == HAL_OK && who == WHO_AM_I_VAL)
  {
    ICM_ADDR = ICM_ADDR_69;
    return 0;
  }
  return -1; // not found on this I2C bus
}

// Init IMU
int ICM20948_Init(void)
{
  uint8_t whoami;
  ICM20948_ReadRegs(0, WHO_AM_I, &whoami, 1);
  if (whoami != 0xEA)
    return -1;

  // Reset device
  ICM20948_WriteReg(0, 0x06, 0x80); // DEVICE_RESET=1
  HAL_Delay(100);
  ICM20948_WriteReg(0, 0x06, 0x01); // wake + clock select

  // Enable accel + gyro
  ICM20948_WriteReg(0, 0x07, 0x00);
  ICM20948_WriteReg(0, 0x05, 0x00); // disable cycle mode

  // Set accel = ±2g, gyro = ±250 dps
  ICM20948_WriteReg(2, 0x14, 0x00); // accel config
  ICM20948_WriteReg(2, 0x01, 0x00); // gyro config

  return 0;
}

// Read raw accel/gyro
void ICM20948_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                      int16_t *gx, int16_t *gy, int16_t *gz)
{
  uint8_t d[12];
  ICM20948_ReadRegs(0, 0x2D, d, 12);
  *ax = (d[0] << 8) | d[1];
  *ay = (d[2] << 8) | d[3];
  *az = (d[4] << 8) | d[5];
  *gx = (d[6] << 8) | d[7];
  *gy = (d[8] << 8) | d[9];
  *gz = (d[10] << 8) | d[11];
}

// -- Magnetometer Functions -- //
// AK09916 Setup (via ICM-20948 I2C Master)
void AK09916_SetupI2CMaster(void)
{
    // 1. Enable I2C Master Mode (Bank 0, USER_CTRL = 0x03)
    // Set I2C_MST_EN (Bit 5, value 0x20)
    ICM20948_WriteReg(0, ICM_REG_USER_CTRL, 0x20);
    HAL_Delay(10);

    // 2. Set AK09916 Measurement Mode (using I2C SLAVE 4 to write a config byte)
    // The previous implementation of ICM20948_WriteReg handles bank selection.

    // a. Configure SLAVE 4 to write to the AK09916 (7-bit address 0x0C)
    ICM20948_WriteReg(ICM_BANK_3, ICM_REG_I2C_SLV4_ADDR, AK09916_I2C_ADDR);
    ICM20948_WriteReg(ICM_BANK_3, ICM_REG_I2C_SLV4_REG, AK_CNTL2);
    ICM20948_WriteReg(ICM_BANK_3, ICM_REG_I2C_SLV4_DO, AK_MODE_CONT_100HZ); // Data: 100Hz Continuous Mode

    // b. Execute the write (Enable SLV4 (0x80) + 1 byte (0x01))
    ICM20948_WriteReg(ICM_BANK_3, ICM_REG_I2C_SLV4_CTRL, 0x81);
    HAL_Delay(50); // Wait for the transaction to complete

    // 3. Configure I2C SLAVE 0 to continuously read magnetometer data

    // a. Configure SLAVE 0 to read from the AK09916
    ICM20948_WriteReg(ICM_BANK_3, ICM_REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); // 0x8C: Read bit + Mag Address
    ICM20948_WriteReg(ICM_BANK_3, ICM_REG_I2C_SLV0_REG, AK_HXL); // Start reading from HXL

    // b. Enable SLV0 (0x80) and read 8 bytes (0x08)
    // The 8 bytes are automatically transferred by the I2C Master to EXT_SENS_DATA_00
    ICM20948_WriteReg(ICM_BANK_3, ICM_REG_I2C_SLV0_CTRL, 0x88);

    // 4. Return to Bank 0 for Accel/Gyro reads
    ICM20948_SelectBank(0);
}


// Read raw magnetometer data
void ICM20948_ReadMagRaw(int16_t *mx, int16_t *my, int16_t *mz)
{
    uint8_t d[8];
    // Read 8 bytes from the external sensor data registers (Bank 0, 0x33)
    ICM20948_ReadRegs(0, ICM_REG_EXT_SENS_DATA_00, d, 8);

    // Combine bytes and subtract bias properly
    *mx = ((int16_t)((d[1] << 8) | d[0])); // X-axis
    *my = ((int16_t)((d[3] << 8) | d[2])); // Y-axis
    *mz = ((int16_t)((d[5] << 8) | d[4])); // Z-axis
}

void apply_mag_calibration(float *mx, float *my, float *mz) {
    float x = *mx - MAG_BIAS_X;
    float y = *my - MAG_BIAS_Y;
    float z = *mz - MAG_BIAS_Z;

    *mx = MAG_SOFT[0][0]*x + MAG_SOFT[0][1]*y + MAG_SOFT[0][2]*z;
    *my = MAG_SOFT[1][0]*x + MAG_SOFT[1][1]*y + MAG_SOFT[1][2]*z;
    *mz = MAG_SOFT[2][0]*x + MAG_SOFT[2][1]*y + MAG_SOFT[2][2]*z;
}

float get_mag_heading(float mx, float my) {
    return atan2f(my, mx); // radians
}

float complementary_heading(float gyro_dps, float mag_heading, float dt) {
    static float heading = 0.0f;  // persistent

    // Convert gyro to rad/s
    float gyro_rad = gyro_dps * (3.14159265f / 180.0f);

    // Integrate gyro
    heading += gyro_rad * dt;

    // Complementary filter: 0.98 gyro, 0.02 mag
    float alpha = 0.98f;
    heading = alpha*heading + (1.0f - alpha)*mag_heading;

    // Keep heading in [-pi, pi]
    if (heading > 3.14159265f) heading -= 2.0f*3.14159265f;
    if (heading < -3.14159265f) heading += 2.0f*3.14159265f;

    return heading;
}



// HCSR04 (Ultrasonic Sensor) Reading Function
uint32_t HCSR04_Read(void)
{
    uint32_t start_tick, stop_tick, pulse_length;

    // --- 1. Send 10 µs pulse on TRIG ---
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    for (volatile int i = 0; i < 300; i++); // ~10 µs delay @168 MHz
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);

    // --- 2. Wait for ECHO rising edge ---
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET);

    start_tick = DWT->CYCCNT;

    // --- 3. Wait for ECHO falling edge ---
    while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET);

    stop_tick = DWT->CYCCNT;

    // --- 4. Compute pulse length ---
    pulse_length = stop_tick - start_tick;

    // Convert to µs (SystemCoreClock = 168 MHz → 1 tick = 1/168 MHz = 5.95 ns)
    uint32_t time_us = pulse_length / (SystemCoreClock / 1000000);

    // Distance (cm) = (time_us * 0.0343) / 2
    return (uint32_t)((time_us * 343) / 20000);  // optimized integer math
}

//Checklist for checking obstacle sides
void orbit_from_front(float target_angle, int base_pwm, float steer_angle){

	uint8_t rxByte;
	uint8_t xPos = 0;       // Track X position (column)
	uint8_t yPos = 0;       // Track Y row (optional if you want wrapping by line)
	uint8_t charWidth = 8;  // Assume font width (adjust if your font differs)
	uint8_t while1 = 0;

	OLED_Clear();


	for (int i = 0; i<3; i++){
		//turn_by_angle_degrees(target_angle, base_pwm, steer_angle);

		//turn_by_angle_degrees(target_angle, base_pwm, steer_angle);
		const char *snap = "SNAP\n";//Send msg to RPI to capture now
		HAL_UART_Transmit(&huart3, (uint8_t*)snap,strlen(snap), HAL_MAX_DELAY);
		HAL_Delay(5000); //5 Seconds for camera to snap
		while (while1!= 1)
		    {
		        if (HAL_UART_Receive(&huart3, &rxByte, 1, HAL_MAX_DELAY) == HAL_OK)
		        {
		        	//const char *snap = "SNAP\n";//Send msg to RPI to capture now
		        	//HAL_UART_Transmit(&huart3, (uint8_t*)snap,strlen(snap), HAL_MAX_DELAY);
		            // Draw one character at current position
		            OLED_ShowChar(xPos, yPos, rxByte, 16, 1);  // size=16, mode=1 (adjust if needed)
		            OLED_Refresh_Gram();

		            if(rxByte == 'b'){
		            	break;
		            }
		            else{
		            	Motor_stop();
		            	exit(0);
		            }
		//TBD - Receive msg from camera before moving, have a timeout
		        }
		    }
		turn_by_angle_degrees(target_angle, base_pwm, steer_angle);
	}
}




/* USER CODE END 0 */


void log_magnetometer_data(void) {
	int16_t mx, my, mz;
    char buffer[100];
    sprintf(buf, "Y Bias: %d", MAG_BIAS_Y);
    OLED_ShowString(0, 30, (uint8_t*)buf);
    OLED_Refresh_Gram();
    HAL_Delay(1000);

    while (1) {
    	ICM20948_ReadMagRaw(&mx,&my,&mz);
    	float magX = (float)mx * 0.15f;
    	float magY = (float)my * 0.15f;
    	float magZ = (float)mz * 0.15f;

    	snprintf(buffer, sizeof(buffer), "%.3f,%.3f,%.3f\r\n", magX, magY, magZ);
    	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
        HAL_Delay(50); // 20 Hz sampling
    }
}



// 2. PID State (Required for persistence between loop iterations)
float g_heading_integral = 0.0f;
float g_heading_last_error = 0.0f;
float g_speed_integral = 0.0f;
float g_speed_last_error = 0.0f;
float g_fused_heading_rad = 0.0f; // Fused heading in RADIANS
uint32_t g_last_time = 0;
int32_t g_prev_left_counts = 0;
int32_t g_prev_right_counts = 0;
bool g_pid_initialized = false;

// 3. PID Constants
const float Kp_h = 40.0f, Ki_h = 5.0f, Kd_h = 3.5f;
const float Kp_e = 0.5f, Ki_e = 0.04f, Kd_e = 0.1f;

void pid_state_reset(void) {
    g_heading_integral = 0.0f;
    g_heading_last_error = 0.0f;
    g_speed_integral = 0.0f;
    g_speed_last_error = 0.0f;

    // Initialize fused heading to match current global heading
    g_fused_heading_rad = 0.0 * M_PI / 180.0f;

    g_last_time = HAL_GetTick();
    g_prev_left_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    g_prev_right_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    g_pid_initialized = true;
}

pid_control_cycle(float target_heading_rad, int base_pwm) {

    // --- Δt calculation ---
    uint32_t now = HAL_GetTick();
    float dt = (now - g_last_time) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    g_last_time = now;

    // --- Sensor Reading & Fused Heading Update ---
    int16_t mx_raw, my_raw, mz_raw;
    ICM20948_ReadMagRaw(&mx_raw, &my_raw, &mz_raw);
    float mx = (float)mx_raw, my = (float)my_raw, mz = (float)mz_raw;
    apply_mag_calibration(&mx, &my, &mz);

    int16_t gz_raw;
    ICM20948_ReadRaw(NULL, NULL, NULL, NULL, NULL, &gz_raw);
    float gz_dps = ((float)gz_raw / 131.0f) - gyro_bias;

    float mag_heading = atan2f(my, mx);
    g_fused_heading_rad += gz_dps * (M_PI / 180.0f) * dt;
    float alpha = 0.98f;
    g_fused_heading_rad = alpha * g_fused_heading_rad + (1.0f - alpha) * mag_heading;

    if (g_fused_heading_rad > M_PI) g_fused_heading_rad -= 2.0f * M_PI;
    if (g_fused_heading_rad < -M_PI) g_fused_heading_rad += 2.0f * M_PI;
    //g_robot_heading = g_fused_heading_rad * 180.0f / M_PI; // Update global state

    // --- Heading PID ---
    float heading_error = target_heading_rad - g_fused_heading_rad;
    if (heading_error > M_PI) heading_error -= 2.0f * M_PI;
    if (heading_error < -M_PI) heading_error += 2.0f * M_PI;
    g_heading_integral += heading_error * dt;
    float heading_derivative = (heading_error - g_heading_last_error) / dt;
    g_heading_last_error = heading_error;
    float gyro_correction = Kp_h * heading_error + Ki_h * g_heading_integral + Kd_h * heading_derivative;

    // --- Encoder PID (Speed Matching) & Odometry ---
    int32_t left_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
    int32_t right_counts = (int32_t)__HAL_TIM_GET_COUNTER(&htim5);
    int32_t left_delta = left_counts - g_prev_left_counts;
    int32_t right_delta = right_counts - g_prev_right_counts;

    //float incremental_dist_cm = counts_to_cm((abs(left_delta) + abs(right_delta)) / 2);
    g_prev_left_counts = left_counts;
    g_prev_right_counts = right_counts;

    //update_coordinates(incremental_dist_cm); // Update odometry state

    float speed_error = (float)(left_delta - right_delta);
    g_speed_integral += speed_error * dt;
    float speed_derivative = (speed_error - g_speed_last_error) / dt;
    g_speed_last_error = speed_error;
    float encoder_correction = Kp_e * speed_error + Ki_e * g_speed_integral + Kd_e * speed_derivative;

    // --- Apply Correction & Clamp ---
    int left_pwm_correction = (int)(0.7f * gyro_correction - 0.3f * encoder_correction);
    int right_pwm_correction = (int)(-0.7f * gyro_correction + 0.3f * encoder_correction);

    int left_pwm = base_pwm + left_pwm_correction;
    int right_pwm = base_pwm + right_pwm_correction;

    if (left_pwm > pwmMax) left_pwm = pwmMax;
    if (left_pwm < pwmMin) left_pwm = pwmMin;
    if (right_pwm > pwmMax) right_pwm = pwmMax;
    if (right_pwm < pwmMin) right_pwm = pwmMin;

    if (g_heading_integral > 500) g_heading_integral = 500;
    if (g_heading_integral < -500) g_heading_integral = -500;
    if (g_speed_integral > 50) g_speed_integral = 50;
    if (g_speed_integral < -50) g_speed_integral = -50;

    Motor_set_pwm(left_pwm, right_pwm);

    // --- OLED Update ---
    OLED_Clear();
    //sprintf(buf, "Hdg: %.1f X:%.1f Y:%.1f", g_robot_heading, g_robot_x, g_robot_y);
    //OLED_ShowString(0, 0, (uint8_t*)buf);
    sprintf(buf, "L:%d R:%d I_H:%.1f", left_pwm, right_pwm, g_heading_integral);
    OLED_ShowString(16, 0, (uint8_t*)buf);
    OLED_Refresh_Gram();

    HAL_Delay(10); // Control loop frequency

    //return incremental_dist_cm;
}


int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t *oled_buf; // buffer to store value to be display on OLED
  uint8_t i, status; // status for checking return

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  // Enable DWT Cycle Counter for precise timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0; // reset counter
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_I2C2_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  MotorDrive_enable();                       // enable PWM needed to drive MotroDrive A and D
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); // For servo
  // start TIM2/TIM5-Encoder to read Motor rotation in interrupt mode
  // Hall sensors produce 13 ticks/counts per turn, gear ratio = 20
  // 260 count per rotation of output (wheel)
  // 360 degree = 260 ticks/counts
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL); // Motor Drive A
  HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_ALL); // Motor Drive D
  // rpm = (int)((1000/no_of_tick) * 60/260 * 1/dt);  // For calculating motor rpm - by multiplying it with speed value
  // rpm = (1000.0f / (float)no_of_tick) * (60.0f / 260.0f);
  OLED_Init();

  uint8_t sbuf[] = "SC2104\n\r";                                 // send to serial port
  HAL_UART_Transmit(&huart3, sbuf, sizeof(sbuf), HAL_MAX_DELAY); // Send through Serial Port @115200

  OLED_Refresh_Gram();
  OLED_Clear();    // get display ready

  if (ICM20948_Detect() == 0)
  {
    sprintf(buf, "ICM @0x%02X", (unsigned)(ICM_ADDR >> 1));
  }
  else
  {
    sprintf(buf, "ICM NOT FOUND");
  }
  OLED_ShowString(0, 0, (uint8_t *)buf);
  OLED_Refresh_Gram();
  HAL_Delay(500);

  if (ICM20948_Init() == 0)
  {
    sprintf(buf, "ICM OK");
    AK09916_SetupI2CMaster();
  }
  else
  {
    sprintf(buf, "ICM FAIL");
  }
  OLED_ShowString(0, 10, (uint8_t *)buf);
  OLED_Refresh_Gram();
  HAL_Delay(500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  start = 1;                 // do a step response upon reset and power up
  MotorDrive_enable();       // enable PWM needed to drive MotroDrive A and D
  millisOld = HAL_GetTick(); // get time value before starting - for PID

  float target_heading;
  float target_heading_rad;


  Queue_Init();
  //HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buffer, COMMAND_SIZE);
  Steering_ToUS(0);


  // --- Gyro warm-up (reduce jerk from bad first samples) ---
  for (int i = 0; i < 50; i++) {   // ~500 ms
	  int16_t gz_raw;
      ICM20948_ReadRaw(NULL, NULL, NULL, NULL, NULL, &gz_raw);
      //gz_filtered = gz_raw / 131.0f;
      HAL_Delay(10);
  }

  float gyro_bias_sum = 0.0f;
    const int num_cal_samples = 700;
    for (int i = 0; i < num_cal_samples; i++) {
        int16_t gz_raw;
        ICM20948_ReadRaw(NULL, NULL, NULL, NULL, NULL, &gz_raw);
        gyro_bias_sum += gz_raw / 131.0f;
        HAL_Delay(5);
  }
  gyro_bias = gyro_bias_sum / num_cal_samples;

  // -- Path Sample Functions -- //

  OLED_Clear();



  //-------------------------------Task2-----------------------------------------
  int dir = 0; //1 for left, -1 for right

  //=======================Obstacle 1==================================
      //Only obstacle 1 can use diamond as it is a known 10x10 obstacle
  	Steering_ToUS(0);
  	int current_pwm = 2200;
  	int32_t dist_cm = 999;
  	dist_cm = HCSR04_Read();
  	pid_state_reset();
	target_heading_rad = 0.0f * M_PI / 180.0f;

	OLED_ShowString(48, 0, (uint8_t*)"1: Approach OBS1");
	OLED_Refresh_Gram();
	Motor_set_pwm(current_pwm, current_pwm); //Move till ultrasonic
  	while (dist_cm> 31) {
  		dist_cm = HCSR04_Read();
  		sprintf(buf, "%3lu cm   ", (unsigned long)dist_cm);
  		OLED_ShowString(24, 56, (uint8_t*)buf);
  		OLED_Refresh_Gram();
  		pid_control_cycle(target_heading_rad, current_pwm);
  	} //Stop when distance less than 35
  	Motor_stop();
  	HAL_Delay(200);
  	dist_cm = 999; //reset
  	char arrow1 = 'L'; //default right

  	//------------------------Get Camera response--------------------------------------

	// Send "SNAP" to RPi and wait for arrow
	const char *snap = "SNAP_\n";
	HAL_UART_Transmit(&huart3, (uint8_t*)snap, (uint16_t)strlen(snap), HAL_MAX_DELAY);
	uint8_t rxByte;
	// Define the expected word size and the buffers
	#define EXPECTED_WORD_SIZE 5
	char received_word[EXPECTED_WORD_SIZE + 1]; // +1 for the null terminator
	arrow1 = '\0'; // Variable to store 'L' or 'R'
	int while1 = 0;
	while (while1!= 1)
	{
		if (HAL_UART_Receive(&huart3, &rxByte, 1, HAL_MAX_DELAY) == HAL_OK)
		{
			OLED_ShowChar(10, 10, rxByte, 16, 1);  // size=16, mode=1 (adjust if needed)
			OLED_Refresh_Gram();

			if(rxByte == 'R'){
				arrow1 = 'R';
				break;
			}
			else if(rxByte == 'L'){
				arrow1 = 'L';
				break;
			}
		}
	}




		// If neither "LEFT_" nor "RIGHT" was received, the loop continues (goes back to HAL_UART_Receive)
		// and waits for the next set of 5 characters./ An infinite loop that is only exited by the 'break' statement
	//HAL_Delay(8000);

  	// ------------------------------------------------------------------------------------
  	// === Run diamond shaped using IR ===
  	if (arrow1 == 'R'){
  		dir = 1;
  	}
  	else if (arrow1 == 'L'){
  		dir = -1;
  	}
  	//Do manual turnings
  	
  	turn_by_angle_degrees(dir *25,2000,dir*30);
  	pid_state_reset();
  	int pwm = 1000;
  	Motor_forward(pwm);

  	uint16_t raw;
  	uint32_t mv;
  	float s;

  	// Inline IR read
	if (dir == 1) { //Turn left
		raw = adc_read_channel(&hadc1, ADC_CHANNEL_5); //Right IR sensor should get ready
		mv = (uint32_t)raw * 3300u / 4095u;
		s = dist_cm_from_mv_5(mv);
		sprintf(buf, "%3lu cm   ", (unsigned long)s);
		OLED_ShowString(24, 56, (uint8_t*)buf);
		OLED_Refresh_Gram();
		pid_control_cycle(target_heading_rad, pwm);
		HAL_Delay(10);
	} else { //Turn right
		raw = adc_read_channel(&hadc1, ADC_CHANNEL_4); //Left IR sensor get ready
		mv = (uint32_t)raw * 3300u / 4095u;
		s = dist_cm_from_mv_4(mv);
		sprintf(buf, "%3lu cm   ", (unsigned long)s);
		OLED_ShowString(24, 56, (uint8_t*)buf);
		OLED_Refresh_Gram();
		pid_control_cycle(target_heading_rad, pwm);
		HAL_Delay(10);
	}
  	while (s>15) { //Continue reading IR till it senses the obstacle
  		if (dir == 1) {
  			raw = adc_read_channel(&hadc1, ADC_CHANNEL_5);
  			mv = (uint32_t)raw * 3300u / 4095u;
  			s = dist_cm_from_mv_5(mv); //Get distance
  			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, pwm);
			HAL_Delay(10);
  		} else {
  			raw = adc_read_channel(&hadc1, ADC_CHANNEL_4);
  			mv = (uint32_t)raw * 3300u / 4095u;
  			s = dist_cm_from_mv_4(mv); //Get distance
  			sprintf(buf, "%3lu cm   ", (unsigned long)s);
  			OLED_ShowString(24, 56, (uint8_t*)buf);
  			OLED_Refresh_Gram();
  			pid_control_cycle(target_heading_rad, pwm);
  			HAL_Delay(10);
  		}
  	}
  	Motor_stop();


  	if (arrow1 == 'L'){ //Values that fckn works for left
  		turn_by_angle_degrees(dir * -35, 1500, dir * -25); //Straighten
  		run_straight_to_distance_cm_backward_MAG(10.0,2000);
  		turn_by_angle_degrees(dir * -30, 1500, dir * -25); //Straighten
  		//turn_by_angle_degrees(dir * 30, 1500, dir * 45); //Straighten
  		//run_straight_to_distance_cm_MAG(5.0, 1500);
  		turn_by_angle_degrees(dir * 10, 1500, dir * 30); //Straighten
  		run_straight_to_distance_cm_backward_MAG(8.0,2000);
  		turn_by_angle_degrees(dir * 10, 1500, dir * 30); //Straighten
  	}
  	else{
  		turn_by_angle_degrees(dir * -30, 1600, dir * -25); //Straighten
  		run_straight_to_distance_cm_backward_MAG(10.0,2000);
  		turn_by_angle_degrees(dir * -30, 1600, dir * -25); //Straighten
  		run_straight_to_distance_cm_MAG(5.0, 1500);
  		turn_by_angle_degrees(dir * 10, 1500, dir * 30); //Straighten
		run_straight_to_distance_cm_backward_MAG(9.0,2000);
		turn_by_angle_degrees(dir * 10, 1500, dir * 30); //Straighten
  	}
  	run_straight_to_distance_cm_backward_MAG(5.0,2000);
  	run_straight_to_distance_cm_backward_MAG(5.0,2000);
  	HAL_Delay(500);

  //=======================Obstacle 2==================================
  	//Motor S
  	char arrow2= 'L'; // default right
  	float target_dist=0;
  /*
  	HAL_UART_Transmit(&huart3, (uint8_t*)snap, (uint16_t)strlen(snap), HAL_MAX_DELAY);

	int while2 = 0;
	while (while2!= 1)
	{
		if (HAL_UART_Receive(&huart3, &rxByte, 1, HAL_MAX_DELAY) == HAL_OK)
		{
			OLED_ShowChar(10, 10, rxByte, 16, 1);  // size=16, mode=1 (adjust if needed)
			OLED_Refresh_Gram();

			if(rxByte == 'R'){
				arrow2 = 'R';
				break;
			}
			else if(rxByte == 'L'){
				arrow2 = 'L';
				break;
			}
		}
	}
*/
  	HAL_Delay(1000);
  	if (arrow2 == 'R'){
  		dir = 1;
  		target_dist = 30.0;

  	}
  	else if (arrow2 == 'L'){
  		dir = -1;
  		target_dist = 25.0;
  	}

  	float travelled_cm =0;

	// -----------------------------------------------------------------------------------

	//If we are too close to the second obstacle, move back till 50cm distance
	dist_cm = HCSR04_Read(); //Sanity Check
	if (dist_cm <target_dist){
		Motor_set_pwm_reverse(800,800);
		dist_cm = HCSR04_Read();
		while (dist_cm<target_dist) {
			dist_cm = HCSR04_Read();
			sprintf(buf, "%3lu cm   ", (unsigned long)dist_cm);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
		}
		Motor_stop();
		HAL_Delay(500);
	}
	else{ //Move forward till 30cm distance
		int32_t start_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
		pid_state_reset();
		current_pwm = 1500;
		Motor_forward(current_pwm);
		while (dist_cm>target_dist) {
			dist_cm = HCSR04_Read();
			sprintf(buf, "%3lu cm   ", (unsigned long)dist_cm);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, current_pwm);

		}
		int32_t cur_pos = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
		int32_t diff = cur_pos - start_pos;
		travelled_cm = counts_to_cm(abs(diff));
		Motor_stop();
		HAL_Delay(500);
	}
  	if (arrow2 == 'L'){ //Values that fckn works for left
		turn_by_angle_degrees(dir * 75, 1500, dir * 30); //Pause after first obstacle
	}
	else{
		turn_by_angle_degrees(dir * 65, 1600, dir * 40); //pause after second obstacle
	}
  	run_straight_to_distance_cm_backward_MAG(25.0,2000);
  	current_pwm = 1000;
  	pid_state_reset();
  	Motor_forward(current_pwm); //Move forward
  	// Inline IR read
	for(i=0;i<5;i++){ //Discard first 10
		if (dir == 1) { //Turn left
			raw = adc_read_channel(&hadc1, ADC_CHANNEL_5); //Right IR sensor should get ready
			mv = (uint32_t)raw * 3300u / 4095u;
			s = dist_cm_from_mv_5(mv);
			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, current_pwm);
			HAL_Delay(10);
		} else { //Turn right
			raw = adc_read_channel(&hadc1, ADC_CHANNEL_4); //Left IR sensor get ready
			mv = (uint32_t)raw * 3300u / 4095u;
			s = dist_cm_from_mv_4(mv);
			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, current_pwm);
			HAL_Delay(10);
		}
	}

  	while (s<15) { //Continue reading IR till it senses the exit point
  		if (dir > 0) { //if turn left
			raw = adc_read_channel(&hadc1, ADC_CHANNEL_5); //Right IR sensor should get ready
			mv = (uint32_t)raw * 3300u / 4095u;
			s = dist_cm_from_mv_5(mv);
			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, current_pwm);
			HAL_Delay(10);

		} else { //if turn right
			raw = adc_read_channel(&hadc1, ADC_CHANNEL_4); //Left IR sensor get ready
			mv = (uint32_t)raw * 3300u / 4095u;
			s = dist_cm_from_mv_4(mv);
			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, current_pwm);
			HAL_Delay(10);
		}
  	}
	//HAL_Delay(500);
	if(arrow2=='L'){
	  	turn_by_angle_degrees(dir * -70, 1500, dir * -40); //u turn to parallel against the obstacle
	  	run_straight_to_distance_cm_backward_MAG(15.0,2000);
	  	turn_by_angle_degrees(dir * -70, 1500, dir * -40);
	}
	else{
		turn_by_angle_degrees(dir * -70, 1500, dir * -30); //u turn to parallel against the obstacle
		Motor_reverse(1000);
		HAL_Delay(1500);
		turn_by_angle_degrees(dir * -75, 1500, dir * -30);
	}

	pid_state_reset();
	current_pwm = 2000;
  	Motor_forward(current_pwm); //Move forward
  	HAL_Delay(2000);
	// Inline IR read
	for(i=0;i<10;i++){ //Discard first 50
		if (dir == 1) { //Turn left
			raw = adc_read_channel(&hadc1, ADC_CHANNEL_5); //Right IR sensor should get ready
			mv = (uint32_t)raw * 3300u / 4095u;
			s = dist_cm_from_mv_5(mv);
			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			pid_control_cycle(target_heading_rad, current_pwm);
			OLED_Refresh_Gram();

			HAL_Delay(10);
		} else { //Turn right
			raw = adc_read_channel(&hadc1, ADC_CHANNEL_4); //Left IR sensor get ready
			mv = (uint32_t)raw * 3300u / 4095u;
			s = dist_cm_from_mv_4(mv);
			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, current_pwm);
			HAL_Delay(10);
		}
	}


	while (s < 15) { //Continue reading IR till it senses the exit point
		if (dir > 0) { //if turn left
			raw = adc_read_channel(&hadc1, ADC_CHANNEL_5); //Right IR sensor should get ready
			mv = (uint32_t)raw * 3300u / 4095u;
			s = dist_cm_from_mv_5(mv);
			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, current_pwm);
			HAL_Delay(10);

		} else { //if turn right
			raw = adc_read_channel(&hadc1, ADC_CHANNEL_4); //Left IR sensor get ready
			mv = (uint32_t)raw * 3300u / 4095u;
			s = dist_cm_from_mv_4(mv);
			sprintf(buf, "%3lu cm   ", (unsigned long)s);
			OLED_ShowString(24, 56, (uint8_t*)buf);
			OLED_Refresh_Gram();
			pid_control_cycle(target_heading_rad, current_pwm);
			HAL_Delay(10);
		}
	}
	turn_by_angle_degrees(dir * -70, 1500, dir * -30); //u turn to parallel against the obstacle, facing the start
	HAL_Delay(1000);

	//============================= Returning Algorithm ====================================
	OLED_Clear();

	sprintf(buf, "Travel: %.1fcm", travelled_cm);
	OLED_ShowString(0, 0,  (uint8_t*)buf);

	sprintf(buf, "Target: %.1fcm", target_dist);   // or target_cm if that's your var
	OLED_ShowString(0, 16, (uint8_t*)buf);

	float total_cm = travelled_cm + target_dist + 50.0;    // combined
	sprintf(buf, "Total : %.1fcm", total_cm);
	OLED_ShowString(0, 32, (uint8_t*)buf);

	OLED_Refresh_Gram();
	run_straight_to_distance_cm(total_cm,1000); //+10 for the total length of first obs
	turn_by_angle_degrees(dir * -70, 1500, dir * -30); //turn
	turn_by_angle_degrees(dir * 70, 1500, dir * 30); //turn to face starting
	pid_state_reset();
	current_pwm = 1000;
	Motor_forward(current_pwm);
	HAL_Delay(500);
	dist_cm = HCSR04_Read(); //Keep on turning until it sense the first obstacle
	while (dist_cm >10){
		dist_cm = HCSR04_Read();
		sprintf(buf, "%3lu cm   ", (unsigned long)dist_cm);
		OLED_ShowString(24, 56, (uint8_t*)buf);
		pid_control_cycle(target_heading_rad, current_pwm);
		OLED_Refresh_Gram();
	}
	Motor_stop();
  	HAL_Delay(50000);


	  Command_t next_command;
	  if (Queue_Dequeue(&next_command))
	  {
		  Execute_Command(&next_command);
	  }

    // Example: keep reading distance while idle
	  uint32_t distance = HCSR04_Read();

	  sprintf(buf, "Dist: %lu cm", distance);
	  OLED_ShowString(0, 60, (uint8_t*)buf);

	  int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
	  ICM20948_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);
	  ICM20948_ReadMagRaw(&mx, &my, &mz);

    // Scale
    ax_g = ax / 16384.0f;
    ay_g = ay / 16384.0f;
    az_g = az / 16384.0f;
    gx_dps = gx / 131.0f;
    gy_dps = gy / 131.0f;
    gz_dps = gz / 131.0f - gyro_bias;

//    sprintf(buf, "AX:%.2f AY:%.2f", ax_g, ay_g);
//    OLED_ShowString(0, 10, (uint8_t *)buf);
//
//    sprintf(buf, "AZ:%.2f", az_g);
//    OLED_ShowString(0, 20, (uint8_t *)buf);

    sprintf(buf, "GX:%.1f GY:%.1f", gx_dps, gy_dps);
    OLED_ShowString(0, 30, (uint8_t *)buf);

    sprintf(buf, "GZ:%.1f", gz_dps);
    OLED_ShowString(0, 40, (uint8_t *)buf);

    sprintf(buf, "Mag X:%d Y:%d Z:%d", mx, my, mz);
    OLED_ShowString(0, 10, (uint8_t *)buf); // Display on row 5

    OLED_Refresh_Gram();

    sprintf(buf, "AX=%.2f AY=%.2f AZ=%.2f | GX=%.1f GY=%.1f GZ=%.1f\r\n",
            ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
    //HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);

    HAL_GPIO_TogglePin(GPIOA, LED_Pin);
    HAL_Delay(500);

/*
        raw4 = adc_read_channel(&hadc1, ADC_CHANNEL_4);
        mv4  = (uint32_t)raw4 * 3300u / 4095u;
        dist4 = dist_cm_from_mv_4(mv4);

        // IR2 PA5 = ADC1_IN5
        raw5 = adc_read_channel(&hadc1, ADC_CHANNEL_5);
        mv5  = (uint32_t)raw5 * 3300u / 4095u;
        dist5 = dist_cm_from_mv_5(mv5);
        printf("IR4 = %.1f cm | IR5 = %.1f cm\r\n", dist4, dist5);
        // Forward fast
        Motor_forward(5000);   // ~60–70 RPM
        HAL_Delay(3000);
        Servo_WriteUS(2400); HAL_Delay(1000);  // right
        Servo_WriteUS(1000); HAL_Delay(1000); //centre
        Servo_WriteUS(500); HAL_Delay(1000); //left
        Motor_reverse(5000);   // ~60–70 RPM
        HAL_Delay(3000);

          //pwmVal 250  > RPM = 0
          //pwmVal 260 >  RPM = 1
          //pwmVal 270  > RPM = 2
          //pwmVal 300  > RPM 2
          //pwmVal 1000 > RPM = 10
          //pwmVal 3000 > RPM = 36
          //pwmVal 5000 > RPM ~ 60
          //pwmVal 6000 > RPM ~ 75

          OLED_ShowString(15, 40, "Motor Moving"); // show message on OLED display at line 40)
          OLED_Refresh_Gram();

*/

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  	  // while

  /* USER CODE END 3 */
}

// This callback is executed when a character is received by UART3.
volatile uint32_t rx_events = 0; // for debug
// Place this in main.c, outside of main()
/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  * the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // Check if the interrupt came from USART3
  if (huart->Instance == USART3)
  {
    // 1. Process the data by enqueuing the command
    // rx_buffer is a global variable holding the newly received 5 bytes

    Queue_Enqueue(rx_buffer);

    // 2. Restart the listener to wait for the next command
    // It is crucial to call HAL_UART_Receive_IT() again immediately to ensure
    // continuous reception and prevent missing data.
    HAL_UART_Receive_IT(&huart3, (uint8_t*)rx_buffer, COMMAND_SIZE);
  }

  // If you also use huart2 with interrupts, add an 'else if (huart->Instance == USART2)' block here
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
  RCC_OscInitStruct.PLL.PLLM = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 7199;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 19999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, OLED4_Pin|OLED3_Pin|OLED2_Pin|OLED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Buzzer_Pin|LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OLED4_Pin OLED3_Pin OLED2_Pin OLED1_Pin */
  GPIO_InitStruct.Pin = OLED4_Pin|OLED3_Pin|OLED2_Pin|OLED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : Buzzer_Pin LED_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin|LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : USER_PB_Pin IMU_INT_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin|IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // --- Ultrasonic HCSR04 Pins ---
  // Trig -> PB14

  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Echo -> PC9
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
//  GPIO_InitStruct.Pin = GPIO_PIN_15;
//  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
void OLED_show(void *argument, int y, int x) // display message on OLED panel
{
  //uint8_t hello[20]="Hello World";
  OLED_Init();
  OLED_Display_On();
//	OLED_ShowString(10,10,argument);
  OLED_ShowString(y, x, argument);
  OLED_Refresh_Gram();
}
*/

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
