#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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

void process_command(char *cmd) {
	HAL_UART_Transmit(&huart3, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    float fval1, fval2;
    int ival;

    // Reset PID state for forward motion
    Motor_forward_reset();

    if (strncmp(cmd, "motor_forward(", 14) == 0) {
        ival = atoi(cmd + 14);
        Motor_forward(ival);
    } else if (strncmp(cmd, "motor_reverse(", 14) == 0) {
        ival = atoi(cmd + 14);
        Motor_reverse(ival);
    } else if (strncmp(cmd, "servo_us(", 9) == 0) {
        ival = atoi(cmd + 9);
        Servo_WriteUS((uint16_t)ival);
    } else if (strncmp(cmd, "servo_deg(", 10) == 0) {
        ival = atoi(cmd + 10);
        Steering_ToUS(ival);
    } else if (strncmp(cmd, "stop", 4) == 0) {
        Motor_stop();
    } else if (sscanf(cmd, "straight_cm(%f,%d)", &fval1, &ival) == 2) {
        run_straight_to_distance_cm(fval1, ival);
    } else if (sscanf(cmd, "turn_arc_deg(%f,%d,%f)", &fval1, &ival, &fval2) == 3) {
        turn_by_angle_degrees(fval1, ival, fval2);
    } else if (sscanf(cmd, "turn_spot_deg(%f,%d)", &fval1, &ival) == 2) {
        turn_on_spot_degrees(fval1, ival);
    } else {
        // Handle unknown commands
        char msg[64];
        snprintf(msg, sizeof(msg), "Unknown command: %s\r\n", cmd);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
        return; // Exit the function to prevent the "Executed" message
    }

    // Send acknowledgement back to RPi
    char msg[64];
    snprintf(msg, sizeof(msg), "Executed: %s\r\n", cmd);
    HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
}

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
#define WHO_AM_I 0x00
#define WHO_AM_I_VAL 0xEA
#define REG_BANK_SEL 0x7F

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
/*
void Motor_forward(int pwmVal)
{
    // --- Timing ---
    uint32_t now = HAL_GetTick();
    float dt = (now - mf_last_time) / 1000.0f;
    if (dt <= 0) dt = 0.001f;   // prevent div/0
    mf_last_time = now;

    if (!mf_initialized) {
        mf_target_heading = mf_heading; // lock current heading
        mf_initialized = 1;
    }

    // --- Filter gyro (yaw rate) ---
    float alpha = 0.9f;
    mf_gz_filtered = alpha * mf_gz_filtered + (1.0f - alpha) * gz_dps;

    // --- Integrate heading ---
    mf_heading += mf_gz_filtered * dt;

    // --- PID calculations ---
    float heading_error = mf_target_heading - mf_heading;
    mf_integral += heading_error * dt;
    if (mf_integral > 1000) mf_integral = 1000;   // clamp integral
    if (mf_integral < -1000) mf_integral = -1000;

    float derivative = (heading_error - mf_last_error) / dt;
    mf_last_error = heading_error;

    float Kp_h = 30.0f;
    float Ki_h = 4.0f;
    float Kd_h = 3.5f;

    int correction = (int)(Kp_h * heading_error +
                           Ki_h * mf_integral +
                           Kd_h * derivative);

    if (correction > 2000) correction = 2000;
    if (correction < -2000) correction = -2000;

    // Offsets for motor balance
    int left_offset  = -200;
    int right_offset = 0;

    int left_pwm  = pwmVal + left_offset  + correction;
    int right_pwm = pwmVal + right_offset - correction;

    // Send to motors (reuse your Motor_set_pwm)
    Motor_set_pwm(left_pwm, right_pwm);
}
*/
void run_straight_to_distance_cm(float target_cm, int base_pwm)
{
    // --- Reset encoder ---
    int16_t start_pos = position;
    float travelled_cm = 0.0f;

    // --- PID state (non-static) ---
    float heading = 0.0f;
    float target_heading = 0.0f;
    float integral = 0.0f;
    float last_error = 0.0f;
    float gz_filtered = 0.0f;
    uint32_t last_time = HAL_GetTick();

    // --- Control parameters ---
    const float tol_cm = 1.0f;
    const float slow_down_cm = 5.0f;
    const float creep_cm = 1.0f;

    // --- PID gains ---
    float Kp_h = 40.0f;
    float Ki_h = 5.0f;
    float Kd_h = 3.5f;

    while (1)
    {
        // --- Distance travelled ---
        int16_t cur_pos = position;
        int32_t diff = (int32_t)cur_pos - (int32_t)start_pos;
        if (diff < 0) diff = -diff;
        travelled_cm = counts_to_cm(diff);

        float remaining = target_cm - travelled_cm;

        // Stop if within tolerance
        if (remaining <= tol_cm)
        {
            Motor_stop();
            break;
        }

        // --- PWM scaling ---
        int pwm_forward = base_pwm;
        if (remaining < slow_down_cm) pwm_forward = (int)(0.6f * base_pwm);
        if (remaining < creep_cm)     pwm_forward = pwmMin + 60;

        // --- Δt calculation ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_time = now;

        // --- Gyro filtering & heading integration ---
        gz_filtered = 0.9f * gz_filtered + 0.1f * gz_dps;
        heading += gz_filtered * dt;

        // --- PID calculation ---
        float error = target_heading - heading;
        integral += error * dt;
        if (integral > 1000) integral = 1000;
        if (integral < -1000) integral = -1000;
        float derivative = (error - last_error) / dt;
        last_error = error;

        int correction = (int)(Kp_h * error + Ki_h * integral + Kd_h * derivative);
        if (correction > 2000) correction = 2000;
        if (correction < -2000) correction = -2000;

        // --- Motor offsets + apply PID ---
        // Removed the hardcoded offsets and rely solely on the PID correction
        int left_pwm  = pwm_forward + correction;
        int right_pwm = pwm_forward - correction;

        // Clamp PWM
        if (left_pwm > pwmMax)  left_pwm = pwmMax;
        if (left_pwm < pwmMin)  left_pwm = pwmMin;
        if (right_pwm > pwmMax) right_pwm = pwmMax;
        if (right_pwm < pwmMin) right_pwm = pwmMin;

        Motor_set_pwm(left_pwm, right_pwm);

        // --- Optional OLED updates ---
        sprintf(buf, "Target: %.1fcm", target_cm);
        OLED_ShowString(0, 0, (uint8_t *)buf);
        sprintf(buf, "Travel: %.1fcm", travelled_cm);
        OLED_ShowString(0, 20, (uint8_t *)buf);
        OLED_Refresh_Gram();

        HAL_Delay(10); // ~100 Hz loop
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

/**
 * @brief Performs a zero-radius turn on the spot by spinning DC motors in opposite directions.
 *
 * This function uses a gyroscope to track the robot's heading and stops when
 * the target angle is reached.
 *
 * @param target_angle The total angle to turn in degrees.
 * Positive for right turns, negative for left turns.
 * (e.g., 90.0f to 360.0f or -90.0f to -360.0f)
 * @param base_pwm The base PWM value for the motors. A higher value results in a faster turn.
 */
void turn_on_spot_degrees(float target_angle, int base_pwm)
{
    // --- PID state variables ---
    float heading = 0.0f;
    float gz_filtered = 0.0f;
    uint32_t last_time = HAL_GetTick();

    // Raw sensor variables for reading
    int16_t ax, ay, az, gx, gy, gz;
    float gx_dps, gy_dps, gz_dps;

    // --- Determine motor directions for on-the-spot turn ---
    if (target_angle > 0)
    {
        // Right turn: Right wheel reverse, left wheel forward
        Motor_set_left_pwm(base_pwm, FORWARD);
        Motor_set_right_pwm(base_pwm, REVERSE);
    }
    else
    {
        // Left turn: Right wheel forward, left wheel reverse
        Motor_set_left_pwm(base_pwm, REVERSE);
        Motor_set_right_pwm(base_pwm, FORWARD);
    }

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



/*
void run_straight_to_distance_cm(float target_cm, int base_pwm)
{
    // --- Reset encoder ---
    int16_t start_pos = position;
    float travelled_cm = 0.0f;

    // --- Reset heading state ---
    float heading = 0.0f;
    float target_heading = 0.0f;
    float integral_h = 0.0f;
    float last_error_h = 0.0f;
    float gz_filtered = 0.0f;

    // --- Control parameters ---
    const float tol_cm = 1.0f;        // stop tolerance
    const float slow_down_cm = 5.0f;  // start slowing down
    const float creep_cm = 1.0f;      // final creep

    // --- PID gains (from Motor_forward) ---
    float Kp_h = 30.0f;
    float Ki_h = 4.0f;
    float Kd_h = 3.5f;

    // --- Time tracking ---
    uint32_t last_time = HAL_GetTick();

    while (1)
    {
        // --- Distance travelled ---
        int16_t cur_pos = position;
        int32_t diff = (int32_t)cur_pos - (int32_t)start_pos;
        if (diff < 0) diff = -diff;
        travelled_cm = counts_to_cm(diff);

        float remaining = target_cm - travelled_cm;

        // Stop if done
        if (remaining <= tol_cm)
        {
            Motor_stop();
            break;
        }

        // --- PWM scheduling for distance ---
        int pwm_forward = base_pwm;
        if (remaining < slow_down_cm) pwm_forward = (int)(0.6f * base_pwm);
        if (remaining < creep_cm)     pwm_forward = pwmMin + 60;

        // --- Δt ---
        uint32_t now = HAL_GetTick();
        float dt = (now - last_time) / 1000.0f;
        if (dt <= 0) dt = 0.001f;
        last_time = now;

        // --- Filter gyro yaw rate ---
        gz_filtered = 0.9f * gz_filtered + 0.1f * gz_dps;
        heading += gz_filtered * dt;

        // --- Heading PID ---
        float error_h = target_heading - heading;
        integral_h += error_h * dt;
        if (integral_h > 1000) integral_h = 1000;
        if (integral_h < -1000) integral_h = -1000;

        float derivative_h = (error_h - last_error_h) / dt;
        last_error_h = error_h;

        int correction = (int)(Kp_h * error_h + Ki_h * integral_h + Kd_h * derivative_h);
        if (correction > 2000) correction = 2000;
        if (correction < -2000) correction = -2000;

        // --- Motor offsets + correction ---
        int left_offset  = -200;
        int right_offset = 0;

        int left_pwm  = pwm_forward + left_offset  + correction;
        int right_pwm = pwm_forward + right_offset - correction;

        Motor_set_pwm(left_pwm, right_pwm);

        //HAL_Delay(10); // ~100 Hz loop

        sprintf(buf, "Target: %.1fcm", target_cm);
        OLED_ShowString(0, 0, (uint8_t *)buf);

        sprintf(buf, "Travel: %.1fcm", travelled_cm);
        OLED_ShowString(0, 20, (uint8_t *)buf);
    }

    // --- Final display on OLED ---
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
*/

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

  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, 10);
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

void uart_polling_task(void)
{
    uint8_t rxByte;
    uint8_t xPos = 0;       // Track X position (column)
    uint8_t yPos = 0;       // Track Y row (optional if you want wrapping by line)
    uint8_t charWidth = 8;  // Assume font width (adjust if your font differs)

    OLED_Clear();

    while (1)
    {
        if (HAL_UART_Receive(&huart3, &rxByte, 1, HAL_MAX_DELAY) == HAL_OK)
        {
            // Draw one character at current position
            OLED_ShowChar(xPos, yPos, rxByte, 16, 1);  // size=16, mode=1 (adjust if needed)
            OLED_Refresh_Gram();

            if(rxByte == 'f'){
            	run_straight_to_distance_cm(20.0,3000);
            }
            if(rxByte == 'r'){
            	turn_by_angle_degrees(90, 3000, 35);
            }

            // Advance cursor
            xPos += charWidth;

            // Wrap to next line if needed
            if (xPos >= 128) {
                xPos = 0;
                yPos += 16;   // move down by font height
                if (yPos >= 64) {
                    // Screen full → clear + reset
                    yPos = 0;
                    OLED_Clear();
                }
            }

            // Command buffer handling
            if (rxByte == ':')
            {
                cmd_index = 0;
                commandReady = 0;
            }
            else if (rxByte == ';')
            {
                cmd_buf[cmd_index] = '\0';
                commandReady = 1;

                // Reset screen + cursor for next string
                OLED_Clear();
                xPos = 0;
                yPos = 0;

                // Show final command string at top-left
                OLED_ShowString(0, 0, (uint8_t*)cmd_buf);
                OLED_Refresh_Gram();

                HAL_UART_Transmit(&huart3, (uint8_t*)cmd_buf, strlen(cmd_buf), HAL_MAX_DELAY);

                cmd_index = 0;
            }
            else if (cmd_index < CMD_BUF_LEN - 1)
            {
                cmd_buf[cmd_index++] = rxByte;
            }
            else
            {
                cmd_index = 0;
                commandReady = 0;
            }
        }
    }
}



//Checklist for checking obstacle sides
void orbit(float target_angle, int base_pwm, float steer_angle){
	//For current set up,: 68.0, 2000, 35.0 : it will turn 90 degrees clockwise 3x from starting
	//Distance btwn obstacle and bot is 1 iphone 13 pro max and a bit more away
	for (int i = 0; i<3; i++){
		turn_by_angle_degrees(target_angle, base_pwm, steer_angle);
		const char *snap = "SNAP\n";//Send msg to RPI to capture now
		HAL_UART_Transmit(&huart3, (uint8_t*)snap,strlen(snap), HAL_MAX_DELAY);
		HAL_Delay(5000); //5 Seconds for camera to snap
		while (1)
		    {
		        if (HAL_UART_Receive(&huart3, &rxByte, 1, HAL_MAX_DELAY) == HAL_OK)
		        {
		            // Draw one character at current position
		            OLED_ShowChar(xPos, yPos, rxByte, 16, 1);  // size=16, mode=1 (adjust if needed)
		            OLED_Refresh_Gram();

		            if(rxByte == 'bu'){
		            	break;
		            }
		            else{
		            	Motor_stop();
		            	exit(0);
		            }
		//TBD - Receive msg from camera before moving, have a timeout
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
  OLED_ShowString(10, 5, "SC2104/CE3002"); // show message on OLED display at line 5)
  OLED_ShowString(40, 30, "Lab 4");        // show message on OLED display at line 30)
  oled_buf = "Motor Control";              // anther way to show message through buffer
  OLED_ShowString(10, 50, oled_buf);       // another message at line 50

  uint8_t sbuf[] = "SC2104\n\r";                                 // send to serial port
  HAL_UART_Transmit(&huart3, sbuf, sizeof(sbuf), HAL_MAX_DELAY); // Send through Serial Port @115200
  HAL_UART_Transmit(&huart2, sbuf, sizeof(sbuf), HAL_MAX_DELAY); // Send through BT @9600

  OLED_Refresh_Gram();
  HAL_Delay(3000); // pause for 3 second to show message
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

  start = 0;
  angle = 0;
  target_angle = 1000; // rotate 1000 degree
  error = target_angle - angle;
  error_old = 0;
  error_area = 0;

  // motor drive here
  OLED_Clear();
  OLED_ShowString(0, 0, "Target: ");
  OLED_ShowString(0, 10, "Rotated: ");
  OLED_ShowString(0, 30, "RPM = ");
  sprintf(buf, "%4d", target_angle); // Hall Sensor = 26 poles/13 pulses, DC motor = 20x13 = 260 pulse per revolution
  OLED_ShowString(60, 0, buf);

  OLED_ShowString(15, 40, "Press User");    // show message on OLED display at line 40)
  OLED_ShowString(0, 50, "button to stop"); // show message on OLED display at line 50)
  OLED_Refresh_Gram();

  if (target_angle > 0) // Determine rotation direction)
    direction = 0;
  else
    direction = 1;

  start = 1;                 // do a step response upon reset and power up
  MotorDrive_enable();       // enable PWM needed to drive MotroDrive A and D
  millisOld = HAL_GetTick(); // get time value before starting - for PID


  //Calibrate_Encoder_Counts();
  //Motor_forward(3000);

  //run_straight_to_distance_cm(80.0,3000);
  //turn_by_angle_degrees(180.0, 3000, 90.0);
  //turn_on_spot_degrees(180.0, 3000);
  //Motor_forward(3000);

  //HAL_UART_Receive_IT(&huart3, &ch, 1);
  //Servo_WriteUS(2400); HAL_Delay(1000);  // right
  /*
  Steering_ToUS(-45);HAL_Delay(800);
  Steering_ToUS(-25);HAL_Delay(800);
  Steering_ToUS(-10);HAL_Delay(800);
  Steering_ToUS(0);HAL_Delay(800);
  Steering_ToUS(45);HAL_Delay(800);
  Steering_ToUS(25);HAL_Delay(800);
  Steering_ToUS(10);HAL_Delay(800);
  Steering_ToUS(0);HAL_Delay(800);
  */

  //Steering_ToUS(0);
  //HAL_Delay(1000);
  //turn_by_angle_degrees(-60.0,2000,45.0); //Only distance negative
  //HAL_Delay(500);
  //turn_by_angle_degrees(-60.0,2000,-45.0f);

  turn_by_angle_degrees(-65.0, 2000, 35.0); //Left
  Motor_stop();
  turn_by_angle_degrees(65.0,2000,-35.0); //Right
  Motor_stop();
  HAL_Delay(2000);
  orbit(68.0,2000,35.0);

  while (1) {

	//uart_polling_task();
	const char *msg = "SNAP\r\n";  // example message
	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

	if (commandReady)
	{
	        process_command((char*)cmd_buf);
	        commandReady = 0; // Reset flag for next command
	 }
	/*
    if (HAL_UART_Receive(&huart3, &ch, 20, 115200) == HAL_OK) {
        // Echo received char (optional)
        HAL_UART_Transmit(&huart3, &ch, 20, HAL_MAX_DELAY);

        if (ch == '\r' || ch == '\n') {
            if (cmd_index > 0) {
                cmd_buf[cmd_index] = '\0';  // Null-terminate

                // Show received command on OLED
                OLED_ShowString(0, 0, (const uint8_t*)cmd_buf);

                // Echo back the full command
                HAL_UART_Transmit(&huart3, (uint8_t*)cmd_buf, strlen(cmd_buf), HAL_MAX_DELAY);

                // Process the command
                process_command(cmd_buf);

                // Reset buffer index for next command
                cmd_index = 0;
            }
        } else {
            if (cmd_index < CMD_BUF_LEN - 1) {
                cmd_buf[cmd_index++] = ch;
            } else {
                // Overflow protection: reset if buffer full
                cmd_index = 0;
            }
        }
    }
    */


    // Example: keep reading distance while idle
	uint32_t distance = HCSR04_Read();

	sprintf(buf, "Dist: %lu cm", distance);
	OLED_ShowString(0, 50, (uint8_t*)buf);

    int16_t ax, ay, az, gx, gy, gz;
    ICM20948_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz);

    // Scale
    ax_g = ax / 16384.0f;
    ay_g = ay / 16384.0f;
    az_g = az / 16384.0f;
    gx_dps = gx / 131.0f;
    gy_dps = gy / 131.0f;
    gz_dps = gz / 131.0f;
/*
    sprintf(buf, "AX:%.2f AY:%.2f", ax_g, ay_g);
    OLED_ShowString(0, 10, (uint8_t *)buf);

    sprintf(buf, "AZ:%.2f", az_g);
    OLED_ShowString(0, 20, (uint8_t *)buf);

    sprintf(buf, "GX:%.1f GY:%.1f", gx_dps, gy_dps);
    OLED_ShowString(0, 30, (uint8_t *)buf);

    sprintf(buf, "GZ:%.1f", gz_dps);
    OLED_ShowString(0, 40, (uint8_t *)buf);

    OLED_Refresh_Gram();

    sprintf(buf, "AX=%.2f AY=%.2f AZ=%.2f | GX=%.1f GY=%.1f GZ=%.1f\r\n",
            ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps);
    //HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen(buf), HAL_MAX_DELAY);

    HAL_GPIO_TogglePin(GPIOA, LED_Pin);
    HAL_Delay(500);


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
  } // while

  /* USER CODE END 3 */
}

// This callback is executed when a character is received by UART3.

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // Check if the callback is from the correct UART handle
    if (huart->Instance == USART3)
    {
        // Your command-processing logic
        if (ch == '\0') {
            if (cmd_index > 0) {
                cmd_buf[cmd_index] = '\0';

                // Display the received command on the OLED
                OLED_ShowString(0, 0, (const uint8_t*)cmd_buf);
                HAL_UART_Transmit(&huart3, (uint8_t*)cmd_buf, strlen(cmd_buf), HAL_MAX_DELAY);

                // Process the full command
                process_command(cmd_buf);
                cmd_index = 0;
            }
        } else {
            if (cmd_index < CMD_BUF_LEN - 1) {
                cmd_buf[cmd_index++] = ch;
            }
        }

        // Re-arm the interrupt to listen for the next character
        HAL_UART_Receive_IT(&huart3, &ch, 1);
    }
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
  huart3.Init.BaudRate = 115200;
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
