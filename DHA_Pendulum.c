//=========================================================
// File Name: DHA_Pendulum.c
// MPU board: Raspberry Pi Pico
// Accelerometer + Gyro sensor: MPU6050
// UART Device   : FT231X(Akizuki)
// 
// 2023/08/24  written by @Tanuki_Bayashin
// 
// This Program is to Control the Inverted Pendulum System.
// version 1.0 (as a DHA_Pendulum.c)
// 
// Apply Mr.DHA Electronic Hobby Study Class
// for Raspberry Pi Pico.
//=========================================================

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "pico/binary_info.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"

//=========================================================
// Port Setting
// DigitalOut;
// I2C i2c(PB_9, PB_8);   //Gyro + Accelerometer (SDA, SCLK)
// Serial uart_usb(USBTX, USBRX); //UART (over USB)
//=========================================================

#define UART_ID uart0
#define BAUD_RATE 115200 // 115200 / 8[bit] / (1/0.01[sec]) = 144 characters

#define UART_TX_PIN 12    //(GPIO12 #16)
#define UART_RX_PIN 13    //(GPIO13 #17)

#define PICO_I2C_SDA_PIN 8 //(GPIO8 #11)
#define PICO_I2C_SCL_PIN 9 //(GPIO9 #12)

#define PI 3.1415926
#define ONE_G 16383  // 2^14 - 1 = 16383

// PWM Modules
#define SERVO_1 0 //(GPIO0 #1Pin PWM_A[0])
#define SERVO_2 3 //(GPIO3 #5Pin PWM_B[1])

#define LED_25 25           // LED_25

//=========================================================
//Accelerometer and gyro offset
//=========================================================
// variables for calculate offset
// for offset of accelaration
float acc_Y_offset = 0.0;
float acc_Z_offset = 0.0;
// for offset of angular velocity
float gyro_X_offset = 0.0;

float deg2rad = 0.0174533;  // 3.141593 / 180.0
float rad2deg = 57.296;     // 180.0 / 3.141593
// 姿勢オフセット
float deg_offset = 0.0;    // [deg]
float rad_offset = 0.055;    // [rad]

// 相補フィルタ
float angle_int = 0.0, angle_adj = 0.0;
float angle_adj_t, rad_e, rads_e;

//=========================================================
//Accelerometer and gyro statistical data
//=========================================================
// By default these devices are on bus address 0x68
static int addr = 0x68;

int16_t acceleration[3], gyro[3];
int16_t* temperature;
int sample_num = 1000;

float P_angle, P_angle_dot, P_angle_ave, P_angle_dot_ave;
float P_angle2;
float P_angle_data[10] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
float P_angle_dot_data[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

//=========================================================
// PWM to Mortor Voltage
// DATAs from a experiment
//=========================================================
int Motor_Mode = 0; // モーターが正転(2)か逆転(1)かSTOP(0)か
int Motor_Flag = 1; // Motor_Flag: 0 -> pwm_value must be 0 (未使用)
uint pwm1_slice_num, pwm2_slice_num;

float volts;
int pwm_value;

//=========================================================
// PWM to Mortor Voltage
// DATAs from a experiment
//=========================================================
//feed back gain
//(R=1000, Q = diag(1, 1, 10, 10), f=100Hz)
//float K[4] = { 28.37445212,  4.36987912,  0.13194284,  0.36487902 };

static uint32_t count_control = 0;
int cnt_main, cnt_uart;

//=========================================================
//Motor control variables
float feedback_rate = 0.02; //sec
struct repeating_timer timer_control, timer_standing, timer_Filter;
bool cancelled_control, cancelled_standing, cancelled_Filter;

static uint32_t t_start = 0, t_end = 0;
static uint32_t t_start_control, t_end_control;
static uint32_t t_start_Filter, t_end_Filter;

float speed_L, speed_R, adj_L, adj_R;
int duty_L, duty_R;

// コントロール
//float k1 = 8.0, k2 = 0.07;
float k1 = 4.5, k2 = 0.013;

//=========================================================
// Accelerometer (MPU6050)
// Gyroscope
//========================================================= 
//
#ifdef i2c_default
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = { 0x6B, 0x00 };
    i2c_write_blocking(i2c_default, addr, buf, 2, false);

    // set P_angle_dot ±500[degree/sec]
    buf[0] = 0x1B;
    buf[1] = 0x08;  // 0x01 => 0x08 changed(2022/08/11)
    i2c_write_blocking(i2c_default, addr, buf, 2, false);

    // set accelaration ±2G 
    buf[0] = 0x1C;
    buf[1] = 0x00;
    i2c_write_blocking(i2c_default, addr, buf, 2, false);

    // set Digital LPF
    // BW 44Hz(4.9ms delay) for Accel,
    // BW 42Hz(4.8ms delay) for Gyro
    buf[0] = 0x1A;
    buf[1] = 0x00;
    i2c_write_blocking(i2c_default, addr, buf, 2, false);

}

float get_P_angle() {

    uint8_t buffer[6];
    float sum;
    float x, y;

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    // true to keep master control of bus
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    acceleration[0] = (buffer[0] << 8) | buffer[1];
    acceleration[1] = (buffer[2] << 8) | buffer[3];
    acceleration[2] = (buffer[4] << 8) | buffer[5];

    // 86.03 is the value of compensation for initial posture
    x = (float)acceleration[1] - acc_Y_offset;
    y = (float)acceleration[2] - acc_Z_offset;

    P_angle = - (float)atan2((double)x, (double)y); // [rad]
    //P_angle *= rad2deg; // [deg]

    //if ((P_angle > 60.0) || (P_angle < -60.0)) { // -60 <= P_angle <= 60 [deg]
/*    if ((P_angle > 1.05) || (P_angle < -1.05)) { // -60 <= P_angle <= 60 [deg]
            P_angle = P_angle_data[0];
    }
*/
    for (int i = 0; i < 9; i++) {
        P_angle_data[9 - i] = P_angle_data[8 - i];
    }
    P_angle_data[0] = P_angle;

    sum = 0.0;
    for (int i = 0; i < 5; i++) { // 10 -> 5 ** 2023/07/23
        sum += P_angle_data[i];
    }

    P_angle_ave = sum / 5.0; // 10.0 -> 5.0 **

    return P_angle; // [rad]

}

float get_P_angle_dot() {

    uint8_t buffer[2];
    float sum;

    // Now gyro data from reg 0x43 for 6 bytes
    uint8_t val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    gyro[0] = (buffer[0] << 8) | buffer[1];

    // 2^15 - 1 = 32767, ±500 [degree/s] for Full range
    // -500を掛けているのは座標系を合わせるため
    P_angle_dot = (gyro[0] - gyro_X_offset) * (-500.0) / 32767.0; // [deg/s]
    P_angle_dot *= deg2rad; // [rad/s]

    //if ((P_angle_dot > 500.0) || (P_angle_dot < -500.0)) {
    if ((P_angle_dot > 8.72) || (P_angle_dot < -8.72)) { // 500[deg/s] = 8.72[rad/s]
        P_angle_dot = P_angle_dot_data[0];
    }

    for (int i = 0; i < 9; i++) {
        P_angle_dot_data[9 - i] = P_angle_dot_data[8 - i];
    }
    P_angle_dot_data[0] = P_angle_dot;

    sum = 0.0;
    for (int i = 0; i < 5; i++) {   // 10 -> 5 **
        sum += P_angle_dot_data[i];
    }

    P_angle_dot_ave = sum / 5.0;  // 10.0 -> 5.0 **

    return P_angle_dot;  // [rad/s]
}

#endif

void Calibration() {

    //// Calculate these variables
    // acc_Y_offset;
    // acc_Z_offset;
    // gyro_X_offset;

    float acc_Y_sum = 0.0;
    float acc_Z_sum = 0.0;
    float gyro_X_sum = 0.0;
    float ONE_G_sin, ONE_G_cos, P_angle_0;

    P_angle_0 = 90.0; // 2023/07/29 measured
    ONE_G_sin = ONE_G * sin(P_angle_0 * PI / 180.0);
    ONE_G_cos = ONE_G * cos(P_angle_0 * PI / 180.0);
    ///////////////////////////////////////////////
    // Calculate offset of acc and gyro raw data //
    // sample_num = 1000                          //
    ///////////////////////////////////////////////
    for (int i = 0; i < sample_num; i++) {
        P_angle = get_P_angle();
        P_angle_dot = get_P_angle_dot();

        // #define ONE_G 16383 = 2^14 - 1
        acc_Y_sum += acceleration[1] - ONE_G_cos;
        acc_Z_sum += acceleration[2] - ONE_G_sin;
        gyro_X_sum += gyro[0];

        // delay 500[μs]
        sleep_us(500);
    }

    // results of calculation of mean values
    acc_Y_offset = acc_Y_sum / sample_num;
    acc_Z_offset = acc_Z_sum / sample_num;
    gyro_X_offset = gyro_X_sum / sample_num;

    return;
}

//=========================================================
// Complement Filter Proccess
// (2023/09/15)
//=========================================================
bool Comple_Filter(struct repeating_timer* t)
{
    t_start_Filter = time_us_32();
    
    // 車体の角度と回転速度を計算する(相補フィルタ)

    P_angle = get_P_angle();            // ロボットの角度[rad]
    P_angle_dot = get_P_angle_dot();    // ロボットの角速度[rad/s]

    // 相補フィルタ
    angle_adj_t = angle_adj;
    angle_int += P_angle_dot_ave * 0.002;
    angle_adj = P_angle_ave - angle_int;
    angle_adj = angle_adj_t + 0.002 * (angle_adj - angle_adj_t) * 2.0;

    rad_e = angle_int + angle_adj;
    rads_e = P_angle_dot_ave;

    t_end_Filter = time_us_32() - t_start_Filter;

    return true;
}


//////////////////////////////////////////
// Calculate the velosity of servo motor
// (2023/09/15)
// 
//////////////////////////////////////////
bool Control_Pendulum(struct repeating_timer* t)
{
    //about 400usec passed for this process

    t_start_control = time_us_32();

    // モーターが出す速度の計算
    speed_L = ((rad_e - rad_offset) + rads_e * k2) * k1;

    // speedの最大値・最小値処理
    if (speed_L < -1.0) {
        speed_L = -1.0;
    }
    if (speed_L > 1.0) {
        speed_L = 1.0;
    }
    //speed_L = 0.0;

    //adj_L = -15.0;
    //adj_R = -20.0;
    
    adj_L = -5.0;
    adj_R = -5.0;


    if (speed_L <= -1.0) {
        speed_R = 1.0;
    }
    else if (speed_L < -0.4) {
        speed_R = -speed_L * 0.826 + 0.00353;

    }
    else if (speed_L < 0.0) {
        speed_R = -speed_L * 0.758 - 0.0299;
    }
    else if (speed_L == 0) {
        speed_R = 0.0;
    }
    else if (speed_L < 0.4) {
        speed_R = -speed_L * 0.8025 - 0.0267;
    }
    else if (speed_L < 1.0) {
        speed_R = -speed_L * 1.182 + 0.158;
    }
    else {
        speed_R = -1.0;
    }

    duty_L = (int)(speed_L * 244.1 + 732.3 + adj_L);  // モーターのスピードをDuty比に変換する
    duty_R = (int)(speed_R * 244.1 + 732.3 + adj_R); // モーターのスピードをDuty比に変換する

    // MAX:9764 
    /* レベル値(デューティカウント値)を設定(ここでは0) */
    pwm_set_chan_level(pwm1_slice_num, PWM_CHAN_A, duty_L);
    pwm_set_chan_level(pwm2_slice_num, PWM_CHAN_B, duty_R);

    t_end_control = time_us_32() - t_start_control;
    //t_end_control = 5000;

    return true;
}

void send_a_3times() {

    uart_puts(UART_ID, "a\n"); // 1st send

    sleep_ms(500);

    uart_puts(UART_ID, "a\n"); // 2nd send

    sleep_ms(500);

    uart_puts(UART_ID, "a\n"); // 3rd send
    sleep_ms(500);

    return;
}

//=========================================================
// Main
//=========================================================
void main() {

    static char s[100];
    int LED_cnt;
    float P_ratio, P_dot_ratio;

    //rad_offset = deg_offset * deg2rad;
    deg_offset = rad_offset * rad2deg;

    /////////////////////////
    // UART 初期設定       //
    /////////////////////////
    // process something important ?
    stdio_init_all();

    // Set up our UART with the required speed.
    uart_init(UART_ID, BAUD_RATE);

    // Set the TX and RX pins by using the function select on the GPIO
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    sleep_ms(100);
    send_a_3times();

    uart_puts(UART_ID, " Hello, UART!\n\r");

    /////////////////////////////
    // 加速度センサー 初期設定 //
    /////////////////////////////

#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c / mpu6050_i2c example requires a board with I2C pins
    uart_puts(UART_ID, "Default I2C pins were not defined");
#else
    uart_puts(UART_ID, "Hello, MPU6050! Reading raw data from registers...");

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_I2C_SDA_PIN);
    gpio_pull_up(PICO_I2C_SCL_PIN);

    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_I2C_SDA_PIN, PICO_I2C_SCL_PIN, GPIO_FUNC_I2C));

    mpu6050_reset();

#endif
    uart_puts(UART_ID, " mpu6050 reset!\n\r");

    /////////////////////////////////////////
    // GPIO(for LEDs and BUTTONs) initialize
    /////////////////////////////////////////
    gpio_init(LED_25);
    gpio_set_dir(LED_25, GPIO_OUT);

    ///////////////////////////////////
    // Calibration Start
    ///////////////////////////////////

    Calibration();

    snprintf(s, sizeof(s), "%5.3f,%5.3f,%5.3f\n", 
                                acc_Y_offset, acc_Z_offset, gyro_X_offset);
    uart_puts(UART_ID, s);
    sleep_ms(100);

    
    /////////////////////////
    // A/D 変換の処理      //
    /////////////////////////  ×（A/D 変換はお休み）
    // ADC module initialize
    //adc_init();
    // Make sure GPIO is high-impedance, no pullups etc
    //adc_gpio_init(26);
    //adc_gpio_init(27);
    //adc_gpio_init(28);

    // Select ADC input 0 (GPIO26)
    //adc_select_input(0);

    ///////////////////////////
    // PWM の前処理          //
    // (Motor_IN1, Motor_IN2)//
    ///////////////////////////
    /* GPIOにPWMを割り当て */
    gpio_set_function(SERVO_1, GPIO_FUNC_PWM);
    gpio_set_function(SERVO_2, GPIO_FUNC_PWM);

    pwm1_slice_num = pwm_gpio_to_slice_num(SERVO_1);
    pwm2_slice_num = pwm_gpio_to_slice_num(SERVO_2);

    /* clkdiv と wrap を指定 */
    //pwm frequency: 50[Hz]
    pwm_set_clkdiv(pwm1_slice_num, 610.375);
    pwm_set_wrap(pwm1_slice_num, 9764);

    pwm_set_clkdiv(pwm2_slice_num, 610.375);
    pwm_set_wrap(pwm2_slice_num, 9764);

    /* レベル値(デューティカウント値)を設定(ここでは0) */
    pwm_set_chan_level(pwm1_slice_num, PWM_CHAN_A, 732 - 5);
    pwm_set_chan_level(pwm2_slice_num, PWM_CHAN_B, 732 - 5);

    /* pwm0 start */
    pwm_set_enabled(pwm1_slice_num, true);
    pwm_set_enabled(pwm2_slice_num, true);
    /* END of PWM Module Setting */

    /////////////////////////////////////////////  
    //  Timer
    /////////////////////////////////////////////

    // calculate P_angle [deg] as initial values
    P_angle_data[0] = get_P_angle();
    for (int i = 1; i < 10; i++) {
        P_angle_data[i] = P_angle_data[0];
    }

    // calculate P_angle_dot [deg/s] as initial values
    P_angle_dot_data[0] = get_P_angle_dot();
    for (int i = 1; i < 10; i++) {
        P_angle_dot_data[i] = P_angle_dot_data[0];
    }

    // start main control process (2.0 ms)
    add_repeating_timer_ms(-2, Comple_Filter, NULL, &timer_Filter);
    sleep_ms(1);
    
    // start main control process (20.0 ms)
    add_repeating_timer_ms(-20, Control_Pendulum, NULL, &timer_control);
    sleep_ms(1);

    uart_puts(UART_ID, "Start!!\n");
    sleep_ms(1);
 
    //===========================================
    //Main loop
    //it takes 10 msec (calculation)
    //===========================================

    LED_cnt = 0;
    while(1)
    {
        t_start = time_us_32();
        
        //snprintf(s, sizeof(s), "%4d, %4d, %6d, %6d, %6d, %4.3f\n", t_end, t_end_control, acceleration[1], acceleration[2],
        //                                                 gyro[0], P_angle_ave);
        //snprintf(s, sizeof(s), "%d, %d, %4.3f, %4.3f\n", t_end, t_end_control, P_angle_ave, P_angle_dot_ave);
        //P_ratio = (rad_e / P_angle_ave - 1.0) * 100.0;
        //P_dot_ratio = rads_e / P_angle_dot_ave - 1.0;
        //snprintf(s, sizeof(s), "%4d, %4.3f, %4.3f, %4.3f, %4.3f\n", t_end_Filter, rad_e, rads_e, P_ratio, P_dot_ratio);
        //snprintf(s, sizeof(s), "%4d, %4.3f, %4.3f, %4.3f\n", t_end_Filter, P_angle_ave, angle_int, angle_adj);
        //snprintf(s, sizeof(s), "%4d, %4.3f, %4.3f, %4.3f\n", t_end_Filter, P_angle_ave, rad_e, P_ratio);
        //snprintf(s, sizeof(s), "%4.3f, %4.3f\n", P_angle_ave, rad_e);
        //snprintf(s, sizeof(s), "%5d, %4d, %4d, %4.3f, %4.3f\n", t_end, t_end_control, t_end_Filter, rad_e, rads_e);
        snprintf(s, sizeof(s), "%4.3f, %4.3f, %4.3f, %4.3f\n", rad_e, rads_e, speed_L, speed_R);
        uart_puts(UART_ID, s);

        LED_cnt++;
        if (LED_cnt > 50) {
            LED_cnt = 0;
        }
        if (LED_cnt > 25) {
            gpio_put(LED_25, 1); // LED_25 is ON
        }
        else {
            gpio_put(LED_25, 0); // LED_25 is OFF
        }


        sleep_ms(20);

        t_end = time_us_32() - t_start;
    }
    //===========================================
    //Main loop (end)
    //=========================================== 

    cancelled_control = cancel_repeating_timer(&timer_control);
    uart_puts(UART_ID, "cancelled... _control");
    snprintf(s, sizeof(s), "%d", cancelled_control);
    uart_puts(UART_ID, s);
    uart_puts(UART_ID, "\0");

    sleep_ms(1);

    cancelled_Filter = cancel_repeating_timer(&timer_Filter);
    uart_puts(UART_ID, "cancelled... _Filter");
    snprintf(s, sizeof(s), "%d", cancelled_Filter);
    uart_puts(UART_ID, s);
    uart_puts(UART_ID, "\0");

    sleep_ms(1);

    return;
}
