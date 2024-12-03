#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"
#include "I2Cdev.h"
/* MPU6050 Objects */
MPU6050 mpu_front(0x69);
MPU6050 mpu_back(0x68);
/* Pin Definitions */
#define ROTARY_CLK 2
#define ROTARY_DT 3
#define AHALL_PIN 8
#define BHALL_PIN 9
#define PWM_PIN 11
#define DIR_PIN 12
#define ENABLE_PIN 10
/* IMU Variables */
float imu_angle = 0.0;
float filtered_imu_angle = 0;
const float imu_alpha = 0.01;
float imu_error = 0.0;
float imu_p = 0.0;
const float imu_kp = 1.0;
/* Rotary Encoder Variables */
int rotary_counter = 0;
uint8_t rotary_state = 0;
long rotary_last_interrupt_time = 0;
float rotary_angle = 0.0;
const int rotary_pulses_per_rev = 256 * 4;  // EN25-ABZ-256 encoder
const int rotary_debounce_time = 1;
/* Motor Variables */
float motor_angle = 0.0;
long motor_pulse_count = 0;
/* Control Variables */
float error = 0.0;
float last_error = 0.0;
float p_control = 0.0;
float d_control = 0.0;
float dt = 1;
float control_output = 0.0;
float desired_angle;
/* Filter Parameters */
const float filter_fs = 100;
const float filter_alpha = 0.1;
float filter_prev_output = 0.0;
/* Lookup Table */
const int8_t state_table[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
void rotaryEncoderISR() {
    long current_time = micros();
    if (current_time - rotary_last_interrupt_time < rotary_debounce_time) {
        return;
    }
    rotary_last_interrupt_time = current_time;
    uint8_t current_state = (digitalRead(ROTARY_CLK) << 1) | digitalRead(ROTARY_DT);
    uint8_t index = (rotary_state << 2) | current_state;
    int8_t state_change = state_table[index];
    rotary_state = current_state;
    if (state_change != 0) {
        rotary_counter += state_change;
        rotary_angle = 19*(rotary_counter * 360.0) / rotary_pulses_per_rev;
    }
}
void motorEncoderISR() {
    static uint8_t last_state = 0;
    uint8_t new_state = (digitalRead(AHALL_PIN) << 1) | digitalRead(BHALL_PIN);
    uint8_t table_index = (last_state << 2) | new_state;
    int8_t direction = state_table[table_index];
    if(direction != 0) {
        motor_pulse_count += direction;
        motor_angle = (motor_pulse_count * 360.0) / rotary_pulses_per_rev;
    }
    last_state = new_state;
}
void setup() {
    Serial.begin(115200);
    Wire.begin();
    /* MPU6050 Setup */
    mpu_front.initialize();
    mpu_back.initialize();
    if (!mpu_front.testConnection() || !mpu_back.testConnection()) {
        Serial.println("MPU connection failed");
        while(1);
    }
    mpu_front.CalibrateAccel(6);
    mpu_front.CalibrateGyro(6);
    mpu_back.CalibrateAccel(6);
    mpu_back.CalibrateGyro(6);
    /* Pin Setup */
    pinMode(ROTARY_CLK, INPUT_PULLUP);
    pinMode(ROTARY_DT, INPUT_PULLUP);
    pinMode(AHALL_PIN, INPUT_PULLUP);
    pinMode(BHALL_PIN, INPUT_PULLUP);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    /* Initial States */
    rotary_state = (digitalRead(ROTARY_CLK) << 1) | digitalRead(ROTARY_DT);
    digitalWrite(ENABLE_PIN, HIGH);
    /* Interrupt Setup */
    attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), rotaryEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_DT), rotaryEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(AHALL_PIN), motorEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BHALL_PIN), motorEncoderISR, CHANGE);
}
float kp = 0.150;
float kd = 1.5;
void loop() {
    static unsigned long last_print_time = 0;
    unsigned long current_time = micros();
    /* Serial Print */
    if (current_time - last_print_time >= 1000000/600) {
        last_print_time = current_time;
        char buffer[96];
        // sprintf(buffer, "%.6f,%.6f,%d,%.6f", motor_angle, desired_angle, filtered_imu_angle, rotary_angle);
        sprintf(buffer, "%.6f,%.6f,%.6f,%.6f", desired_angle, motor_angle, filtered_imu_angle, rotary_angle);
        Serial.println(buffer);
    }
    /* IMU Processing */
    int16_t ax1, ay1, az1, gx1, gy1, gz1;
    int16_t ax2, ay2, az2, gx2, gy2, gz2;
    mpu_front.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    mpu_back.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
    float ax_avg = (ax1 + ax2) / 2.0;
    float ay_avg = (ay1 + ay2) / 2.0;
    float az_avg = (az1 + az2) / 2.0;
    float raw_imu_angle = atan2(ay_avg, sqrt(pow(ax_avg, 2) + pow(az_avg, 2))) * 180 / PI;
    filtered_imu_angle = (imu_alpha * (round(raw_imu_angle*100))/100) + ((1 - imu_alpha) * filtered_imu_angle);
    desired_angle = 1*rotary_angle + 10*filtered_imu_angle;
    error = desired_angle - (motor_angle);
    /* Control Logic - Encoder */
    float filter_input = rotary_angle;
    float filter_output = (filter_alpha * filter_input) + ((1 - filter_alpha) * filter_prev_output);
    filter_prev_output = filter_output;
    /* Adaptive Control Parameters */
    if (abs(rotary_angle - filter_output) > 40) {
        kp = 0.010;
        kd = 0.001;
    }
    if (abs(error) < 10) {
        kp = 0.15;
        kd = 1.5;
    }
    /* PD Control */
    p_control = kp*error;
    d_control = ((error - last_error) / dt) * kd;
    control_output = (p_control + d_control);
    digitalWrite(DIR_PIN, control_output >= 0 ? LOW : HIGH);
    control_output = map(abs(control_output), 0, 25, 18, 255);
    control_output = constrain(control_output, 0, 240);
    analogWrite(PWM_PIN, control_output);
    last_error = error;
    // Serial.print("rota ");
    // Serial.print(rotary_angle);
    // Serial.print(("  fit"));
    // Serial.print((filter_output));
    // Serial.print(("   kp   "));
    // Serial.print(kp);
    // Serial.print(("   error   "));
    // Serial.println(error);
}