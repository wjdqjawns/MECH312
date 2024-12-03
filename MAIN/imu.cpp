// #include <Arduino.h>
// #include <Wire.h>
// #include "MPU6050.h"
// #include "I2Cdev.h"

// // MPU6050 Objects
// MPU6050 mpuFront(0x69);
// MPU6050 mpuBack(0x68);

// float filtered_imuAngle = 0.0;
// const float imu_alpha = 0.9;

// // Pin Definitions
// #define AHALL_PIN 8
// #define BHALL_PIN 9
// #define PWM_PIN 11
// #define DIR_PIN 12
// #define ENABLE_PIN 10

// // Motor Variables
// long motorPulseCount = 0;
// const int pulsesPerRevolution = 256 * 4;  // EN25-ABZ-256 encoder
// float motorAngle;
// float error = 0;
// float last_error = 0;
// unsigned long prev_time = 0;
// float P_control = 0;
// float D_control = 0;
// float controller = 0;

// // Control Parameters
// float kp = 0.390;
// float kd = 2.7;
// float previous_output = 0;
// const float fs = 1000;
// const float alpha = 0.00018;

// // IMU Variables
// float imuAngle = 0.0;
// float reference_angle = 0.0;

// const int8_t stateTable[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// void motorEncoderISR()
// {
//     static uint8_t lastState = 0;
//     uint8_t newState = (digitalRead(AHALL_PIN) << 1) | digitalRead(BHALL_PIN);
//     uint8_t tableIndex = (lastState << 2) | newState;
//     int8_t direction = stateTable[tableIndex];
//     if(direction != 0)
//     {
//         motorPulseCount += direction;
//         motorAngle = (motorPulseCount * 360.0) / pulsesPerRevolution;
//     }
//     lastState = newState;
// }

// void setup()
// {
//     Serial.begin(115200);
//     Wire.begin();
    
//     // MPU6050 Setup
//     mpuFront.initialize();
//     mpuBack.initialize();
    
//     if (!mpuFront.testConnection() || !mpuBack.testConnection())
//     {
//         Serial.println("MPU connection failed");
//         while(1);
//     }
    
//     mpuFront.CalibrateAccel(6);
//     mpuFront.CalibrateGyro(6);
//     mpuBack.CalibrateAccel(6);
//     mpuBack.CalibrateGyro(6);
    
//     // Motor Setup
//     pinMode(AHALL_PIN, INPUT_PULLUP);
//     pinMode(BHALL_PIN, INPUT_PULLUP);
//     pinMode(PWM_PIN, OUTPUT);
//     pinMode(DIR_PIN, OUTPUT);
//     pinMode(ENABLE_PIN, OUTPUT);
//     digitalWrite(ENABLE_PIN, HIGH);
//     attachInterrupt(digitalPinToInterrupt(AHALL_PIN), motorEncoderISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(BHALL_PIN), motorEncoderISR, CHANGE);
// }

// void loop()
// {
//     static unsigned long lastTime = 0;
//     unsigned long currentTime = micros();

//     // Read MPU6050 data
//     int16_t ax1, ay1, az1, gx1, gy1, gz1;
//     int16_t ax2, ay2, az2, gx2, gy2, gz2;
    
//     mpuFront.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
//     mpuBack.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
    
//     // Calculate average accelerometer values
//     float axAvg = (ax1 + ax2) / 2.0;
//     float ayAvg = (ay1 + ay2) / 2.0;
//     float azAvg = (az1 + az2) / 2.0;
    
//     // Calculate raw IMU angle
//     float raw_imuAngle = atan2(ayAvg, sqrt(pow(axAvg, 2) + pow(azAvg, 2))) * 180 / PI;
    
//     // Apply IIR filter
//     filtered_imuAngle = (imu_alpha * raw_imuAngle) + ((1 - imu_alpha) * filtered_imuAngle);
    
//     // Use filtered IMU angle for reference calculation
//     reference_angle =  filtered_imuAngle;

//     if (currentTime - lastTime >= 1000000/600) {
//         lastTime = currentTime;
//         char buffer[32];
//         sprintf(buffer, "%.6f,%.6f", filtered_imuAngle, motorAngle);
//         Serial.println(buffer);
//     }
    
//     // P Control
//     error = (reference_angle - motorAngle)/1;
//     float LPF_input = reference_angle;
//     digitalWrite(DIR_PIN, error >= 0 ? LOW : HIGH);
    
//     float LPF_output = (alpha * LPF_input) + ((1 - alpha) * previous_output);
//     previous_output = LPF_output;
    
//     int limitedError = min(abs(error), 30);
//     P_control = map(limitedError, 0, 30, 40, 255) * kp;
    
//     controller = P_control;
//     controller = constrain(controller, 0, 100);
    
//     analogWrite(PWM_PIN, controller);
//     last_error = error;
// }