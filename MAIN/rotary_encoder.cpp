// #include <Arduino.h>

// #define ROTARY_CLK 2
// #define ROTARY_DT 3
// #define AHALL_PIN 8
// #define BHALL_PIN 9
// #define PWM_PIN 11
// #define DIR_PIN 12
// #define ENABLE_PIN 10

// int rotaryCounter = 0;
// uint8_t rotaryState = 0;
// long lastRotaryInterruptTime = 0;
// float rotaryAngle;
// long motorPulseCount = 0;
// const int pulsesPerRevolution = 256 * 4;  // EN25-ABZ-256 엔코더 기준
// float motorAngle;
// float error = 0;
// float last_error = 0;
// unsigned long prev_time = 0;
// float P_control = 0;
// float D_control = 0;
// float controller = 0;
// const int8_t stateTable[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
// const int rotaryDebounceTime = 1;

// float kp = 0.390;
// float kd = 2.7;
// float previous_output = 0;  // 이전 출력값 저장
// const float fs = 2500;  // 샘플링 주파수를 낮춤
// const float alpha = 0.000005;  // alpha 값을 직접 설정

// void rotaryEncoderISR() {
//     long currentInterruptTime = micros();
//     if (currentInterruptTime - lastRotaryInterruptTime < rotaryDebounceTime) {
//         return;
//     }
//     lastRotaryInterruptTime = currentInterruptTime;
//     uint8_t currentState = (digitalRead(ROTARY_CLK) << 1) | digitalRead(ROTARY_DT);
//     uint8_t index = (rotaryState << 2) | currentState;
//     int8_t stateChange = stateTable[index];
//     rotaryState = currentState;
//     if (stateChange != 0) {
//         rotaryCounter += stateChange;
//         rotaryAngle = 1*(rotaryCounter * 360.0) / pulsesPerRevolution;
//     }
// }

// void motorEncoderISR() {
//     static uint8_t lastState = 0;
//     uint8_t newState = (digitalRead(AHALL_PIN) << 1) | digitalRead(BHALL_PIN);
//     uint8_t tableIndex = (lastState << 2) | newState;
//     int8_t direction = stateTable[tableIndex];
//     if(direction != 0) {
//         motorPulseCount += direction;
//         motorAngle = (motorPulseCount * 360.0) / pulsesPerRevolution;
//     }
//     lastState = newState;
// }

// void setup() {
//     Serial.begin(115200);
//     // 로터리 엔코더 설정
//     pinMode(ROTARY_CLK, INPUT_PULLUP);
//     pinMode(ROTARY_DT, INPUT_PULLUP);
//     rotaryState = (digitalRead(ROTARY_CLK) << 1) | digitalRead(ROTARY_DT);
//     attachInterrupt(digitalPinToInterrupt(ROTARY_CLK), rotaryEncoderISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(ROTARY_DT), rotaryEncoderISR, CHANGE);
//     // 모터 및 모터 엔코더 설정
//     pinMode(AHALL_PIN, INPUT_PULLUP);
//     pinMode(BHALL_PIN, INPUT_PULLUP);
//     pinMode(PWM_PIN, OUTPUT);
//     pinMode(DIR_PIN, OUTPUT);
//     pinMode(ENABLE_PIN, OUTPUT);
//     digitalWrite(ENABLE_PIN, HIGH);
//     attachInterrupt(digitalPinToInterrupt(AHALL_PIN), motorEncoderISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(BHALL_PIN), motorEncoderISR, CHANGE);
// }

// void loop() {
//     static unsigned long lastTime = 0;
//     unsigned long currentTime = micros();

//     if (currentTime - lastTime >= 1000000/600) {
//         lastTime = currentTime;
//         char buffer[32];
//         sprintf(buffer, "%.6f,%.6f", rotaryAngle, motorAngle);
//         Serial.println(buffer);
//     }
    
//     // PD 제어 부분은 원래 속도로 계속 실행
//     error = (rotaryAngle - motorAngle)/1;
//     float LPF_input = rotaryAngle;
//     digitalWrite(DIR_PIN, error >= 0 ? LOW : HIGH);
    
//     float LPF_output = (alpha * LPF_input) + ((1 - alpha) * previous_output);
//     previous_output = LPF_output;
    
//     if (abs(rotaryAngle - LPF_output) > 30) {
//         kp = 0.155;
//         kd = 3.1;
//     }
//     if (abs(error) < 6) {
//         kp = 0.390;
//         kd = 2.6;
//     }
    
//     int limitedError = min(abs(error), 30);
//     P_control = map(limitedError, 0, 30, 44, 255) * kp;
    
//     float dt = 0.01;
//     D_control = ((error - last_error) / dt) * kd;
    
//     controller = P_control + D_control;
//     controller = constrain(controller, 0, 100);
    
//     analogWrite(PWM_PIN, controller);
//     last_error = error;
// }