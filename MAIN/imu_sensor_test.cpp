// #include <Arduino.h>
// #include <Wire.h>
// #include "MPU6050.h"
// #include "I2Cdev.h"

// // RK4 필터 구조체 및 함수 정의
// struct State {
//     float angle;
//     float rate;
// };

// const float dt = 1.0/1000.0; // 샘플링 시간 (100Hz로 변경)
// State imuState = {0.0, 0.0};

// State derivative(State state, float input) {
//     State dstate;
//     dstate.angle = state.rate;
//     dstate.rate = input;
//     return dstate;
// }

// State rungeKutta4(State current, float input) {
//     State k1 = derivative(current, input);
    
//     State temp = current;
//     temp.angle += k1.angle * dt/2;
//     temp.rate += k1.rate * dt/2;
//     State k2 = derivative(temp, input);
    
//     temp = current;
//     temp.angle += k2.angle * dt/2;
//     temp.rate += k2.rate * dt/2;
//     State k3 = derivative(temp, input);
    
//     temp = current;
//     temp.angle += k3.angle * dt;
//     temp.rate += k3.rate * dt;
//     State k4 = derivative(temp, input);
    
//     State next;
//     next.angle = current.angle + (dt/6.0) * (k1.angle + 2*k2.angle + 2*k3.angle + k4.angle);
//     next.rate = current.rate + (dt/6.0) * (k1.rate + 2*k2.rate + 2*k3.rate + k4.rate);
    
//     return next;
// }

// // MPU6050 Objects
// MPU6050 mpu;

// void setup() {
//     Serial.begin(115200);
//     Wire.begin();
    
//     mpu.initialize();
    
//     if (!mpu.testConnection()) {
//         Serial.println("MPU connection failed");
//         while(1);
//     }
    
//     mpu.CalibrateAccel(6);
//     mpu.CalibrateGyro(6);
    
//     Serial.println("Time,RawAngle,FilteredAngle,GyroRate");
// }

// void loop() {
//     static unsigned long lastTime = 0;
//     unsigned long currentTime = millis();

//     if (currentTime - lastTime >= 1000/1000) { // 100Hz로 데이터 수집
//         lastTime = currentTime;

//         // IMU 데이터 읽기
//         int16_t ax, ay, az, gx, gy, gz;
//         mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
//         // 각도 계산
//         float raw_imuAngle = atan2(ay, sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI;
//         float gyroRate = gy / 131.0; // ±250°/s 범위에서의 변환 계수
        
//         // RK4 필터 적용
//         imuState = rungeKutta4(imuState, gyroRate);
        
//         // 데이터 출력
//         Serial.print(raw_imuAngle, 6);
//         Serial.print(",");
//         Serial.print(imuState.angle, 6);
//         Serial.print(",");
//         Serial.println(gyroRate, 6);
//     }
// }