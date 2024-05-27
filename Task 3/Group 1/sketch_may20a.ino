#include <Wire.h>
#include <MPU6050.h>
#include "Kalman.h"

MPU6050 mpu;
Kalman kalmanX;
Kalman kalmanY;

// Variables for raw data
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Variables for angle calculation
double accAngleX, accAngleY;
double gyroAngleX, gyroAngleY;
double kalAngleX, kalAngleY;

// Constants for calibration
const int numReadings = 100;
double gyroXoffset, gyroYoffset, gyroZoffset;

unsigned long timer;

void setup() {
    Serial.begin(9600);
    Wire.begin();
    mpu.initialize();

    // Check if the sensor is connected
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed");
        while (1);
    }

    // Calibration
    calibrateSensor();

    timer = micros();
}

// void loop() {
//     // Read raw data
//     mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

//     // Convert to acceleration in g's
//     double accX = (double)ax / 16384.0;
//     double accY = (double)ay / 16384.0;
//     double accZ = (double)az / 16384.0;

//     // Calculate angles
//     accAngleX = (atan(accY / sqrt(accX * accX + accZ * accZ)) * 180 / PI);
//     accAngleY = (atan(-1 * accX / sqrt(accY * accY + accZ * accZ)) * 180 / PI);

//     // Gyro rates in degrees per second
//     double gyroXrate = ((double)gx - gyroXoffset) / 131.0;
//     double gyroYrate = ((double)gy - gyroYoffset) / 131.0;

//     // Calculate delta time
//     double dt = (double)(micros() - timer) / 1000000;
//     timer = micros();

//     // Kalman filter
//     kalAngleX = kalmanX.getAngle(accAngleX, gyroXrate, dt);
//     kalAngleY = kalmanY.getAngle(accAngleY, gyroYrate, dt);

//     // Print the results
//     Serial.print("Kalman X: "); Serial.print(kalAngleX); Serial.print(" | ");
//     Serial.print("Kalman Y: "); Serial.println(kalAngleY);

//     delay(10);
// }
void loop() {
    // Read raw data
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert to acceleration in g's
    double accX = (double)ax / 16384.0;
    double accY = (double)ay / 16384.0;
    double accZ = (double)az / 16384.0;

    // Calculate angles
    accAngleX = (atan(accY / sqrt(accX * accX + accZ * accZ)) * 180 / PI);
    accAngleY = (atan(-1 * accX / sqrt(accY * accY + accZ * accZ)) * 180 / PI);

    // Gyro rates in degrees per second
    double gyroXrate = ((double)gx - gyroXoffset) / 131.0;
    double gyroYrate = ((double)gy - gyroYoffset) / 131.0;

    // Calculate delta time
    double dt = (double)(micros() - timer) / 1000000;
    timer = micros();

    // Kalman filter
    kalAngleX = kalmanX.getAngle(accAngleX, gyroXrate, dt);
    kalAngleY = kalmanY.getAngle(accAngleY, gyroYrate, dt);

    // Print the results in a table-like format
    Serial.println("==========================================");
    Serial.print("Acc X: "); Serial.print(accX, 4); Serial.print("\t");
    Serial.print("Acc Y: "); Serial.print(accY, 4); Serial.print("\t");
    Serial.print("Acc Z: "); Serial.print(accZ, 4); Serial.print("\t");

    Serial.print("Gyro X: "); Serial.print(gyroXrate, 4); Serial.print("\t");
    Serial.print("Gyro Y: "); Serial.print(gyroYrate, 4); Serial.print("\t");
    Serial.print("Gyro Z: "); Serial.print((double)(gz - gyroZoffset) / 131.0, 4); Serial.print("\t");

    Serial.print("Kalman X: "); Serial.print(kalAngleX, 4); Serial.print("\t");
    Serial.print("Kalman Y: "); Serial.println(kalAngleY, 4);
    Serial.println("==========================================");

    delay(100);
}
void calibrateSensor() {
    long gyroXsum = 0, gyroYsum = 0, gyroZsum = 0;

    for (int i = 0; i < numReadings; i++) {
        mpu.getRotation(&gx, &gy, &gz);
        gyroXsum += gx;
        gyroYsum += gy;
        gyroZsum += gz;
        delay(10);
    }

    gyroXoffset = (double)gyroXsum / numReadings;
    gyroYoffset = (double)gyroYsum / numReadings;
    gyroZoffset = (double)gyroZsum / numReadings;
}
