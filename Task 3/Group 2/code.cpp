#include <Wire.h>
#include <MPU6050.h>
#include <Kalman.h>

MPU6050 mpu;
Kalman kalmanAx; // Kalman filter for accelerometer X-axis
Kalman kalmanAy; // Kalman filter for accelerometer Y-axis
Kalman kalmanAz; // Kalman filter for accelerometer Z-axis
Kalman kalmanGx; // Kalman filter for gyroscope X-axis
Kalman kalmanGy; // Kalman filter for gyroscope Y-axis
Kalman kalmanGz; // Kalman filter for gyroscope Z-axis


uint32_t timer;
float dt; // Time difference

// Raw data variables
int16_t ax, ay, az;
int16_t gx, gy, gz;

// Filtered data variables
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Wait for sensor to stabilize
  delay(1000);

  // Initialize timer
  timer = millis();
}

void loop() {
  // Read raw data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate dt
  dt = (double)(millis() - timer) / 1000;
  timer = millis();

  // Convert raw values to actual units
  double accXraw = ax / 16384.0;
  double accYraw = ay / 16384.0;
  double accZraw = az / 16384.0;
  double gyroXrate = gx / 131.0;
  double gyroYrate = gy / 131.0;
  double gyroZrate = gz / 131.0;

  // Apply Kalman filter
  
  accX = kalmanAx.getAngle(accXraw, 0, dt); // Assuming no rate for accelerometer
  accY = kalmanAy.getAngle(accYraw, 0, dt);
  accZ = kalmanAz.getAngle(accZraw, 0, dt);
  gyroX = kalmanGx.getAngle(gyroXrate, 0, dt);
  gyroY = kalmanGy.getAngle(gyroYrate, 0, dt);
  gyroZ = kalmanGz.getAngle(gyroZrate, 0, dt);
  

  // Print the filtered values
  Serial.print("AccX: "); Serial.print(accX); Serial.print(" ");
  Serial.print("AccY: "); Serial.print(accY); Serial.print(" ");
  Serial.print("AccZ: "); Serial.print(accZ); Serial.print(" ");
  Serial.print("GyroX: "); Serial.print(gyroX); Serial.print(" ");
  Serial.print("GyroY: "); Serial.print(gyroY); Serial.print(" ");
  Serial.print("GyroZ: "); Serial.println(gyroZ); 
 

  delay(3);
}