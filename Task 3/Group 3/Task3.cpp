#include <Wire.h>
#include <math.h>

class KalmanFilter {
public:
  KalmanFilter(float q, float r, float p, float intial_value) {
    Q = q;
    R = r;
    P = p;
    X = intial_value;
  }

  float updateEstimate(float measurement) {
    // Prediction update
    P = P + Q;

    // Measurement update
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;

    return X;
  }

private:
  float Q; // Process noise covariance
  float R; // Measurement noise covariance
  float P; // Estimation error covariance
  float K; // Kalman gain
  float X; // Estimated value
};

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

float pitch = 0, roll = 0, yaw = 0;
unsigned long prevTime;

KalmanFilter kalmanX(0.001, 0.1, 0.1, 0);
KalmanFilter kalmanY(0.001, 0.1, 0.1, 0);
KalmanFilter kalmanZ(0.001, 0.1, 0.1, 0);

KalmanFilter kalmanGyroX(0.001, 0.1, 0.1, 0);
KalmanFilter kalmanGyroY(0.001, 0.1, 0.1, 0);
KalmanFilter kalmanGyroZ(0.001, 0.1, 0.1, 0);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  prevTime = millis();
}

void loop() {
  recordAccelRegisters();
  recordGyroRegisters();
  calculatePitchRollYaw();
  printData();
  delay(100);
}

void setupMPU() {
  Wire.beginTransmission(0x68); // 0x68 is the I2C address of the MPU6050
  Wire.write(0x6B); // Accessing the register 6B - Power Management
  Wire.write(0x00); // Setting SLEEP register to 0 (wake up MPU6050)
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68); // I2C address of the MPU6050
  Wire.write(0x1B); // Accessing the register 1B - Gyroscope Configuration
  Wire.write(0x00); // Setting the gyro to full scale +/- 250 deg/s
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68); // I2C address of the MPU6050
  Wire.write(0x1C); // Accessing the register 1C - Accelerometer Configuration
  Wire.write(0x00); // Setting the accel to +/- 2g
  Wire.endTransmission();
}

void recordAccelRegisters() {
  Wire.beginTransmission(0x68); // I2C address of the MPU6050
  Wire.write(0x3B); // Starting register for Accel Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6); // Request 6 bytes of Accel data
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read(); // Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read(); // Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read(); // Store last two bytes into accelZ
  processAccelData();
}

void processAccelData() {
  gForceX = kalmanX.updateEstimate(accelX / 16384.0);
  gForceY = kalmanY.updateEstimate(accelY / 16384.0);
  gForceZ = kalmanZ.updateEstimate(accelZ / 16384.0);
}

void recordGyroRegisters() {
  Wire.beginTransmission(0x68); // I2C address of the MPU6050
  Wire.write(0x43); // Starting register for Gyro Readings
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6); // Request 6 bytes of Gyro data
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read(); // Store first two bytes into gyroX
  gyroY = Wire.read() << 8 | Wire.read(); // Store middle two bytes into gyroY
  gyroZ = Wire.read() << 8 | Wire.read(); // Store last two bytes into gyroZ
  processGyroData();
}

void processGyroData() {
  rotX = kalmanGyroX.updateEstimate(gyroX / 131.0);
  rotY = kalmanGyroY.updateEstimate(gyroY / 131.0);
  rotZ = kalmanGyroZ.updateEstimate(gyroZ / 131.0);
}

void calculatePitchRollYaw() {
  float accelAngleX = atan(gForceY / sqrt(gForceX * gForceX + gForceZ * gForceZ)) * 180 / PI;
  float accelAngleY = atan(-gForceX / sqrt(gForceY * gForceY + gForceZ * gForceZ)) * 180 / PI;

  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  pitch = 0.98 * (pitch + rotX * elapsedTime) + 0.02 * accelAngleX;
  roll = 0.98 * (roll + rotY * elapsedTime) + 0.02 * accelAngleY;
  yaw += rotZ * elapsedTime;
}

void printData() {
  Serial.print("Gyro (deg)");
  Serial.print(" X=");
  Serial.print(rotX);
  Serial.print(" Y=");
  Serial.print(rotY);
  Serial.print(" Z=");
  Serial.print(rotZ);
  Serial.print(" Accel (g)");
  Serial.print(" X=");
  Serial.print(gForceX);
  Serial.print(" Y=");
  Serial.print(gForceY);
  Serial.print(" Z=");
  Serial.print(gForceZ);
  Serial.print(" Pitch=");
  Serial.print(pitch);
  Serial.print(" Roll=");
  Serial.print(roll);
  Serial.print(" Yaw=");
  Serial.println(yaw);
}