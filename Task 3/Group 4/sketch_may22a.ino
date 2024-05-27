#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Kalman.h> // Make sure to install the Kalman filter library

Adafruit_MPU6050 mpu;
Kalman kalmanX;
Kalman kalmanY;
Kalman kalmanZ; // For yaw

void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

 mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
 // Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    //Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    //Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    //Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    //Serial.println("+-16G");
    break;
  }
 mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  //Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    //Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
   // Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
   // Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    //Serial.println("+- 2000 deg/s");
    break;
  }
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  //Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    //Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    //Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
   // Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    //Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
   // Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
   // Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
   // Serial.println("5 Hz");
    break;
  }
  Serial.println("");
  delay(100);
}

void loop() {
  static uint32_t timer = micros();

  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Calculate delta time */
  double dt = (double)(micros() - timer) / 1000000;
  timer = micros();

  /* Calculate roll and pitch (in degrees) */
  double roll  = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
  double pitch = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;

  /* Convert gyro rates to degrees per second */
  double gyroXrate = g.gyro.x * RAD_TO_DEG;
  double gyroYrate = g.gyro.y * RAD_TO_DEG;
  double gyroZrate = g.gyro.z * RAD_TO_DEG;

  /* Apply Kalman filter */
  double kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt);
  double kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
  double kalAngleZ = kalmanZ.getAngle(0, gyroZrate, dt); // Yaw angle integration

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Yaw: ");
  Serial.print(kalAngleZ); // Only Kalman filtered yaw is available
  Serial.println(" degrees");

  Serial.print("Kalman Roll: ");
  Serial.print(kalAngleX);
  Serial.print(", Kalman Pitch: ");
  Serial.print(kalAngleY);
  Serial.print(", Kalman Yaw: ");
  Serial.print(kalAngleZ);
  Serial.println(" degrees");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(3000);
}
