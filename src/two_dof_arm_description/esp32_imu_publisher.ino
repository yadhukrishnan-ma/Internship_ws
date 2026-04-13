#include <Wire.h>
#include <FastIMU.h>

TwoWire I2C_0 = TwoWire(0);
TwoWire I2C_1 = TwoWire(1);

MPU6500 imu1(I2C_0);
MPU6500 imu2(I2C_1);

calData calib1 = { 0 };
calData calib2 = { 0 };

AccelData accel1, accel2;
GyroData gyro1, gyro2;

const unsigned long PERIOD_US = 10000;
unsigned long lastTime = 0;

float q1_w = 1, q1_x = 0, q1_y = 0, q1_z = 0;
float q2_w = 1, q2_x = 0, q2_y = 0, q2_z = 0;
const float beta = 0.1f;
const float dt = 0.01f;

void madgwick(float ax, float ay, float az,
              float gx, float gy, float gz,
              float &qw, float &qx, float &qy, float &qz) {
  float norm = sqrt(ax*ax + ay*ay + az*az);
  if (norm == 0.0f) return;
  ax /= norm; ay /= norm; az /= norm;

  gx *= DEG_TO_RAD; gy *= DEG_TO_RAD; gz *= DEG_TO_RAD;

  float f1 = 2.0f*(qx*qz - qw*qy) - ax;
  float f2 = 2.0f*(qw*qx + qy*qz) - ay;
  float f3 = 1.0f - 2.0f*(qx*qx + qy*qy) - az;

  float grad_w = -2.0f*qy*f1 + 2.0f*qx*f2;
  float grad_x =  2.0f*qz*f1 + 2.0f*qw*f2 - 4.0f*qx*f3;
  float grad_y = -2.0f*qw*f1 + 2.0f*qz*f2 - 4.0f*qy*f3;
  float grad_z =  2.0f*qx*f1 + 2.0f*qy*f2;

  norm = sqrt(grad_w*grad_w + grad_x*grad_x + grad_y*grad_y + grad_z*grad_z);
  if (norm == 0.0f) return;
  grad_w /= norm; grad_x /= norm; grad_y /= norm; grad_z /= norm;

  float qdot_w = 0.5f*(-qx*gx - qy*gy - qz*gz) - beta*grad_w;
  float qdot_x = 0.5f*( qw*gx + qy*gz - qz*gy) - beta*grad_x;
  float qdot_y = 0.5f*( qw*gy - qx*gz + qz*gx) - beta*grad_y;
  float qdot_z = 0.5f*( qw*gz + qx*gy - qy*gx) - beta*grad_z;

  qw += qdot_w * dt;
  qx += qdot_x * dt;
  qy += qdot_y * dt;
  qz += qdot_z * dt;

  norm = sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  qw /= norm; qx /= norm; qy /= norm; qz /= norm;
}

void setup() {
  Serial.begin(921600);
  delay(2000);

  I2C_0.begin(22, 21, 400000);
  I2C_1.begin(19, 18, 400000);
  delay(500);

  int err1 = imu1.init(calib1, 0x68);
  if (err1 != 0) { Serial.printf("ERR:IMU1 %d\n", err1); while(1); }
  Serial.println("IMU1 OK");

  int err2 = imu2.init(calib2, 0x68);
  if (err2 != 0) { Serial.printf("ERR:IMU2 %d\n", err2); while(1); }
  Serial.println("IMU2 OK");

  Serial.println("INFO:CALIBRATING - keep still");
  delay(3000);
  imu1.calibrateAccelGyro(&calib1);
  imu2.calibrateAccelGyro(&calib2);
  imu1.init(calib1, 0x68);
  imu2.init(calib2, 0x68);

  Serial.println("INFO:READY");
}

void loop() {
  unsigned long now = micros();
  if (now - lastTime < PERIOD_US) return;
  lastTime = now;

  imu1.update();
  imu2.update();

  imu1.getAccel(&accel1);
  imu1.getGyro(&gyro1);
  imu2.getAccel(&accel2);
  imu2.getGyro(&gyro2);

  madgwick(accel1.accelX, accel1.accelY, accel1.accelZ,
           gyro1.gyroX, gyro1.gyroY, gyro1.gyroZ,
           q1_w, q1_x, q1_y, q1_z);

  madgwick(accel2.accelX, accel2.accelY, accel2.accelZ,
           gyro2.gyroX, gyro2.gyroY, gyro2.gyroZ,
           q2_w, q2_x, q2_y, q2_z);

  Serial.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
    q1_w, q1_x, q1_y, q1_z,
    q2_w, q2_x, q2_y, q2_z);
}
