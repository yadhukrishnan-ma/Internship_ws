#include <Wire.h>
#include <FastIMU.h>

// Trick to expose private variables (q0, q1, q2, q3) in the library
#define private public
#include <MadgwickAHRS.h>
#undef private

MPU6500 imu1;
MPU6500 imu2;

Madgwick filter1;
Madgwick filter2;

AccelData accel1, accel2;
GyroData  gyro1,  gyro2;

// ── Gyro bias offsets (computed at startup) ──────────────────────
float gx_off1 = 0, gy_off1 = 0, gz_off1 = 0;
float gx_off2 = 0, gy_off2 = 0, gz_off2 = 0;

// ── Timing for accurate sample rate ─────────────────────────────
unsigned long lastMicros = 0;
float actualFreq = 200.0;

// ── Packet sequencing for drop detection ────────────────────────
uint32_t seqNum = 0;

// ── Calibration parameters ──────────────────────────────────────
const int CAL_SAMPLES = 1000;       // 4 seconds at 250Hz for better gyro bias
const float ACCEL_MAG_MIN = 0.85;   // g — reject if sensor moved during cal
const float ACCEL_MAG_MAX = 1.15;   // g — reject if sensor moved during cal

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C for faster reads

  calData cal = {};

  int err1 = imu1.init(cal, 0x68);
  int err2 = imu2.init(cal, 0x69);

  if (err1 != 0) Serial.println("IMU1 FAILED");
  else           Serial.println("IMU1 OK");

  if (err2 != 0) Serial.println("IMU2 FAILED");
  else           Serial.println("IMU2 OK");

  delay(500);

  // ── Gyro Calibration: measure zero-rate bias ──────────────────
  // Keep the sensors STILL during this phase!
  Serial.println("CALIBRATING... Keep sensors still!");
  float sx1 = 0, sy1 = 0, sz1 = 0;
  float sx2 = 0, sy2 = 0, sz2 = 0;
  int validSamples = 0;
  int rejectedSamples = 0;

  for (int i = 0; i < CAL_SAMPLES; i++) {
    imu1.update();
    imu2.update();
    imu1.getAccel(&accel1);
    imu1.getGyro(&gyro1);
    imu2.getAccel(&accel2);
    imu2.getGyro(&gyro2);

    // Check accelerometer magnitude — should be ~1g if stationary
    float mag1 = sqrt(accel1.accelX*accel1.accelX +
                      accel1.accelY*accel1.accelY +
                      accel1.accelZ*accel1.accelZ);
    float mag2 = sqrt(accel2.accelX*accel2.accelX +
                      accel2.accelY*accel2.accelY +
                      accel2.accelZ*accel2.accelZ);

    if (mag1 >= ACCEL_MAG_MIN && mag1 <= ACCEL_MAG_MAX &&
        mag2 >= ACCEL_MAG_MIN && mag2 <= ACCEL_MAG_MAX) {
      sx1 += gyro1.gyroX;  sy1 += gyro1.gyroY;  sz1 += gyro1.gyroZ;
      sx2 += gyro2.gyroX;  sy2 += gyro2.gyroY;  sz2 += gyro2.gyroZ;
      validSamples++;
    } else {
      rejectedSamples++;
    }
    delay(4);
  }

  if (validSamples < 200) {
    Serial.println("WARNING: Too few valid calibration samples! Keep sensors STILL!");
    Serial.printf("Valid: %d, Rejected: %d\n", validSamples, rejectedSamples);
    // Use whatever we got
    if (validSamples == 0) validSamples = 1;  // prevent div by zero
  }

  gx_off1 = sx1 / validSamples;
  gy_off1 = sy1 / validSamples;
  gz_off1 = sz1 / validSamples;
  gx_off2 = sx2 / validSamples;
  gy_off2 = sy2 / validSamples;
  gz_off2 = sz2 / validSamples;

  Serial.printf("CAL DONE (%d valid, %d rejected)\n", validSamples, rejectedSamples);
  Serial.printf("  gyro1_bias=(%.3f,%.3f,%.3f)\n", gx_off1, gy_off1, gz_off1);
  Serial.printf("  gyro2_bias=(%.3f,%.3f,%.3f)\n", gx_off2, gy_off2, gz_off2);

  // ── Start Madgwick filters ────────────────────────────────────
  filter1.begin(200);
  filter2.begin(200);

  // Let the Madgwick filter converge for 2 seconds before sending data
  Serial.println("CONVERGING... Madgwick filter warming up (2s)");
  unsigned long convergeStart = millis();
  while (millis() - convergeStart < 2000) {
    imu1.update();
    imu2.update();
    imu1.getAccel(&accel1);
    imu1.getGyro(&gyro1);
    imu2.getAccel(&accel2);
    imu2.getGyro(&gyro2);

    float gx1 = gyro1.gyroX - gx_off1;
    float gy1 = gyro1.gyroY - gy_off1;
    float gz1 = gyro1.gyroZ - gz_off1;
    float gx2 = gyro2.gyroX - gx_off2;
    float gy2 = gyro2.gyroY - gy_off2;
    float gz2 = gyro2.gyroZ - gz_off2;

    filter1.updateIMU(gx1, gy1, gz1,
      accel1.accelX, accel1.accelY, accel1.accelZ);
    filter2.updateIMU(gx2, gy2, gz2,
      accel2.accelX, accel2.accelY, accel2.accelZ);
    delay(4);
  }

  lastMicros = micros();
  seqNum = 0;

  // ── Signal readiness to ROS2 bridge ───────────────────────────
  Serial.println("READY");
  delay(100);
}

void loop() {
  // ── Measure actual loop time for accurate Madgwick ─────────────
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0f;
  lastMicros = now;

  // Update Madgwick sample rate dynamically (clamp to reasonable range)
  if (dt > 0.001f && dt < 0.05f) {
    actualFreq = 1.0f / dt;
  }

  imu1.update();
  imu2.update();

  imu1.getAccel(&accel1);
  imu1.getGyro(&gyro1);
  imu2.getAccel(&accel2);
  imu2.getGyro(&gyro2);

  // ── Subtract gyro bias ────────────────────────────────────────
  float gx1 = gyro1.gyroX - gx_off1;
  float gy1 = gyro1.gyroY - gy_off1;
  float gz1 = gyro1.gyroZ - gz_off1;
  float gx2 = gyro2.gyroX - gx_off2;
  float gy2 = gyro2.gyroY - gy_off2;
  float gz2 = gyro2.gyroZ - gz_off2;

  // ── Dynamic Sample Rate for accurate Madgwick integration ─────
  if (dt > 0.001f && dt < 0.1f) {
      filter1.invSampleFreq = dt;
      filter2.invSampleFreq = dt;
  }

  // ── Update filters with bias-corrected gyro data ──────────────
  filter1.updateIMU(gx1, gy1, gz1,
    accel1.accelX, accel1.accelY, accel1.accelZ);
  filter2.updateIMU(gx2, gy2, gz2,
    accel2.accelX, accel2.accelY, accel2.accelZ);

  // ── Send: seq, timestamp_us, q1w,q1x,q1y,q1z, q2w,q2x,q2y,q2z ─
  // 10 fields: enables drop detection + latency measurement on ROS side
  Serial.printf("%u,%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
    seqNum, now,
    filter1.q0, filter1.q1, filter1.q2, filter1.q3,
    filter2.q0, filter2.q1, filter2.q2, filter2.q3);

  seqNum++;

  delay(4);  // ~200Hz target (4ms delay + processing time)
}
