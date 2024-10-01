#ifndef mpu_h
#define mpu_h

#include "Arduino.h"
#include <Wire.h>

// Declare MPU variables
int gyro_x, gyro_y, gyro_z;
long acc_x_raw, acc_y_raw, acc_z_raw;
int temperature;
long gyro_z_cal;
float yaw_rate = 0;
float yaw_angle = 0;
long acc_z = 0;
long acc_z_diff = 0;
long acc_z_old;

const int calibrationPasses = 125; // Gyro calibration takes 1s @ 125Hz refresh rate
int movement_threshold = 80; // Threshold for vehicle movement based on z acceleration

// Function to process MPU-6050 data
void processMpu6050Data() {
  gyro_z -= gyro_z_cal;

  // Gyro angle calculations
  yaw_rate = (gyro_z * 0.0000646); // Yaw rate in degrees per second
  yaw_angle += yaw_rate;

  // Measure acceleration (for movement detection)
  acc_z = acc_z_raw;
  acc_z_diff = abs(acc_z_old - acc_z);
  acc_z_old = acc_z;

  // Reset yaw_angle if vehicle is not moving for 500ms
  static unsigned long lastMovement;
  if (acc_z_diff > movement_threshold) lastMovement = millis();
  if (millis() - lastMovement >= 500) yaw_angle = 0;
}

// Sub function to read raw data from MPU-6050
void readMpu6050Raw() {
  Wire1.beginTransmission(0x68);          // Start communication with MPU-6050
  Wire1.write(0x3B);                      // Send the starting register address
  Wire1.endTransmission();                // End transmission
  Wire1.requestFrom(0x68, 14);            // Request 14 bytes from MPU-6050
  while (Wire1.available() < 14);         // Wait for all bytes
  acc_x_raw = Wire1.read() << 8 | Wire1.read();
  acc_y_raw = Wire1.read() << 8 | Wire1.read();
  acc_z_raw = Wire1.read() << 8 | Wire1.read();
  temperature = Wire1.read() << 8 | Wire1.read();
  gyro_x = Wire1.read() << 8 | Wire1.read();
  gyro_y = Wire1.read() << 8 | Wire1.read();
  gyro_z = Wire1.read() << 8 | Wire1.read();
}

// Main function to read MPU-6050 data
void readMpu6050Data() {
  static unsigned long lastReading;
  if (micros() - lastReading >= 8000) {   // Read data every 8000us (equals 125Hz)
    lastReading = micros();

    readMpu6050Raw();                     // Read raw data
    processMpu6050Data();                 // Process the data
  }
}

// MPU-6050 setup function
void setupMpu6050() {
   // Initialize I2C1 with custom pins
  Wire1.setSCL(27); // Set SCL to GPIO 27
  Wire1.setSDA(26); // Set SDA to GPIO 26
  Wire1.begin();    // Begin I2C communication          

  // Wake up MPU-6050
  Wire1.beginTransmission(0x68);
  Wire1.write(0x6B);                      // Power management register
  Wire1.write(0x00);                      // Wake up MPU-6050
  Wire1.endTransmission();

  // Configure accelerometer (+/-8g)
  Wire1.beginTransmission(0x68);
  Wire1.write(0x1C);                      // Accelerometer config register
  Wire1.write(0x10);                      // Set accelerometer range to +/-8g
  Wire1.endTransmission();

  // Configure gyro (250°/sec full scale)
  Wire1.beginTransmission(0x68);
  Wire1.write(0x1B);                      // Gyro config register
  Wire1.write(0x00);                      // Set gyro range to +/-250°/s
  Wire1.endTransmission();

  // Gyro calibration
  int cal_int = 0;
  while (cal_int < calibrationPasses) {
    static unsigned long lastGyroCal;
    if (micros() - lastGyroCal >= 8000) {
      readMpu6050Raw();                   // Read raw data
      gyro_z_cal += gyro_z;               // Add the gyro z-axis offset
      lastGyroCal = micros();
      cal_int++;
    }
  }
  gyro_z_cal /= calibrationPasses;        // Average the gyro calibration value
}

#endif
