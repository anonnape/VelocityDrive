const float headingMultiplier = 2.0; // Multiplier to adjust the heading to keep front wheels parallel
byte mrscGain = 80; // Default MRSC gain if no remote gain input is used
boolean mpuInversed = false; // Variable to store the state of the direction inversion switch

// Function prototypes
void readMpu6050Data(); // You need to implement this function in "mpu.h"
extern float yaw_angle; // External variable to store yaw angle from MPU-6050
extern float yaw_rate;  // External variable to store yaw rate from MPU-6050

int mrsc(int steeringInput) {
  float steeringAngle;
  float gyroFeedback;
  
  // Read data from MPU-6050 sensor
  readMpu6050Data(); // Function to read sensor data (defined in "mpu.h")

  // Calculate steering compensation based on gyro feedback
  int turnRateSetPoint = map(steeringInput, 0, 1000, -50, 50); // Map input steering angle (0-1000) to -50 to 50 range
  int steering = abs(turnRateSetPoint); // Absolute value of steering input
  int gain = map(steering, 0, 50, mrscGain, (mrscGain / 5)); // Calculate gain based on steering input


  if (steering < 5 && mrscGain > 85) { // If driving straight with high gain
    gyroFeedback = yaw_angle * headingMultiplier; // Use yaw angle for feedback
  } else { // If cornering or low gain
    gyroFeedback = yaw_rate * 10.0; // Reduced yaw rate scaling factor to prevent large gyroFeedback
    yaw_angle = 0; // Reset yaw angle
  }

  if (mpuInversed) {
    steeringAngle = turnRateSetPoint + (gyroFeedback * gain / 100.0); // Adjust steering based on feedback
  } else {
    steeringAngle = turnRateSetPoint - (gyroFeedback * gain / 100.0); // Adjust steering based on feedback
  }

  // Debug print for steering angle before constrain

  // Constrain steeringAngle to -50 to 50
  steeringAngle = constrain(steeringAngle, -50.0, 50.0);


  // Map steering angle to output range (0-1000)
  return map(steeringAngle, -50.0, 50.0, 0, 1000);
}
