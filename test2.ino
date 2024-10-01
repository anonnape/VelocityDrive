


// using rc_ESC lib to control motors
#include <ESC.h>
#include <Servo.h>
#include <Arduino.h>
//stability
#include "mpu.h"
 // motors
const uint8_t rl_p = 9; // Rear left pin
const uint8_t rr_p = 8; // Rear right pin
const uint8_t fl_p = 6; // Front left pin
const uint8_t fr_p = 7; // Front right pin


// ESC objets
// min_define_power
#define min_define_value 1030
#define max_define_value 2020


ESC rl(rl_p, min_define_value, max_define_value, 500); // Rear left ESC
ESC rr(rr_p, min_define_value, max_define_value, 500); // Rear right ESC
ESC fl(fl_p, min_define_value, max_define_value, 500); // Front left ESC
ESC fr(fr_p, min_define_value, max_define_value, 500); // Front right ESC
// setting stop pulse as 300 , 500 results in motor jerks 
const uint16_t stop_pulse = 300;
// rl.setStopPulse(stop_pulse);
// rr.setStopPulse(stop_pulse);
// fl.setStopPulse(stop_pulse);
// fr.setStopPulse(stop_pulse);


// servos
const uint8_t ls = 10; // steering servo on the left // both coupled together mechinically and in software
const uint8_t rs = 11; // steering servo on the right

Servo left_servo;
Servo right_servo;

// servo objects

// some more variables for debugging 
const bool arm_on_boot = false;

// input variables or values from transmitter
bool FAILSAFE = true;
static unsigned int sbusByte, byteNmbr;
static byte frame[25]; // 25 bytes per SBUS frame
static unsigned int channel[17]; // 16 channels in SBUS stream + channels 17, 18 & failsafe in channel[0]
static unsigned int i; // a counter
static bool newFrame;
struct ChannelValues {   // standard channels values (throttle , aleiron .... are substituited for names to make code more understadable)
  unsigned int power1; // factor for power
  unsigned int skid; // skid steer factor
  unsigned int steer; // servo steer factor
  unsigned int power2; // another factor for power
  unsigned int aux1; // used for selecting 3 gains for stabilization
  unsigned int aux2; // not used
  unsigned int aux3; // brake
  unsigned int arm; // arming of the motor
};

// Create an instance of the ChannelValues struct
ChannelValues tx;

// when you give up ,you are letting someone else take the win.
// there is 0% chance ppl at robu gonna read this.


  // output variables 
  // output power 0<-->1000 for EACH motor which is mapped for speed_min, speed_max for each motor , rr is rear right , rear left, front right, front left.
  uint16_t rr_output; 
  uint16_t rl_output;
  uint16_t fr_output;
  uint16_t fl_output;

bool power_output; // actually output power motor 
const bool debug = true;
const bool debug_method_ble = true; // serial 0 (over usb) or serial 2 (over bluetooth module)
void setup() {
  // for sbus
    
  if (debug) 
  {
    if(debug_method_ble){ 
    Serial2.setRX(5); // use UART 1 
    Serial2.setTX(4); 
    Serial2.begin(115200); // changed from show 9600
    
    }else{Serial.begin(115200);} // port for debugging
  
    
  }
  Serial1.setRX(17);
  Serial1.setTX(16); // uart 0
  // servos
  left_servo.attach(ls);
  right_servo.attach(rs);


  Serial1.begin(100000, SERIAL_8E2); // SBUS baud rate

  byteNmbr = 255; // invalidate current frame byte
  newFrame = false;
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  dprint("mpu6050"); 
  setupMpu6050(); // Ensure this function initializes the MPU-6050 correctly
  delay(5000); // Wait for a while
  digitalWrite(LED_BUILTIN, 0);

  dprint("Debug is on");
  


}

// functions 

// arm, stop motors
void arm_all_motors(){
      rl.arm();
      rr.arm();
      fl.arm();
      fr.arm();
      delay(10); // delay to prevent false calib mode 
}
// acceleration limit
unsigned int total_power = 0;
unsigned int previous_power = 0;
float acceleration_limit = 0.5; // Adjust as needed // CALIBRATE
float power_adjustment = 0.0;

void updatePower(unsigned int power1, unsigned int power2) {
  unsigned int target_power = calculatePower(power1, power2);
  if (target_power > previous_power) {
    power_adjustment += min(acceleration_limit, (float)(target_power - previous_power));
    total_power = previous_power + round(power_adjustment);
  } else {
    // No limit on deceleration, allow immediate decrease
    total_power = target_power;
  }

  // Ensure total_power is within the valid range (0 to 1000)
  total_power = constrain(total_power, 0, 1000);
  power_adjustment = power_adjustment - round(power_adjustment);

  // Update previous power
  previous_power = total_power;

}



int calculatePower(int temp_power1, int temp_power2) {

  if (temp_power1 >= 490 && temp_power1 <= 510 && temp_power2 >= 490 && temp_power2 <= 510) {
    return 0;
  }

  if (temp_power1 <= temp_power2 + 50) {
    return 0;
  }

  if (temp_power2 > 0 && temp_power1 <= 980) {
    return 0;
  }
  if (temp_power1>900 && temp_power1 <= 500){
    return 0;
  }
  return map(temp_power1 + temp_power2, 0, 2000, 0, 1000);
  
}
// servo steer functions
// Servo configuration // Servo configuration parameters
uint16_t steer_center = 500;  // Neutral position for steering
uint16_t max_steer_right = 1000; // Maximum right steering input
uint16_t max_steer_left = 0; // Maximum left steering input
uint16_t steer_deadzone = 10;  // Deadzone around center
uint16_t min_servo_out = 0;    // Minimum servo value (left limit)
uint16_t max_servo_out = 180;  // Maximum servo value (right limit)

// Current servo values
uint8_t current_ls_val = 0;
uint8_t current_rs_val = 0;

void calculateServoPositions(uint16_t steer_value_input) {
    // Check if input is within the deadzone (centered position)
    if (abs(steer_value_input - steer_center) < steer_deadzone) {
        current_ls_val = (max_servo_out + min_servo_out) / 2;  // Neutral position for both servos
        current_rs_val = current_ls_val;
    } 
    // Steering to the right (value input greater than center)
    else if (steer_value_input > steer_center) {
        uint16_t steer_amount = map(steer_value_input, steer_center, max_steer_right, (max_servo_out + min_servo_out) / 2, max_servo_out);
        current_ls_val = steer_amount;  // Set left and right servos to the same mapped value
        current_rs_val = steer_amount;
    } 
    // Steering to the left (value input less than center)
    else {
        uint16_t steer_amount = map(steer_value_input, steer_center, max_steer_left, (max_servo_out + min_servo_out) / 2, min_servo_out);
        current_ls_val = steer_amount;  // Set left and right servos to the same mapped value
        current_rs_val = steer_amount;
    }
}

// Function to write servo values
void writeServoValues(Servo &left_servo, Servo &right_servo) {
    left_servo.write(current_ls_val);
    right_servo.write(current_rs_val);
}

// functions and variables  for sbus
// serial 1 is uart 0 ,using pins 17, 16

bool getFrame() {
  while (Serial1.available()) {
    sbusByte = Serial1.read();
    // so we use a flag to detect the end of a packet (0) before enabling the capture of next frame
    if ((sbusByte == 0x0F) && newFrame) { // if this byte is SBUS start byte start counting bytes
      newFrame = false;
      byteNmbr = 0;
    } else if (sbusByte == 0) newFrame = true; // end of frame, enable start of next frame (to distinguish from 0x0F channel values)
    if (byteNmbr <= 24) {
      frame[byteNmbr] = sbusByte;
      byteNmbr++;
      if ((byteNmbr == 25) && (sbusByte == 0) && (frame[0] == 0x0F)) return true; // byteNmbr is now invalid, ready for next frame
    }
  }
  return false;
}
void decodeChannels() {
  int bitPtr; // bit pointer in SBUS byte being decoded
  int bytePtr; // byte pointer in SBUS frame
  int chan; // channel number being decoded
  int chanBit; // current channel bit being proccessed

  channel[0] = frame[23];
  bytePtr = 1;
  bitPtr = 0;
  for (chan = 1; chan <= 16; chan++) {
    channel[chan] = 0;
    for (chanBit = 0; chanBit < 11; chanBit++) {
      channel[chan] |= ((frame[bytePtr] >> bitPtr) & 1) << chanBit;
      if (++bitPtr > 7) {
        bitPtr = 0;
        bytePtr++;
      }
    }
  }
}
// function to print debug messages
template<typename... Args>
void dprint(Args... args) {
  if (debug) {
    if (!debug_method_ble){
    (Serial.print(args), ...);  // Expands and prints each argument
    Serial.println();  // Adds a newline at the end
  }else{
        (Serial2.print(args), ...);  // Expands and prints each argument
    Serial2.println();  // Adds a newline at the end
  }

  }
}
// status variables
bool prev_arm_status = false; 
bool arm_status = false;
bool prev_failsafe_status = false; 
void loop() {


  if (getFrame()) {
    decodeChannels();

    unsigned int mappedValue[8];
    for (i = 1; i <= 8; i++) {
      mappedValue[i - 1] = map((channel[i]), 1000, 2000, 0, 1000);
    }
    // Store the mapped values in the tx structure
    tx.power1 = mappedValue[2];
    tx.skid = mappedValue[3];
    tx.steer = mappedValue[0];
    tx.power2 = mappedValue[1];
    tx.aux1 = mappedValue[4];
    tx.aux2 = mappedValue[5];
    tx.aux3 = mappedValue[6];
    tx.arm = mappedValue[7];

    // Print the stored mapped values
    if (!debug) {
      Serial.print("Power1: ");
      Serial.print(tx.power1);
      Serial.print(" ");
      Serial.print("Skid: ");
      Serial.print(tx.skid);
      Serial.print(" ");
      Serial.print("Steer: ");
      Serial.print(tx.steer);
      Serial.print(" ");
      Serial.print("power2: ");
      Serial.print(tx.power2);
      Serial.print(" ");
      Serial.print("Aux1: ");
      Serial.print(tx.aux1);
      Serial.print(" ");
      Serial.print("Aux2: ");
      Serial.print(tx.aux2);
      Serial.print(" ");
      Serial.print("Aux3: ");
      Serial.print(tx.aux3);
      Serial.print(" ");
      Serial.print("Arm: ");
      Serial.print(tx.arm);
      Serial.println();
    }
    // Failsafe check
    if (channel[0] & 8)
      FAILSAFE = true;
    else
      FAILSAFE = false; // failsafe
  }
  // check for arm and failsafe status and arm and disarm the escs

  if (tx.arm > 990 && tx.arm < 1001) {
      arm_status = true;
  } else {
      arm_status = false;
  }
  if (FAILSAFE!=prev_failsafe_status)
  {

  
// to do : another safety thing, arms only when arm channel is true for more than 1 second // not that necessary 

    if(FAILSAFE)
    {
      dprint("FAILSAFE detected , disarming");
      rl.stop();
      rr.stop();
      fl.stop();
      fr.stop();
      total_power = 0;
      power_output = false;
      prev_failsafe_status = FAILSAFE;
    }
  else {
    dprint("FAILSAFE was omitted , must rearm");
    prev_failsafe_status = FAILSAFE;

  }
  }
  if  (prev_arm_status != arm_status){
      if (arm_status){
        if (tx.power1 == 0 && tx.power2 == 0)
        {
        
        digitalWrite(LED_BUILTIN, 1); 
        dprint("ARMING ALL MOTORS"); 
        arm_all_motors();
        power_output = true;
        
        }else{
          dprint("will not arm unless power factors are 0, Please REARM");
          power_output = false;

        }
        prev_arm_status = true;
        
      }
      else{
          power_output = false;
          digitalWrite(LED_BUILTIN, 0); 
          dprint("DISARMING ALL MOTORS"); 
          rl.stop();
          rr.stop();
          fl.stop();
          fr.stop();
          prev_arm_status = false;
      }
      }



  // todo 
  // calculate skid levee, servos anagle etc etc
  updatePower(tx.power1, tx.power2);
  // MAIN control loop
  // variables
  
  // input variables 
  total_power = total_power;   // 0 to 1000  // goes from 0 to 1000 , total power , or speed determined
  // only decreases speed on either side , 

  tx.skid = tx.skid; // 0 to 1000, steer via different side to side wheel speeds , center at 500
  tx.steer = tx.steer; // 0 to 1000, steer via servo , center at 500, greater than 500 = right, less = left , same for tx.skid
  

  // output for servos objects -> left_servo, right_servo
  uint16_t current_ls_val = 0;
  uint16_t current_rs_val = 0;

  // pseudo code
  // determine steer values , only using tx.steer ---> 
  //delay(10); 
        // steer with 
      uint16_t inputSteering = tx.steer; 
      uint16_t outputSteering = inputSteering; //mrsc(inputSteering);
      //dprint(tx.steer, "--->", outputSteering);
      calculateServoPositions(outputSteering);
      writeServoValues(left_servo, right_servo);

  updateMotorOutputs(total_power, tx.skid); 
  // best for calibration
  #define SPEED_MIN 1030 // Set the Minimum Speed in microseconds
  #define SPEED_MAX 1500 // Set the Minimum Speed in microseconds // change to 1500
  if(power_output){
// uncommend for calibration
  // if (tx.power1 > 300) {
  //     rr_output = 1000;
  // } else {
  //     rr_output = 0;
  // }
  //   rl_output = rr_output;
  // fl_output = rl_output; 
  // fr_output = fl_output;
    dprint("--> ", rl_output," --> ",rr_output, "-->", map(rr_output, 0, 1000, SPEED_MIN, SPEED_MAX));


    rr.speed(map(rr_output, 0, 1000, SPEED_MIN, SPEED_MAX));
    rl.speed(map(rl_output, 0, 1000, SPEED_MIN, SPEED_MAX));
    fr.speed(map(fr_output, 0, 1000, SPEED_MIN, SPEED_MAX));
    fl.speed(map(fl_output, 0, 1000, SPEED_MIN, SPEED_MAX));
  }else{
    rr.stop();
    rl.stop();
    fr.stop();
    fl.stop();
  }




}



// motor control function
  void updateMotorOutputs(uint16_t total_power, uint16_t tx_skid) {
    // Adjust skid steer value relative to center (500)
    int16_t skid_adjust = tx_skid - 500; 

    // Calculate power for the right side (fr, rr) by increasing power based on skid steer
    fr_output = constrain(total_power + skid_adjust, 0, 1000);
    rr_output = constrain(total_power + skid_adjust, 0, 1000);

    // Calculate power for the left side (fl, rl) by decreasing power based on skid steer
    fl_output = constrain(total_power - skid_adjust, 0, 1000);
    rl_output = constrain(total_power - skid_adjust, 0, 1000);
  }
