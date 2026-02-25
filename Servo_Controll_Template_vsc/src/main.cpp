/* Servo Motor Controll
used hardware
  - Arduino Board: Arduino Nano ESP32
  - Motor Driver: PCA9685
  - Servo: RDS3235 (MG996R worked as well)
libraries needed
  - Adafruit BusIO
  - Adafruit PWM Servo Driver Library
  - ESP32Servo
  - PCA9685
cnnection from arduino to motor driver
  - GND to GND
  - A5 to SCL
  - A4 to SDA
  - VIN to VCC
useful link
https://learn.adafruit.com/16-channel-pwm-servo-driver/hooking-it-up
*/

// setup
#include <Arduino.h> // to use arduino framework -> a lot of usable references in the internet
#include <Wire.h> // allows the board to communicate with devices over I2C
#include <Adafruit_PWMServoDriver.h> //// "instruction manual for your arduino to talk to the PCA9685 chip"

// configuration
#define SERVO_CHANNEL_0 0 // pin to connect first motor to (the PCA9685 output pin (0-15))
#define SERVO_CHANNEL_1 4
#define SERVO_CHANNEL_2 8
#define SERVO_CHANNEL_3 12
#define SERVO_CHANNEL_4 15
#define SERVO_MIN_0 125 // tick count for 0 degrees for first motor (individual for every servo)
#define SERVO_MAX_0 575 // tick count for 180 degrees
#define SERVO_MIN_1 125 
#define SERVO_MAX_1 575
#define SERVO_MIN_2 125 
#define SERVO_MAX_2 575
#define SERVO_MIN_3 125 
#define SERVO_MAX_3 575
#define SERVO_MIN_4 125 
#define SERVO_MAX_4 575
#define SERVO_FREQ 50 // Analog servos run at 50Hz
#define OSC_FREQ 27000000 // the internal clock speed of the PCA9685 chip so that it can calculate time accurately

// servo struct -> we use a struct to collect all servo info in one id (just makes the usage of the functions easier)
typedef struct {
  int channel; // pin to connect first motor to (the PCA9685 output pin (0-15))
  int max; // tick count for 0 degrees for first motor (individual for every servo)
  int min; // tick count for 180 degrees
} servo;

// create servos
servo shoulder_pitch = {SERVO_CHANNEL_0, SERVO_MIN_0, SERVO_MAX_0}; // to be renamed (joints)
servo shoulder_yaw = {SERVO_CHANNEL_1, SERVO_MIN_1, SERVO_MAX_1};
servo elbow = {SERVO_CHANNEL_2, SERVO_MIN_2, SERVO_MAX_2};
servo wrist_turn = {SERVO_CHANNEL_3, SERVO_MIN_3, SERVO_MAX_3};
// add further servos in the same manner

// create the driver object (default I2C address is 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// function Prototypes
void setServoAngle(servo *id, int degrees);

void setup() {
    // set baud rate to 9600 for i2c communication
    Serial.begin(9600);
    Serial.println("PCA9685 Servo Test Starting..."); // just for debugging

    // initiate communication between esp32 and motor driver
    pwm.begin();
    pwm.setOscillatorFrequency(OSC_FREQ);
    pwm.setPWMFreq(SERVO_FREQ);

    // time for the hardware to stabilize
    delay(10);
}

// implement behavior here
void loop() {
    Serial.println("--- Cycle Start ---"); // for the serial monitor on pc
    
    setServoAngle(&shoulder_pitch, 30); // in degrees
    delay(4000);

    setServoAngle(&shoulder_pitch, 60);
    delay(4000);
}

// convert pwm signal to degrees
void setServoAngle(servo *id, int degrees) {
    // constrain input to prevent physical damage from out-of-bounds values
    degrees = constrain(degrees, 0, 180);
    
    // map degrees to the calibrated tick range
    int pulse = map(degrees, 0, 180, id->min, id->max);
    
    // send pwm signal
    pwm.setPWM(id->channel, 0, pulse);

    // feedback for debugging
    Serial.print("Angle: ");
    Serial.print(degrees);
    Serial.print("° | Ticks: ");
    Serial.println(pulse);
}