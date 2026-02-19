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
#include <Adafruit_PWMServoDriver.h> // "instruction manual for your arduino to talk to the PCA9685 chip"

// configuration
#define SERVO_CHANNEL 0 // the PCA9685 output pin (0-15)
#define SERVOMIN      125 // tick count for 0 degrees
#define SERVOMAX      575 // tick count for 180 degrees
#define SERVO_FREQ    50 // Analog servos run at 50Hz
#define OSC_FREQ      27000000 // the internal clock speed of the PCA9685 chip so that it can calculate time accurately

// create the driver object (default I2C address is 0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// function Prototypes
void setServoAngle(int degrees);

void setup() {
    Serial.begin(9600);
    Serial.println("PCA9685 Servo Test Starting...");

    pwm.begin();
    pwm.setOscillatorFrequency(OSC_FREQ);
    pwm.setPWMFreq(SERVO_FREQ);

    // time for the hardware to stabilize
    delay(10);
}

// implement behavior here
void loop() {
    Serial.println("--- Cycle Start ---"); // for the serial monitor on pc
    
    setServoAngle(0); // in degrees
    delay(2000); // give the motor time to move

    setServoAngle(90);
    delay(2000);

    setServoAngle(180);
    delay(2000);
}

// convert pwm signal to degrees
void setServoAngle(int degrees) {
    // constrain input to prevent physical damage from out-of-bounds values
    degrees = constrain(degrees, 0, 180);
    
    // map degrees to the calibrated tick range
    int pulse = map(degrees, 0, 180, SERVOMIN, SERVOMAX);
    
    pwm.setPWM(SERVO_CHANNEL, 0, pulse);

    // feedback for debugging
    Serial.print("Angle: ");
    Serial.print(degrees);
    Serial.print("° | Ticks: ");
    Serial.println(pulse);
}