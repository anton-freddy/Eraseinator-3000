#ifndef PINS_H
#define PINS_H

#include <Arduino.h>


//  I2C Pins
const int I2CA_SDA = 8; // I2C BUS A DATA
const int I2CA_SCL = 9; // I2C BUS A CLOCK
const int I2CB_SDA = 1; // I2C BUS B DATA
const int I2CB_SCL = 2; // I2C BUS B CLOCK



//  LUNA
const int16_t LiDAR_ADD_1 = 0x10;   // LiDAR 1 I2C Addr - Straight Ahead
const int16_t LiDAR_ADD_2 = 0x25;   // LiDAR 2 I2C Addr - Servo Mounted
const int16_t LiDAR_ADD_3 = 0x15;   // LiDAR 3 I2C Addr
uint16_t LiDAR_frame_rate = 0x00;   // LiDAR Frame Rate, 0 = trigger mode
const int LiDAR_1_signal_pin = 48;  // LiDAR 1 Signal pin (PIN 6 on LiDAR Connector)
const int LiDAR_2_signal_pin = 47;  // LiDAR 2 Signal pin (PIN 6 on LiDAR Connector)
const int LED_RING_pin = 21;        // Top LED ring pin, LiDAR 3 Signal pin on PCB


// Motor Pins
// S2
const int R_Stepper_STEP_PIN = 17;      // STEP PIN for Right Stepper
const int R_Stepper_DIR_PIN = 18;       // DIR PIN for Right Stepper
const int R_Stepper_ENABLE_PIN = 16;    // ENABLE PIN for Right Stepper

// S1
const int L_Stepper_STEP_PIN = 15;      // STEP PIN for Left Stepper
const int L_Stepper_DIR_PIN = 7;        // DIR PIN for Left Stepper
const int L_Stepper_ENABLE_PIN = 6;     // ENABLE PIN for Left Stepper

const int MS1_pin = 42;                 // MS1 PIN for both Steppers
const int MS2_pin = 41;                 // MS2 PIN for both Steppers
const int MS3_pin = 40;                 // MS3 PIN for both Steppers

// Slave MCU
const uint8_t slave_ADDR = 0x55;        // Slave I2C Address for Slave MCU (ESP32)

//  Line Sensors
const int L_IR_PIN = 14;            // IR1 on board pinout
const int BATTERY_LEVEL_PIN = 10;   // IR2 on board pinout
const int C_BUMP_PIN = 38;          // IR3 on board pinout
const int R_IR_PIN = 39;            // IR4 on board pinout



//  Limit Switches
const int R_BUMP_PIN = 4; // LIM1 on board pinout
const int L_BUMP_PIN = 5; // LIM2 on board pinout


//  SPI
const int MISO_PIN = 13;
const int MOSI_PIN = 11;
const int SPI_SCK_PIN = 12;

#endif