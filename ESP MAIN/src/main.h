#ifndef MAIN_H
#define MAIN_H

// Inlcudes
#include <Arduino.h>
#include <Esp.h>
#include <pins.h>
#include <string>
#include <math.h>
#include <iostream>
#include <web_connection.h>

// Libaries
#include <Wire.h>
#include <DDR_Stepper.h>
#include <ESP32Servo.h>
#include <FS.h>
#include <LittleFS.h>
#include <FFat.h>
#include <LiDAR.h>
#include <Battery.h>
#include <ERROR.h>


const float WHEEL_CIRCUMFERENCE = 153.15; // 157.1; // Dia = 48.75
const float WHEEL_DISTANCE = 311.6;       // 281.6; // OUTER: 311.6mm INNER: 281.6mm
const int MICROSTEP = 2;
const int STEPPER_STEP_COUNT = 200;
const int GEAR_RATIO = 2;
// const int STEPS_PER_REV;

Battery battery(8000, 13000, BATTERY_LEVEL_PIN);//3041


const int obstciale_radius = 200;

//  Time Variables
unsigned long previousMillis_Servo = 0;

//  Servo Var
  int servo_increment = 5;
  int servo_pos = 90;
  int servo_delayBetweenSteps = 20;

float map_f(float x, float in_min, float in_max, float out_min, float out_max)
{
  const float run = in_max - in_min;
  if (run == 0)
  {
    log_e("map_f(): Invalid input range, min == max");
    return -1; // AVR returns -1, SAM returns 0
  }
  const float rise = out_max - out_min;
  const float delta = x - in_min;
  return (delta * rise) / run + out_min;
}

void float_to_ints(float floatValue, int &integerPart, int &decimalPart)
{
  // Split the float into integer and decimal parts
  integerPart = int(floatValue);
  decimalPart = int((floatValue - integerPart) * 1000); // Multiply by 1000 to get 3 decimal places
}

float ints_to_float(int integerPart, int decimalPart)
{
  // Reconstruct the float from integer and decimal parts
  return float(integerPart) + float(decimalPart) / 1000.0; // Divide by 1000 to restore the decimal places
}


enum RegisterAddress
{
    REG_SERVO_POS,
    REG_SERVO_STATE,
    REG_CHARGE_LEVEL,
    REG_XPOS,
    REG_YPOS,
    REG_APOS,
    REG_DCL,
    REG_DCR
};

enum ServoState
{
    CENTER = 0,
    RUN = 1,
    PAUSE = 2
};

bool setup_I2C();
void setup_IR();
bool get_IR1_status();
bool get_IR2_status();
void IR1_ISR();
void IR2_ISR();
void setupBatterySense();
int getBatteryLevel();
void bump_ISR_C();
void bump_ISR_L();
void bump_ISR_R();
void setupBumpers();
void setBumpBackOFF();
void tickServo();
void setDCmotor(int motor_select, bool active);

bool bump_triggred = false;
Servo servo;
bool SERVO_CLOCKWISE = true;
int SERVO_pos = 0;
long servo_previousMillis = 0;
#define SERVO_INTERVAL 200
bool is_I2C_setup = false;
void setup_servo();
void move_servo();

int requestIntFromSlave(uint8_t regAddress);
float requestFloatFromSlave(uint8_t regAddress);

void writeIntToSlave(uint8_t regAddress, int data);
void writeFloatToSlave(uint8_t regAddress, float data);


void I2C_Scan(TwoWire &I2C) {
  TwoWire *ptr;
  ptr = &I2C;
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    ptr->beginTransmission(address);
    error = ptr->endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}

#endif