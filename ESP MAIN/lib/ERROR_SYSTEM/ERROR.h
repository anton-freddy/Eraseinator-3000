#ifndef ERROR_H
#define ERROR_H

#include <Arduino.h>
#include <iostream>
#include <math.h>
#include <WebSerial.h>
#include <LittleFS.h>

#define LiDAR_1 0
#define LiDAR_2 1
#define LiDAR_3 2
#define LEFT_ENCODER 3
#define RIGHT_ENCODER 4
#define ENCODERS 34
#define SERVO 5
#define LEFT_STEPPER 6
#define RIGHT_STEPPER 7
#define MOVEMENT 8
#define OLED_DISPLAY 9
#define IR1 10
#define IR2 11
#define IR3 12
#define IR4 13
#define DC_MOTORS 14
#define L_DC_MOTOR 15
#define R_DC_MOTOR 16
#define BATTERY 17

void send_ERROR(int module_code, int error_code);

String getLiDAR_ERRORS(int module_code, int error_code);
String getStepper_ERRORS(int module_code, int error_code);
String getEncoder_ERRORS(int module_code, int error_code);
String getMovement_ERRORS(int module_code, int error_code);
String getIR_ERRORS(int module_code, int error_code);

void serial_send_ERROR(String MSG);


#endif  // ERROR_H