#include <Arduino.h>
//#include <ESP32Servo.h>
#include <Wire.h>
#include <round_LCD.h>
#include <OTA-update.h>
//#include <Servo.h>
#include <ESP32_Servo.h>
#include <LittleFS.h>
#include <esp_now.h>

const int SDA_pin = 21;
const int SCL_pin = 22;

const int LDC_pin = 13;
const int RDC_pin = 14;
const int SERVO_pin = 25;

//  LCD
const int RSTLCD_pin = 33;
const int BL_LCD_pin = 32;
const int LCD_MOSI = 23;
const int LCD_CLK_pin = 18;
const int LCD_CS_pin = 17;
const int LCD_DC_pin = 16;

const int INPUT1_pin = 34;
const int INPUT2_pin = 35;

const uint8_t slave_ADDR = 0x55;

long elapsedTime = 0;
long previousTime = 0;
long currentTime = 0;

uint8_t requestREG = 0x00;

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
    CENTER,
    RUN,
    PAUSE
};

struct timing
{
    unsigned long previous = 0;
    unsigned long current = 0;
};

float a_pos = 0;
float x_pos = 0;
float y_pos = 0;

timing servoMillis;
int servoState;
Servo servo;
bool servo_inc = true;
int servo_LIM_H = 130;
int servo_LIM_L = 50;
int servo_speed = 15;
int servo_step_size = 5;
unsigned long ServoPreviousMillis = 0;
unsigned long ServoDelay = 50;  // Delay in milliseconds;

volatile int servo_pos = 90;
int targetPos = 90;
int charge_level = 0;

bool is_L_DC_ON = false;
bool is_R_DC_ON = false;

float ints_to_float(int integerPart, int decimalPart);
void float_to_ints(float floatValue, int &integerPart, int &decimalPart);

void receiveEvent(int numBytes);

void requestEvent(void);

void DC_setup();
void DC_loop();

void servo_setup();
void servo_loop();


constexpr char wifi_SSID[] = "5 Titiwai Place";

int32_t obtain_wifi(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nLast Packet Send Status:\t");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}