#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Adafruit_GC9A01A.h>
#include <display_pics.h>


#define TFT_DC 16
#define TFT_CS 17

#define HSPI_MOSI 23
#define HSPI_SCLK 22
#define TFT_RST 33


struct lcddata {
    int charge_lvl = 0;
    float x_pos = 0;
    float y_pos = 0;
    float a_pos = 0;
};


void lcd_setup();
void lcd_loop();
unsigned long testLines(uint16_t color);
unsigned long testText();
void print_graphics(const uint16_t* array);
void update_LCD_charge_lvl(int charge_lvl);
void update_LCD_x_pos(float x_pos);
void update_LCD_y_pos(float y_pos);
void update_LCD_a_pos(float a_pos);
