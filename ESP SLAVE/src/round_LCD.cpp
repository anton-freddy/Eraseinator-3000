#include <round_LCD.h>

// Define pins for display interface. You'll probably need to edit this for
// your own needs:

// #if defined()

// // Pinout when using Seed Round Display for XIAO in combination with
// // Seeed XIAO RP2040. Other (non-RP2040) XIAO boards, any Adafruit Qt Py
// // boards, and other GC9A01A display breakouts will require different pins.
// #define TFT_CS D1 // Chip select
// #define TFT_DC D3 // Data/command
// #define TFT_BL D6 // Backlight control

// #else // ALL OTHER BOARDS - EDIT AS NEEDED

// Other RP2040-based boards might not have "D" pin defines as shown above
// and will use GPIO bit numbers. On non-RP2040 boards, you can usually use
// pin numbers silkscreened on the board.

// If display breakout has a backlight control pin, that can be defined here
// as TFT_BL. On some breakouts it's not needed, backlight is always on.

// #endif

// Display constructor for primary hardware SPI connection -- the specific
// pins used for writing to the display are unique to each board and are not
// negotiable. "Soft" SPI (using any pins) is an option but performance is
// reduced; it's rarely used, see header file for syntax if needed.

Adafruit_GC9A01A LCD(TFT_CS, TFT_DC /*,HSPI_MOSI,HSPI_SCLK, TFT_RST*/);


lcddata lcd_data;
int battery_lvl = 0;
float x_cord = 0;
float y_cord = 0;


void lcd_setup()
{
  Serial.println("LCD setup start...");

  LCD.begin();

  LCD.fillScreen(GC9A01A_BLACK);
  LCD.setRotation(3);
  LCD.setCursor(50, 120);
  LCD.setTextColor(GC9A01A_WHITE);
  LCD.setTextSize(1);
  LCD.setFont(&FreeSansBold12pt7b);
  LCD.println("Doofenshmirtz");

  delay(5000);
  LCD.fillScreen(GC9A01A_BLACK);
  print_graphics(main_graphic);
  testText();
#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH); // Backlight on
#endif                        // end TFT_BL

}

unsigned long t_PM1 = 0;
unsigned long t_PM2 = 0;
unsigned long t_PM3 = 0;

void lcd_loop(void)
{
  unsigned long currentMillis = millis();
  //delay(500);


  testText();
  //LCD.fillScreen(GC9A01A_BLACK);
}

// TEST FUNCTION

unsigned long testText()
{

  unsigned long start = micros();

  LCD.setCursor(60, 115);
  LCD.setTextColor(GC9A01A_WHITE);
  LCD.setTextSize(1);
  LCD.setFont(&FreeSansBold12pt7b);
  LCD.println("RUNNING");

  LCD.setCursor(55, 170);
  LCD.setTextColor(GC9A01A_WHITE);
  LCD.setTextSize(1);
  LCD.setFont(&FreeSansBold12pt7b);
  LCD.println("100%");

  LCD.setCursor(130, 170);
  LCD.setTextColor(GC9A01A_WHITE);
  LCD.setTextSize(1);
  LCD.setFont(&FreeSansBold12pt7b);
  LCD.println("15%");

  return micros() - start;
}

unsigned long testLines(uint16_t color)
{
  unsigned long start, t;
  int x1, y1, x2, y2,
      w = LCD.width(),
      h = LCD.height();

  LCD.fillScreen(GC9A01A_BLACK);
  yield();

  x1 = y1 = 0;
  y2 = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6)
    LCD.drawLine(x1, y1, x2, y2, color);
  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6)
    LCD.drawLine(x1, y1, x2, y2, color);
  t = micros() - start; // fillScreen doesn't count against timing

  yield();
  LCD.fillScreen(GC9A01A_BLACK);
  yield();

  x1 = w - 1;
  y1 = 0;
  y2 = h - 1;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6)
    LCD.drawLine(x1, y1, x2, y2, color);
  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6)
    LCD.drawLine(x1, y1, x2, y2, color);
  t += micros() - start;

  yield();
  LCD.fillScreen(GC9A01A_BLACK);
  yield();

  x1 = 0;
  y1 = h - 1;
  y2 = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6)
    LCD.drawLine(x1, y1, x2, y2, color);
  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6)
    LCD.drawLine(x1, y1, x2, y2, color);
  t += micros() - start;

  yield();
  LCD.fillScreen(GC9A01A_BLACK);
  yield();

  x1 = w - 1;
  y1 = h - 1;
  y2 = 0;
  start = micros();
  for (x2 = 0; x2 < w; x2 += 6)
    LCD.drawLine(x1, y1, x2, y2, color);
  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6)
    LCD.drawLine(x1, y1, x2, y2, color);

  yield();
  return micros() - start;
}

unsigned long testFastLines(uint16_t color1, uint16_t color2)
{
  unsigned long start;
  int x, y, w = LCD.width(), h = LCD.height();

  LCD.fillScreen(GC9A01A_BLACK);
  start = micros();
  for (y = 0; y < h; y += 5)
    LCD.drawFastHLine(0, y, w, color1);
  for (x = 0; x < w; x += 5)
    LCD.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color)
{
  unsigned long start;
  int n, i, i2,
      cx = LCD.width() / 2,
      cy = LCD.height() / 2;

  LCD.fillScreen(GC9A01A_BLACK);
  n = min(LCD.width(), LCD.height());
  start = micros();
  for (i = 2; i < n; i += 6)
  {
    i2 = i / 2;
    LCD.drawRect(cx - i2, cy - i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2)
{
  unsigned long start, t = 0;
  int n, i, i2,
      cx = LCD.width() / 2 - 1,
      cy = LCD.height() / 2 - 1;

  LCD.fillScreen(GC9A01A_BLACK);
  n = min(LCD.width(), LCD.height());
  for (i = n; i > 0; i -= 6)
  {
    i2 = i / 2;
    start = micros();
    LCD.fillRect(cx - i2, cy - i2, i, i, color1);
    t += micros() - start;
    // Outlines are not included in timing results
    LCD.drawRect(cx - i2, cy - i2, i, i, color2);
    yield();
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color)
{
  unsigned long start;
  int x, y, w = LCD.width(), h = LCD.height(), r2 = radius * 2;

  LCD.fillScreen(GC9A01A_BLACK);
  start = micros();
  for (x = radius; x < w; x += r2)
  {
    for (y = radius; y < h; y += r2)
    {
      LCD.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color)
{
  unsigned long start;
  int x, y, r2 = radius * 2,
            w = LCD.width() + radius,
            h = LCD.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for (x = 0; x < w; x += r2)
  {
    for (y = 0; y < h; y += r2)
    {
      LCD.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles()
{
  unsigned long start;
  int n, i, cx = LCD.width() / 2 - 1,
            cy = LCD.height() / 2 - 1;

  LCD.fillScreen(GC9A01A_BLACK);
  n = min(cx, cy);
  start = micros();
  for (i = 0; i < n; i += 5)
  {
    LCD.drawTriangle(
        cx, cy - i,     // peak
        cx - i, cy + i, // bottom left
        cx + i, cy + i, // bottom right
        LCD.color565(i, i, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles()
{
  unsigned long start, t = 0;
  int i, cx = LCD.width() / 2 - 1,
         cy = LCD.height() / 2 - 1;

  LCD.fillScreen(GC9A01A_BLACK);
  start = micros();
  for (i = min(cx, cy); i > 10; i -= 5)
  {
    start = micros();
    LCD.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     LCD.color565(0, i * 10, i * 10));
    t += micros() - start;
    LCD.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
                     LCD.color565(i * 10, i * 10, 0));
    yield();
  }

  return t;
}

unsigned long testRoundRects()
{
  unsigned long start;
  int w, i, i2,
      cx = LCD.width() / 2 - 1,
      cy = LCD.height() / 2 - 1;

  LCD.fillScreen(GC9A01A_BLACK);
  w = min(LCD.width(), LCD.height());
  start = micros();
  for (i = 0; i < w; i += 6)
  {
    i2 = i / 2;
    LCD.drawRoundRect(cx - i2, cy - i2, i, i, i / 8, LCD.color565(i, 0, 0));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects()
{
  unsigned long start;
  int i, i2,
      cx = LCD.width() / 2 - 1,
      cy = LCD.height() / 2 - 1;

  LCD.fillScreen(GC9A01A_BLACK);
  start = micros();
  for (i = min(LCD.width(), LCD.height()); i > 20; i -= 6)
  {
    i2 = i / 2;
    LCD.fillRoundRect(cx - i2, cy - i2, i, i, i / 8, LCD.color565(0, i, 0));
    yield();
  }

  return micros() - start;
}

void print_graphics(const uint16_t* array){
  LCD.setRotation(3);
  int h = 240, w = 240, row, col, buffidx = 0;
  for (row = 0; row < h; row++)
  { // For each scanline...
    for (col = 0; col < w; col++)
    { // For each pixel...
      // To read from Flash Memory, pgm_read_XXX is required.
      // Since image is stored as uint16_t, pgm_read_word is used as it uses 16bit address
      LCD.drawPixel(col, row, array[0 + buffidx]);
      buffidx++;
    } // end pixel
  }
}

void update_LCD_charge_lvl(int charge_lvl){
  lcd_data.charge_lvl = charge_lvl;
}

void update_LCD_x_pos(float x_pos){
  lcd_data.x_pos = x_pos;
}
void update_LCD_y_pos(float y_pos){
  lcd_data.y_pos = y_pos;
}
void update_LCD_a_pos(float a_pos){
  lcd_data.a_pos = a_pos;
}