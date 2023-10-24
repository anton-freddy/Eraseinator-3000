#include <main.h>
#include <freertos/FreeRTOS.h>

void task1code(void *pvParameter)
{
  for (;;)
  {
    lcd_loop();
  }
}

void setup()
{
  Serial.begin(115200);
  TaskHandle_t Task1;

  // Serv
  DC_setup();
  // OTA_setup();
  servo_setup();

  Wire.begin(slave_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  servoState = 1;
  lcd_setup();

  // BaseType_t result = xTaskCreatePinnedToCore(
  //   task1code,
  //   "Task1",
  //   10000,
  //   NULL,
  //   1,
  //   &Task1,
  //   0);

  // if (result != pdPASS) {
  //   Serial.println("Error creating task!");
  // } else {
  //   Serial.println("Task created successfully");
  // }
  if (!LittleFS.begin())
  {

    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  // while(1){
  // servo.write(120);
  //   WebSerial.println(servo.read());
  //   delay(1000);
  //   servo.write(50);
  //   WebSerial.println(servo.read());
  //   delay(1000);
  // }
  // WebSerial.println("End of SetUp");

  // servo.write(90);
  // while(1);
}
int pos = 90;
unsigned long lcd_update_millis = 0;
// bool servo_inc = true;
void loop()
{
  servo.attach(SERVO_pin);
  if (servo_pos >= servo_LIM_H)
  {
    servo_inc = false;
  }
  else if (servo_pos <= servo_LIM_L)
  {
    servo_inc = true;
  }
  if (servo_inc)
  {
    servo_pos += servo_step_size;
  }
  else
  {
    servo_pos -= servo_step_size;
  }
  servo.write(servo_pos);
  Serial.println("Servo pos: " + (String)servo_pos);

  //   //lcd_loop();
  //   DC_loop();
  delay(50);
  //servo_loop();
  //  DC_loop();

  //  if(millis()- lcd_update_millis > 1000){
  //    lcd_update_millis = millis();
  //   //  update_LCD_charge_lvl(charge_level);
  //   //  update_LCD_x_pos(x_pos);
  //   //  update_LCD_y_pos(y_pos);
  //   //  update_LCD_a_pos(a_pos);
  //   lcd_loop();
  //  }
  // servo.attach(SERVO_pin);
  // servo.write(135);
  // delay(1000);
  // servo.write(45);
  // delay(1000);
  // OTA_loop();
  // DC_loop();
}

void receiveEvent(int numBytes)
{

  requestREG = Wire.read(); // Read the register address
  Serial.println("Receive Event: "+(String)requestREG);
  if (numBytes > 1)
  {
    switch (requestREG)
    {
    case REG_SERVO_POS:
    {
      servo_pos = (int)Wire.read();
      WebSerial.println("REG_SERVO_POS RECEIVE EVENT - Received POS: " + (String)targetPos);
      return;
    }
    break;

    case REG_SERVO_STATE:
    {
      int received = Wire.read();
      if (received == 0)
      {
        servoState = CENTER;
      }
      else if (received == 1)
      {
        servoState = RUN;
      }
      else
      {
        servoState = PAUSE;
      }
      WebSerial.println("REG_SERVO_STATE RECEIVE EVENT - Received: " + (String)servoState);
      return;
    }
    break;
    case REG_CHARGE_LEVEL:
    {
      charge_level = Wire.read();
      update_LCD_charge_lvl(charge_level);
      WebSerial.println("REG_CHARGE_LEVEL RECEIVE EVENT - Received: " + (String)charge_level);
      return;
    }
    break;
    case REG_XPOS:
    {
      int num = Wire.read();
      int dec = Wire.read();
      x_pos = ints_to_float(num, dec);
      update_LCD_x_pos(x_pos);
      WebSerial.println("REG_XPOS RECEIVE EVENT - Received: " + (String)x_pos);
      return;
    }
    break;
    case REG_YPOS:
    {
      int num = Wire.read();
      int dec = Wire.read();
      y_pos = ints_to_float(num, dec);
      update_LCD_y_pos(y_pos);
      WebSerial.println("REG_YPOS RECEIVE EVENT - Received: " + (String)y_pos);
      return;
    }
    break;
    case REG_APOS:
    {
      int num = Wire.read();
      int dec = Wire.read();
      a_pos = ints_to_float(num, dec);
      update_LCD_a_pos(a_pos);
      return;
    }
    break;

    case REG_DCL:
    {
      is_L_DC_ON = (int)Wire.read();
      return;
    }
    break;
    case REG_DCR:
    {
      is_R_DC_ON = (int)Wire.read();
      return;
    }
    break;
      // Add more cases for additional registers here

    default:
    {
    }
    break;
    }
  }
}

void requestEvent()
{
  switch (requestREG)
  {
  case REG_SERVO_POS:
  {
    Serial.println("in request funcx");
    uint8_t data = servo_pos;
    Wire.write(&data, sizeof(uint8_t));
  }
  break;

  case REG_SERVO_STATE:
  {
    Wire.write(servoState);
  }
  break;

  case REG_CHARGE_LEVEL:
  {
    Wire.write(charge_level);
  }
  break;

  case REG_APOS:
  {
    int num, dec;
    float_to_ints(a_pos, num, dec);
    Wire.write(num);
    Wire.write(dec);
  }
  break;

  case REG_XPOS:
  {
    int num, dec;
    float_to_ints(x_pos, num, dec);
    Wire.write(num);
    Wire.write(dec);
  }
  break;

  case REG_YPOS:
  {
    int num, dec;
    float_to_ints(y_pos, num, dec);
    Wire.write(num);
    Wire.write(dec);
  }
  break;
  case REG_DCL:
  {
    Wire.write((int)is_L_DC_ON);
  }
  break;
  case REG_DCR:
  {
    Wire.write((int)is_R_DC_ON);
  }
  break;
    // Add more cases for additional registers here

  default:
    break;
  }
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

void DC_setup()
{
  pinMode(RDC_pin, OUTPUT);
  pinMode(LDC_pin, OUTPUT);
  digitalWrite(RDC_pin, LOW);
  digitalWrite(LDC_pin, LOW);
  is_L_DC_ON = LOW;
  is_R_DC_ON = LOW;
}

void DC_loop()
{
  digitalWrite(LDC_pin, (int)is_L_DC_ON);
  digitalWrite(RDC_pin, (int)is_R_DC_ON);
}

void servo_setup()
{
  servo.attach(SERVO_pin);
  servo_pos = 135;
  servo.write(servo_pos);
  delay(1000);
  servo.write(45);
  servoState = RUN;
}

unsigned long servoPrevMilli = 0;
void servo_loop()
{
  // for(servo_pos = servo_pos; servo_pos <= servo_LIM_H; servo_pos++){
  //   servo.write(servo_pos);
  //   delay(servo_speed);
  // }
  // for(servo_pos = servo_pos; servo_pos > servo_LIM_L; servo_pos--){
  //   servo.write(servo_pos);
  //   delay(servo_speed);
  // }
  servo.attach(SERVO_pin);
  Serial.println("LOOP");
  if (servoState == 1)
  {
    if (millis() - servoPrevMilli > servo_speed)
    {
      servoPrevMilli = millis();
      if (servo_pos >= servo_LIM_H)
      {
        servo_inc = false;
      }
      else if (servo_pos <= servo_LIM_L)
      {
        servo_inc = true;
      }
      if (servo_inc)
      {
        servo_pos += servo_step_size;
      }
      else
      {
        servo_pos -= servo_step_size;
      }
      servo.write(servo_pos);
    }
    //   for(servo_pos = servo_pos; servo_pos <= servo_LIM_H; servo_pos++){
    //   servo.write(servo_pos);
    //   delay(servo_speed);
    // }
    // for(servo_pos = servo_pos; servo_pos > servo_LIM_L; servo_pos--){
    //   servo.write(servo_pos);
    //   delay(servo_speed);
    // }
  }
  else if (servoState == 0)
  {
    servo.write(90);
  }
  else
  {
    servo.write(servo_pos);
  }
}