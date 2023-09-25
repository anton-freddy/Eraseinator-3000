#include <main.h>

void setup()
{
  DC_setup();
  Serial.begin(115200);
  OTA_setup();
  //servo_setup();

  // Wire.begin(slave_ADDR);
  // Wire.onReceive(receiveEvent);
  // Wire.onRequest(requestEvent);
  // servoState = RUN;
  lcd_setup();
  // while(1){
  //   servo.write(120);
  //   WebSerial.println(servo.read());
  //   delay(1000);
  //   servo.write(50);
  //   WebSerial.println(servo.read());
  //   delay(1000);
  // }
}

void loop()
{
  OTA_loop();
  // servo_loop();
  lcd_loop();
  // int pos = 90;
  // for(pos = pos; pos < 130; pos++){
  //   servo.attach(SERVO_pin);
  //   servo.write(pos);
  //   servo.detach();
  //   delay(20);
  // }
  // for(pos = pos; pos > 50; pos++){
  //   servo.attach(SERVO_pin);
  //   servo.write(pos);
  //   servo.detach();
  //   delay(20);
  // }
  DC_loop();
}

void receiveEvent(int numBytes)
{
  requestREG = Wire.read(); // Read the register address

  if (numBytes > 1)
  {
    switch (requestREG)
    {
    case REG_SERVO_POS:
    {
      targetPos = (int)Wire.read();
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
      L_DC_state = (int)Wire.read();
      return;
    }
    break;
    case REG_DCR:
    {
      R_DC_state = (int)Wire.read();
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
    Wire.write(servo_pos);
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
    Wire.write(L_DC_state);
  }
  break;
  case REG_DCR:
  {
    Wire.write(R_DC_state);
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
}

void DC_loop()
{
  digitalWrite(LDC_pin, L_DC_state);
  digitalWrite(RDC_pin, R_DC_state);
  if (L_DC_state != LOW || HIGH)
  {
    Serial.println("No Valid State for L DC Motor");
    L_DC_state = LOW;
  }

  if (R_DC_state != LOW || HIGH)
  {
    Serial.println("No Valid State for R DC Motor");
    R_DC_state = LOW;
  }
}

void servo_setup()
{
  pinMode(SERVO_pin, OUTPUT);
  servo.attach(SERVO_pin);
  servo_pos = 90;
  servo.write(servo_pos);
  servoState = CENTER;
}

void servo_loop()
{
  servoMillis.current = millis();
  switch (servoState)
  {

  case RUN:
  {

    if (servoMillis.current - servoMillis.previous >= servo_speed)
    {
      servoMillis.previous = servoMillis.current;

      if (servo_pos <= servo_LIM_L)
      {
        servo_inc = true;
      }
      if (servo_pos >= servo_LIM_H)
      {
        servo_inc = false;
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
  }
  break;

  case PAUSE:
  {
    if (servo.read() != servo_pos)
    {
      servo.write(servo_pos);
    }
  }
  break;

  case CENTER:
  {
    targetPos = 90;
    if (servo.read() != targetPos)
    {
      servo.write(targetPos);
    }
  }
  break;
  }
}