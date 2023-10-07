
#include <main.h>
#include <ERROR.h>

TwoWire I2CA = TwoWire(0);
TwoWire I2CB = TwoWire(1);

DDR_Control ROOMBA(WHEEL_CIRCUMFERENCE, WHEEL_DISTANCE, MICROSTEP, STEPPER_STEP_COUNT, GEAR_RATIO);

TF_LUNA LiDAR1(LiDAR_ADD_1, LiDAR_frame_rate, LiDAR_1, I2CA);
TF_LUNA LiDAR2(LiDAR_ADD_2, LiDAR_frame_rate, LiDAR_2, I2CA);

// AS5600_ENC L_ENCODER;
// AS5600_ENC R_ENCODER;


int loop1_previousMillis = 0;
int loop1_currentMillis = 0;

long previousMillis = 0;
long currentMillis = 0;

void setup()
{

  Serial.begin(115200);
  // if (!LittleFS.begin())
  // {
  //   Serial.println("An Error has occurred while mounting LittleFS");
  //   return;
  // }
  // web_setup();
  is_I2C_setup = setup_I2C();

  //I2C_Scan(I2CA);
  setupBatterySense();
  LiDAR1.begin();
  LiDAR2.begin();
  // Serial.println("BEFORE ENCODER SETUP");
  // L_ENCODER.setWire(I2CB);
  // R_ENCODER.setWire(I2CA);
  // Serial.println("AFTER ENCODER SETUP");

  // while (1)
  // {
    
  //   Serial.println("Encoder L: " + (String)L_ENCODER.getRawAngle());
  //   Serial.println("Encoder R: " + (String)R_ENCODER.getRawAngle());
  //   Serial.println("LiDAR 1: " + (String)LiDAR1.getDistance());
  //   Serial.println("LiDAR 2: " + (String)LiDAR2.getDistance());
  //   delay(500);
  // }
  

  ROOMBA.setUpMotors(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN, MS1_pin, MS2_pin, MS3_pin);
  ROOMBA.setUpEncoders(I2CB, I2CA);
  ROOMBA.begin(KMH, 10); // 19.1525439
  pinMode(LED_RING_pin, OUTPUT);
  digitalWrite(LED_RING_pin, HIGH);

  setupBumpers();
  // server_var.motor_on = 0;
  //  setup_IR();
  //  setup_servo();

  delay(1000);
  // while(1);
}

bool clockwise = false;
//  Contorl Robot Movement

unsigned long prevMillisBump = 0;

bool bump = false;
int bump_timeout = 0;
void loop()
{
  bump_timeout++;
  if (bump_triggred)
  {
    setBumpBackOFF();
    bump_triggred = false;
    bump_timeout = 0;
    bump = true;
  }
  //Serial.println(LiDAR1.getDistance());

  // web_loop();

  // send_x_object((ROOMBA.getXCoordinate()+ (9.88 * cos(ROOMBA.getOrientation()) + (3.1 + LiDAR1.getDistance())))/10);
  // send_y_object((ROOMBA.getYCoordinate()+ (9.88 * sin(ROOMBA.getOrientation()) + (3.1 + LiDAR1.getDistance())))/10);
  // if (millis() - prevMillisBump >= 200)
  // {
  //   prevMillisBump = millis();
  //   if (bump)
  //   {
  //     card_bump_state.update("ACTIVE");
  //     if (bump_timeout > 1000)
  //     {
  //       bump = false;
  //     }
  //   }
  //   else
  //   {
  //     card_bump_state.update("IDLE");
  //   }
  // }
  // card_lidar_1.update(LiDAR1.getDistance());
  // card_lidar_2.update(LiDAR2.getDistance());
  // card_charge_level.update(battery.level());
  // dashboard.sendUpdates();

  // if (server_var.led_on == 1)
  // {
  //   digitalWrite(LED_RING_pin, HIGH);
  // }
  // else
  // {
  //   digitalWrite(LED_RING_pin, LOW);
  // }

  // if (server_var.motor_on == 1)
  // {
  //   ROOMBA.movementState = RUNNING;
  // }
  // else
  // {
  //   ROOMBA.movementState = WAIT;
  // }

  //ROOMBA.loop();
}

void setup_IR()
{
  pinMode(L_IR_PIN, INPUT);
  pinMode(R_IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(R_IR_PIN), IR1_ISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(L_IR_PIN), IR2_ISR, RISING);
  Serial.println("IR SETUP COMPLETE");
}
bool get_IR1_status()
{
  int status = digitalRead(L_IR_PIN);
  if (status == HIGH)
  {
    return true;
  }
  else
  {
    return false;
  }
}
bool get_IR2_status()
{
  int status = digitalRead(R_IR_PIN);
  if (status == HIGH)
  {
    return true;
  }
  else if (status == LOW)
  {
    return false;
  }
}

void IR1_ISR()
{
  ROOMBA.stop();
}

void IR2_ISR()
{
  ROOMBA.stop();
}

void setup_servo()
{
  servo_pos = 90;
  writeIntToSlave(REG_SERVO_POS, servo_pos);
}

void writeToSlave(uint8_t regAddress, uint8_t data)
{
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(regAddress); // Register address
  I2CB.write(data);
  I2CB.endTransmission();
}

void writeIntToSlave(uint8_t regAddress, int data)
{
  uint8_t temp = data;
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.write(&temp, sizeof(uint8_t));
  I2CB.endTransmission();
}
void writeFloatToSlave(uint8_t regAddress, float data)
{
  int num, dec;
  float_to_ints(data, num, dec);
  uint8_t temp1 = num;
  uint8_t temp2 = dec;
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.write(&temp1, sizeof(uint8_t));
  I2CB.write(&temp2, sizeof(uint8_t));
  I2CB.endTransmission();
}

int requestIntFromSlave(uint8_t regAddress)
{
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.endTransmission();

  I2CB.requestFrom(slave_ADDR, sizeof(uint8_t));
  uint8_t read;
  I2CB.readBytes(&read, sizeof(uint8_t));
  return read;
}

float requestFloatFromSlave(uint8_t regAddress)
{
  I2CB.beginTransmission(slave_ADDR);
  I2CB.write(&regAddress, sizeof(uint8_t));
  I2CB.endTransmission();

  I2CB.requestFrom(slave_ADDR, 2 * sizeof(uint8_t));
  uint8_t num = 0;
  uint8_t dec = 0;
  I2CB.readBytes(&num, sizeof(uint8_t));
  I2CB.readBytes(&dec, sizeof(uint8_t));

  return ints_to_float(num, dec);
}

bool setup_I2C()
{
  if (I2CA.begin(I2CA_SDA, I2CA_SCL, 100000) && I2CB.begin(I2CB_SDA, I2CB_SCL, 100000))
  {
    return true;
  }
  else
    return false;
}

void bump_ISR()
{
  bump_triggred = true;
}

void setBumpBackOFF()
{
  Serial.println("BUMP TRIGRRED");
  ROOMBA.movementState = BACK_OFF;
  ROOMBA.obsticale_xpos = ROOMBA.getXCoordinate();
  ROOMBA.obsticale_ypos = ROOMBA.getYCoordinate();
  ROOMBA.clearMoves();
  ROOMBA.obsticale_avoiding = true;
  ROOMBA.obsticale_prev_orientation = ROOMBA.getOrientation();
  float arcLength = PI * obstciale_radius;
  int numWaypoints = int(arcLength / 50);
  float angleIncrement = PI / numWaypoints;
  float orientation = ROOMBA.getOrientation();
  for (int i = numWaypoints / 2; i < numWaypoints; i++)
  {

    float deltaX = obstciale_radius * cos(orientation); // Calculate new x-coordinate increment
    float deltaY = obstciale_radius * sin(orientation); // Calculate new y-coordinate increment
    ROOMBA.enqueueMove(deltaX, deltaY);

    orientation += angleIncrement;
    // Handle orientation overflow
    if (orientation > 2 * PI)
    {
      orientation -= 2 * PI;
    }
    else if (orientation < 0)
    {
      orientation += 2 * PI;
    }
  }
  ROOMBA.backOff_previous_millis = millis();
  return;
}

void setupBumpers()
{
  pinMode(C_BUMP_PIN, INPUT);
  pinMode(R_BUMP_PIN, INPUT_PULLUP);
  pinMode(L_BUMP_PIN, INPUT_PULLUP);
  attachInterrupt(C_BUMP_PIN, bump_ISR, HIGH);
  attachInterrupt(R_BUMP_PIN, bump_ISR, HIGH);
  attachInterrupt(L_BUMP_PIN, bump_ISR, HIGH);
}

void tickServo()
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_Servo >= servo_delayBetweenSteps)
  {
    previousMillis_Servo = currentMillis;

    if (servo_pos <= 50 || servo_pos >= 130)
    {
      servo_increment = -servo_increment; // Reverse direction when reaching limits
    }

    servo_pos += servo_increment;
    writeIntToSlave(REG_SERVO_POS, servo_pos);
  }
}

void setupBatterySense()
{
  battery.begin(3300, 3.939, &linear);
}

void setDCmotor(int motor_select, bool active)
{

  switch (motor_select)
  {
  case left:
  {
    writeIntToSlave(REG_DCL, active);
  }
  break;

  case right:
  {
    writeIntToSlave(REG_DCR, active);
  }
  break;

  default:
  {
    writeIntToSlave(REG_DCL, active);
    writeIntToSlave(REG_DCR, active);
  }
  }
}

//  Returns true if motor is running
bool getDCmotor(int motor_select)
{

  switch (motor_select)
  {
  case left:
  {
    return requestIntFromSlave(REG_DCL);
  }
  break;

  case right:
  {
    return requestIntFromSlave(REG_DCR);
  }
  break;
  default:
  {
    send_ERROR(DC_MOTORS, 0x00);
  }
  }
}