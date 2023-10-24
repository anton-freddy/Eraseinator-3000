
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

void loop2(void);

void task1code(void *pvParameters)
{
  for (;;)
  {
    loop2();
  }
}

void setup()
{

  Serial.begin(115200);
  if (!LittleFS.begin())
  {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }


  //web_setup();

  client_setup();
  is_I2C_setup = setup_I2C();

  I2C_Scan(I2CB);
  I2C_Scan(I2CA);
  setupBatterySense();
  LiDAR1.begin();
  LiDAR2.begin();
  ROOMBA.setUpMotors(L_Stepper_STEP_PIN, L_Stepper_DIR_PIN, L_Stepper_ENABLE_PIN, R_Stepper_STEP_PIN, R_Stepper_DIR_PIN, R_Stepper_ENABLE_PIN, MS1_pin, MS2_pin, MS3_pin);
  ROOMBA.setUpEncoders(I2CB, I2CA);
  ROOMBA.begin(KMH, 50); // 19.1525439
  // Serial.println("BEFORE ENCODER SETUP");
  // L_ENCODER.setWire(I2CB);
  // R_ENCODER.setWire(I2CA);
  // Serial.println("AFTER ENCODER SETUP");
  ROOMBA.movementState = RUNNING;
  // while (1)
  // {

     I2C_Scan(I2CB);

  //   Serial.println("Encoder L: " + (String)L_ENCODER.getRawAngle());
  //   Serial.println("Encoder R: " + (String)R_ENCODER.getRawAngle());
  //   Serial.println("LiDAR 1: " + (String)LiDAR1.getDistance());
  //   Serial.println("LiDAR 2: " + (String)LiDAR2.getDistance());
  //   delay(500);
  // }

  pinMode(LED_RING_pin, OUTPUT);
  digitalWrite(LED_RING_pin, HIGH);

  setupBumpers();
  // server_var.motor_on = 0;
  //  setup_IR();
  //  setup_servo();

  Serial.println("SETUP COMPLETE");

  // while(1){
  //   I2CB.beginTransmission(slave_ADDR);
  //   I2CB.write(REG_SERVO_POS);
  //   I2CB.endTransmission();
  // }
  delay(2000);
    // xTaskCreatePinnedToCore(
    //   task1code, /* Function to implement the task */
    //   "Task1", /* Name of the task */
    //   10000,  /* Stack size in words */
    //   NULL,  /* Task input parameter */
    //   1,  /* Priority of the task */
    //   &Task1,  /* Task handle. */
    //   0); /* Core where the task should run */


  // while(1);
}

bool clockwise = false;
//  Contorl Robot Movement

unsigned long prevMillisBump = 0;

bool bump = false;
int bump_timeout = 0;

bool lidar1_data_send = true;

unsigned long previousMillis_map = 0;
void loop()
{
    ROOMBA.setUp360();
  //client_loop();
  // bump_timeout++;
  // if (bump_triggred)
  // {
  //   setBumpBackOFF();
  //   bump_triggred = false;
  //   bump_timeout = 0;
  //   bump = true;
  // }
  //ROOMBA.loop();
  //Serial.println("LOOP");
  // web_loop();
  //ROOMBA.setPosition(ROOMBA.getOrientation() +0.01);

  if(millis() - previousMillis_map > 50){
    previousMillis_map = millis();

    if(lidar1_data_send){
      lidar1_data_send = false;
      send_lidar1_obj();
      //send_lidar1_obj(obj1_x, obj1_y, LiDAR1.getDistance());
      //send_coordinates(obj1_x, obj1_y, ROOMBA.getXCoordinate(), ROOMBA.getYCoordinate());
    } else {
      lidar1_data_send = true;
      // send_lidar2_obj();
      //send_lidar2_obj(obj1_x, obj1_y, LiDAR2.getDistance());
      //send_coordinates(obj1_x, obj1_y, ROOMBA.getXCoordinate(), ROOMBA.getYCoordinate());
    }

  }

  // send_x_object((ROOMBA.getXCoordinate()+ (9.88 * cos(ROOMBA.getOrientation()) + (3.1 + LiDAR1.getDistance())))/10);
  // send_y_object((ROOMBA.getYCoordinate()+ (9.88 * sin(ROOMBA.getOrientation()) + (3.1 + LiDAR1.getDistance())))/10);

  // dashboard.sendUpdates();

  // web_loop();

  // if (server_var.motor_on == 1)
  // {
  //   ROOMBA.movementState = RUNNING;
  // }
  // else
  // {
  //   ROOMBA.movementState = WAIT;
  // }

  // ROOMBA.loop();
}

void loop2(void)
{
  //web_loop();

  // if (millis() - previousMillis_map > 100)
  // {
  //   previousMillis_map = millis();
  //   float obj1_x, obj1_y, obj2_x, obj2_y;
  //   send_lidar1_obj(obj1_x, obj1_y, LiDAR1.getDistance());
  //   send_lidar2_obj(obj2_x, obj2_y, LiDAR2.getDistance());
  //   // Serial.println("Object @ X: " + (String)obj_x + " Y: " + (String)obj_y);
  //   // Serial.println("Lidar 1:  " + (String)LiDAR1.getDistance());

  //   sendCoordinatesToWeb(obj1_x, obj1_y, ROOMBA.getXCoordinate(), ROOMBA.getYCoordinate());
  //   sendCoordinatesToWeb(obj2_x, obj2_y, ROOMBA.getXCoordinate(), ROOMBA.getYCoordinate());
  // }
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
  return false;
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
  return false;
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
  break;
  }
}

void send_lidar1_obj(void)
{ 
  

  float angle = ROOMBA.getOrientation();
  float lidar1_dist = LiDAR1.getDistance();
  float xRob = ROOMBA.getXCoordinate();
  float yRob = ROOMBA.getYCoordinate();
  float x = xRob + ((LiDAR_offset_center + LiDAR_tolerance_offset + lidar1_dist) * sinf(angle));
  float y = yRob + ((LiDAR_offset_center + LiDAR_tolerance_offset + lidar1_dist) * cosf(angle));

  send_coordinates(x, y, xRob, yRob);
}

void send_lidar2_obj(void)
{
  float lidar2_dist = LiDAR2.getDistance();
  float angle = ROOMBA.getOrientation();
  float xRob = ROOMBA.getXCoordinate();
  float yRob = ROOMBA.getYCoordinate();
  float servo_angle = get_servo_global_angle();
  float x = xRob + (LiDAR_servo_offset_center * sinf(servo_angle)) + (LiDAR_servo_distance + lidar2_dist + LiDAR_tolerance_offset) * sinf(servo_angle);
  float y = yRob + (LiDAR_servo_offset_center * cosf(servo_angle)) + (LiDAR_servo_distance + lidar2_dist + LiDAR_tolerance_offset) * cosf(servo_angle);

  send_coordinates(x, y, xRob, yRob);
}

float get_servo_angle()
{
  int angle = requestIntFromSlave(REG_SERVO_POS);
  Serial.println("Servo Angle: " +(String)angle);
  return  angle;
}
float get_servo_global_angle()
{
  float raw_angle = get_servo_angle();
  float angle = raw_angle - 90;
  float angle_rad = angle * (PI / 180);
  float global_angle = ROOMBA.getOrientation() + angle_rad;
  if (global_angle > 2 * PI)
  {
    global_angle -= 2 * PI;
  }
  else if (global_angle == (2 * PI))
  {
    global_angle = 0;
  }
  else if (global_angle < 0)
  {
    global_angle += 2 * PI;
  }

  return global_angle;
}