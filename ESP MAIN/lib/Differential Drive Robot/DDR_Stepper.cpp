#include <DDR_Stepper.h>

/*******************************************************
  Method: Constructor
  In: wheel_circumfrence (mm)
      wheel_distance (mm)
      MICRO_STEP (1, 2, 4, 8, 16, 32)
      STEPPER_STEP_COUNT (200)
      GEAR_RATIO (1, 2, 4, 8, 16, 32)
  Out: none
  Description: Constructor for the DDR_Control class
*******************************************************/
DDR_Control::DDR_Control(float wheel_circumfrence, float wheel_distance, int MICRO_STEP, int STEPPER_STEP_COUNT, int GEAR_RATIO)
{
  steps_per_rev = MICRO_STEP * STEPPER_STEP_COUNT * 1.0 / GEAR_RATIO;
  wheel_circumfrence_mm = wheel_circumfrence;
  wheel_distance_mm = wheel_distance;
  steps_per_mm = steps_per_rev * (1.0 / wheel_circumfrence_mm);
  micro_step = MICRO_STEP;
  stepper_steps_per_rev = STEPPER_STEP_COUNT;
  gear_ratio = GEAR_RATIO;

  target.a_pos = 0;
  target.y_pos = 0;
  target.x_pos = 0;
}

/*******************************************************
  Method: update stepper DIR pin
  In: none
  Out: none
  Description: Updates the DIR pin for the stepper motors
*******************************************************/
void DDR_Control::update_stepper_DIR_pin()
{
  switch (L_direction)
  {
  case 1:
    digitalWrite(L_D_pin, HIGH);
    break;

  case -1:
    digitalWrite(L_D_pin, LOW);
    break;

  default:
    send_ERROR(LEFT_STEPPER, 0x00);
    break;
  }
  switch (R_direction)
  {
  case 1:
    digitalWrite(R_D_pin, LOW);
    break;

  case -1:
    digitalWrite(R_D_pin, HIGH);
    break;

  default:
    send_ERROR(RIGHT_STEPPER, 0x00);
    break;
  }
}

/*******************************************************
  Method:Move Steppers
  In: none
  Out: none
  Description: Generates the step pulses for the stepper motors, function must be in continous loop with no blocking code
*******************************************************/
void DDR_Control::moveSteppers()
{
  update_stepper_DIR_pin();

  // L_step_time = (steps_per_mm * leftWheelSpeed) / 1000000;
  L_step_time = (1000000 / (steps_per_mm * leftWheelSpeed));
  R_step_time = (1000000 / (steps_per_mm * rightWheelSpeed));
  // R_step_time = (steps_per_mm * rightWheelSpeed) / 1000000;

  if (L_STEP_MOVING)
  {
    //enableStepper(left);
    unsigned long current_time = micros();
    // long current_time = millis();
    if ((current_time - L_previous_time) >= L_step_time)
    {
      L_previous_time = current_time;
      digitalWrite(L_S_pin, HIGH);
    }
    digitalWrite(L_S_pin, LOW);
  }

  if (R_STEP_MOVING)
  {
    //enableStepper(right);
    unsigned long current_time = micros();
    // long current_time = millis();
    if ((current_time - R_previous_time) >= R_step_time)
    {
      R_previous_time = current_time;
      digitalWrite(R_S_pin, HIGH);
    }
    digitalWrite(R_S_pin, LOW);
  }

  //disableStepper(both);
}

/*******************************************************
  Method: Set speed for left stepper
  In: speed (steps/s)
  Out: none
  Description: Sets the speed of the left stepper motor
*******************************************************/
void DDR_Control::L_setSpeed_SPS(long SPS)
{
  if (SPS < 0)
  {
    L_direction = 1;
  }
  else
  {
    L_direction = -1;
  }
  L_Speed_SPS = SPS;
}

/*******************************************************
  Method: Set speed for right stepper
  In: speed (steps/s)
  Out: none
  Description: Sets the speed of the right stepper motor
*******************************************************/
void DDR_Control::R_setSpeed_SPS(long SPS)
{
  if (SPS < 0)
  {
    R_direction = 1;
  }
  else
  {
    R_direction = -1;
  }
  R_Speed_SPS = SPS;
}

/*******************************************************
  Method: Set speed for left stepper
  In: speed (mm/s)
  Out: none
  Description: Sets the speed of the left stepper motor
*******************************************************/
void DDR_Control::L_setSpeed_MMPS(float MMPS)
{
  leftWheelSpeed = abs(MMPS);
  if (MMPS > 0)
  {
    L_direction = 1;
  }
  else if (MMPS < 0)
  {
    L_direction = -1;
  }
}

/*******************************************************
  Method: Set speed for right stepper
  In: speed (mm/s)
  Out: none
  Description: Sets the speed of the right stepper motor
*******************************************************/
void DDR_Control::R_setSpeed_MMPS(float MMPS)
{
  rightWheelSpeed = abs(MMPS);
  if (MMPS > 0)
  {
    R_direction = 1;
  }
  else if (MMPS < 0)
  {
    R_direction = -1;
  }
}

/*******************************************************
  Method: Resume Steppers
  In: motor select (left, right, both)
  Out: none
  Description: Enables the stepper motors
*******************************************************/
void DDR_Control::resumeStepper(motor select)
{
  switch (select)
  {
  case both:
    L_STEP_MOVING = true;
    R_STEP_MOVING = true;
    enableStepper(both);
    break;
  case left:
    L_STEP_MOVING = true;
    enableStepper(left);
    break;

  case right:
    R_STEP_MOVING = true;
    enableStepper(right);
    break;
  default:
    break;
  }
}

/*******************************************************
  Method: Stop Steppers
  In: motor select (left, right, both)
  Out: none
  Description: Disables the stepper motors
*******************************************************/
void DDR_Control::stopStepper(motor select)
{
  switch (select)
  {
  case both:
    L_STEP_MOVING = false;
    R_STEP_MOVING = false;
    disableStepper(both);
    break;
  case left:
    L_STEP_MOVING = false;
    disableStepper(left);
    break;

  case right:
    R_STEP_MOVING = false;
    disableStepper(right);
    break;
  default:
    break;
  }
}

/*******************************************************
  Method: Update Position
  In: change in pos (deltaX, deltaY, deltaOrientation)
  Out: none
  Description: Updates the position by the change in position
*******************************************************/
void DDR_Control::updatePosition(float deltaX, float deltaY, float deltaOrientation)
{
  updatePosition(deltaX, deltaY);
  updatePosition(deltaOrientation);
}

/*******************************************************
  Method: Update Position
  In: change in pos (deltaX, deltaY)
  Out: none
  Description: Updates the position by the change in position
*******************************************************/
void DDR_Control::updatePosition(float deltaX, float deltaY)
{
  x_pos += deltaX;
  y_pos += deltaY;
}

/*******************************************************
  Method: Update Position
  In: change in pos (deltaOrientation)
  Out: none
  Description: Updates the position by the change in position
*******************************************************/
void DDR_Control::updatePosition(float deltaOrientation)
{
  a_pos += deltaOrientation;
  if (a_pos < 0)
  {
    a_pos = a_pos + (2 * PI);
  }
  else if (a_pos > (2 * PI))
  {
    a_pos = a_pos - (2 * PI);
  }
  else if (a_pos == (2 * PI))
  {
    a_pos = 0;
  }
}

/*******************************************************
  Method: Orientation Calculation
  In: Target X and Y
  Out: Target Orientation
*******************************************************/
float DDR_Control::calc_orientation(float target_x, float target_y)
{
  float deltaX = target_x - x_pos;
  float deltaY = target_y - y_pos;
  float target_orientation = 0;
  if (deltaX == 0)
  {
    if (deltaY > 0)
    {
      target_orientation = 0;
    }
    else
    {
      target_orientation = PI;
    }
  }
  else if (deltaY == 0)
  {
    if (deltaX > 0)
    {
      target_orientation = 0.5 * PI;
    }
    else
    {
      target_orientation = 1.5 * PI;
    }
  }
  else if (deltaX > 0)
  {
    if (deltaY > 0)
    {
      target_orientation = atan(deltaX / deltaY);
    }
    else if (deltaY < 0)
    {
      target_orientation = 0.5 * PI + atan(abs(deltaY / deltaX));
    }
    else
    {
      send_ERROR(MOVEMENT, 0x02);
    }
  }
  else if (deltaX < 0)
  {
    if (deltaY < 0)
    {
      target_orientation = PI + atan(abs(deltaX / deltaY));
    }
    else if (deltaY > 0)
    {
      target_orientation = 1.5 * PI + atan(abs(deltaY / deltaX));
    }
    else
    {
      send_ERROR(MOVEMENT, 0x03);
    }
  }
  else
  {
    Serial.println((String)deltaX);
    Serial.println((String)deltaY);
    send_ERROR(MOVEMENT, 0x04);
  }

  if (target_orientation > 2 * PI)
  {
    send_ERROR(MOVEMENT, 0x05);
  }
  else if (target_orientation < 0)
  {
    send_ERROR(MOVEMENT, 0x06);
  }
  else
  {
    if (target_orientation == (2 * PI))
    {
      target_orientation = 0;
    }
  }
  return target_orientation;
}

/*******************************************************
  Method: delta Y Calculation
  In: Diagonal Distance
  Out: delta Y
*******************************************************/
float DDR_Control::calc_delta_Y(float straight_line_dist)
{
  if (a_pos == 0)
  {
    return 0; // No change in X
  }
  else if (a_pos == (0.5 * PI))
  {
    return straight_line_dist; // No change in Y
  }
  else if (a_pos == PI)
  {
    return 0; // No chnage along X axis
  }
  else if (a_pos == 1.5 * PI)
  {
    return -straight_line_dist; // No change in Y
  }
  else if (a_pos > 0 && a_pos < (0.5 * PI))
  {
    return straight_line_dist * sinf(a_pos);
  }
  else if (a_pos > (0.5 * PI) && a_pos < PI)
  {
    return straight_line_dist * sinf(a_pos - (0.5 * PI));
  }
  else if (a_pos > PI && a_pos < (1.5 * PI))
  {
    return -(straight_line_dist * sinf(a_pos - PI));
  }
  else if (a_pos > (1.5 * PI) && a_pos < (2 * PI))
  {
    return -(straight_line_dist * sinf(a_pos - (1.5 * PI)));
  }
  else
  {
    send_ERROR(MOVEMENT, 0x07);
    return 0;
  }
}

/*******************************************************
  Method: delta X Calculation
  In: Diagonal Distance
  Out: delta X
*******************************************************/
float DDR_Control::calc_delta_X(float straight_line_dist)
{
  if (a_pos == 0)
  {
    return 0; // No change in X
  }
  else if (a_pos == (0.5 * PI))
  {
    return straight_line_dist; // No change in Y
  }
  else if (a_pos == PI)
  {
    return 0; // No chnage along X axis
  }
  else if (a_pos == 1.5 * PI)
  {
    return -straight_line_dist; // No change in Y
  }
  else if (a_pos > 0 && a_pos < (0.5 * PI))
  {
    return straight_line_dist * cosf(a_pos);
  }
  else if (a_pos > (0.5 * PI) && a_pos < PI)
  {
    return straight_line_dist * cosf(a_pos - (0.5 * PI));
  }
  else if (a_pos > PI && a_pos < (1.5 * PI))
  {
    return -(straight_line_dist * cosf(a_pos - PI));
  }
  else if (a_pos > (1.5 * PI) && a_pos < (2 * PI))
  {
    return -(straight_line_dist * cosf(a_pos - (1.5 * PI)));
  }
  else
  {
    send_ERROR(MOVEMENT, 0x08);
    return 0;
  }
}

/*******************************************************
  Method: Diagonal Distance Calculation
  In: Traget X & Y
  Out: Diagonal Distance
*******************************************************/
float DDR_Control::calc_diagonal_distance(float target_x, float target_y)
{
  float deltaX = target_x - x_pos;
  float deltaY = target_y - y_pos;
  float distance_diagonal;

  if (deltaX == 0)
  {
    distance_diagonal = deltaY;
  }
  else if (deltaY == 0)
  {
    distance_diagonal = deltaX;
  }
  else
  {
    distance_diagonal = sqrt(powf(deltaX, 2) + powf(deltaY, 2)); // abs(deltaX / sin(getOrientation())); // sqrt(pow(deltaX, 2) + pow(deltaY, 2)); // Need to add signed to quadrant 3 & 4
  }
  return distance_diagonal;
}

/*******************************************************
  Method: delta Orientation Calculation
  In: Target Orientation
  Out: delta Orientation
*******************************************************/
float DDR_Control::get_delta_theta(float target)
{
  float angleDiff = target - getOrientation();
  if (angleDiff > PI)
  {
    return angleDiff - (2 * PI);
  }
  else if (angleDiff < -PI)
  {
    return angleDiff + (2 * PI);
  }
  else
  {
    return angleDiff;
  }
}

/*******************************************************
  Method: Initialize the robot
  In: speed unit (KMH, MMS)
      speed (speed in units)
  Out: none
*******************************************************/
void DDR_Control::begin(unit speed_units, float speed)
{

  if (speed_units == KMH)
  {
    setSpeedInKMH(speed);
    defaultSpeed_MMS = speed * 277.7777777778;
    Serial.println(defaultSpeed_MMS);
    Serial.println(leftWheelSpeed);
  }
  else
  {
    setSpeedInMMS(speed);
    defaultSpeed_MMS = speed;
  }

  x_pos = 0;
  y_pos = 0;
  a_pos = 0;
  L_MOVE_DONE = true;
  R_MOVE_DONE = true;
  ROT_MOVE = false;
  STR_MOVE = false;
  previous_cord_LS = 0;
  previous_cord_RS = 0;
  current_cord_LS = 0;
  current_cord_RS = 0;
  target_cord_LS = 0;
  target_cord_RS = 0;

  Serial.println("ROOMBA SETUP COMPLETE");
}

/*******************************************************
  Method: Add moves to buffer
  In: xy coordinates
  Out: none
*******************************************************/
void DDR_Control::enqueueMove(float x, float y)
{
  Move *newMove = new Move;
  newMove->x = x;
  newMove->y = y;
  newMove->next = nullptr;

  if (tail == nullptr)
  {
    head = newMove;
    tail = newMove;
  }
  else
  {
    tail->next = newMove;
    tail = newMove;
  }
}

/*******************************************************
  Method: Add moves to buffer before current move
  In: xy coordinates
  Out: none
*******************************************************/
void DDR_Control::insertMoveBeforeCurrent(float x, float y)
{
  Move *newMove = new Move;
  newMove->x = x;
  newMove->y = y;

  if (head == nullptr)
  {
    head = newMove;
    tail = newMove;
    newMove->next = nullptr;
  }
  else
  {
    newMove->next = head;
    head = newMove;
  }
}

/*******************************************************
  Method: Executes the next move in the buffer
  In: none
  Out: returns true if the buffer is empty
*******************************************************/
bool DDR_Control::executeMoves()
{
  if (head != nullptr)
  {
    Move *current = head;
    setUpMove(current->x, current->y);

    head = current->next;
    delete current; // Free memory for the completed move
    return false;
  }

  return true;
}

/*******************************************************
  Method: Clears the move buffer
  In: none
  Out: none
*******************************************************/
void DDR_Control::clearMoves()
{
  while (head != nullptr)
  {
    Move *current = head;
    head = current->next;
    delete current; // Free memory for the removed move
  }
  head = nullptr; // Reset the head pointer to nullptr since the list is empty
  tail = nullptr; // Reset the tail pointer to nullptr since the list is empty
}

/*******************************************************
  Method: Add moves to buffer
  In: xy coordinates
  Out: none
*******************************************************/
void DDR_Control::relative_enqueueMove(float x, float y)
{
  Move *newMove = new Move;
  newMove->x = x;
  newMove->y = y;
  newMove->next = nullptr;

  if (tail == nullptr)
  {
    head = newMove;
    tail = newMove;
  }
  else
  {
    tail->next = newMove;
    tail = newMove;
  }
}

/*******************************************************
  Method: Add moves to buffer before current move
  In: xy coordinates
  Out: none
*******************************************************/
void DDR_Control::relative_insertMoveBeforeCurrent(float x, float y)
{
  Move *newMove = new Move;
  newMove->x = x;
  newMove->y = y;

  if (relative_head == nullptr)
  {
    relative_head = newMove;
    relative_tail = newMove;
    newMove->next = nullptr;
  }
  else
  {
    newMove->next = relative_head;
    relative_head = newMove;
  }
}

/*******************************************************
  Method: Executes the next move in the buffer
  In: none
  Out: returns true if the buffer is empty
*******************************************************/
bool DDR_Control::relative_executeMoves()
{
  if (relative_head != nullptr)
  {
    Move *current = relative_head;
    setUpMove(current->x, current->y);

    relative_head = current->next;
    delete current; // Free memory for the completed move
    return false;
  }

  return true;
}

/*******************************************************
  Method: Clears the move buffer
  In: none
  Out: none
*******************************************************/
void DDR_Control::relative_clearMoves()
{
  while (relative_head != nullptr)
  {
    Move *current = head;
    relative_head = current->next;
    delete current; // Free memory for the removed move
  }
  relative_head = nullptr; // Reset the head pointer to nullptr since the list is empty
  relative_tail = nullptr; // Reset the tail pointer to nullptr since the list is empty
}

/*******************************************************
  Method: update the current position
  In: none
  Out: none
*******************************************************/
void DDR_Control::updatePose()
{

  float L_ENC_READ = getEncoderAngle(left);
  float R_ENC_READ = getEncoderAngle(right);
  unsigned long currentTime = millis();
  float elapsedTime = (currentTime - pose_previousTime) / 1000.0; // Convert to seconds

  // Read encoder values

  float L_delta_theta = -1 * (L_ENC_READ - L_ENC_PREVIOUS);
  float R_delta_theta = -1 * (R_ENC_READ - R_ENC_PREVIOUS);
  L_ENC_PREVIOUS = L_ENC_READ;
  R_ENC_PREVIOUS = R_ENC_READ;
  // Calculate the change in angle accounting for overflow
  if (L_delta_theta > 180)
  {
    L_delta_theta -= 360.0; // Account for counter-clockwise movement
  }
  else if (L_delta_theta < -180)
  {
    L_delta_theta += 360.0; // Account for clockwise movement
  }

  if (R_delta_theta > 180)
  {
    R_delta_theta -= 360.0; // Account for counter-clockwise movement
  }
  else if (R_delta_theta < -180)
  {
    R_delta_theta += 360.0; // Account for clockwise movement
  }
  float leftDist = (L_delta_theta / 360.0) * (wheel_circumfrence_mm) * (float)(1.0 / gear_ratio);  // Function to get left encoder distance change in cm
  float rightDist = (R_delta_theta / 360.0) * (wheel_circumfrence_mm) * (float)(1.0 / gear_ratio); // Function to get right encoder distance change in cm

  // Calculate wheel velocities
  float leftVel = (leftDist /* - L_prev_dist */) / elapsedTime;
  float rightVel = (rightDist /* - R_prev_dist*/) / elapsedTime;

  // Calculate robot linear and angular velocities
  float linearVel = (leftVel + rightVel) / 2.0;
  float angularVel = (leftVel - rightVel) / (wheel_distance_mm);

  // Update robot pose
  updatePosition(angularVel * elapsedTime);
  updatePosition((linearVel * sin(getOrientation()) * elapsedTime), (linearVel * cos(getOrientation()) * elapsedTime));

  // Store current values for next iteration
  L_prev_dist = leftDist;
  R_prev_dist = rightDist;
  pose_previousTime = currentTime;

  // unsigned long currentMillis = millis();
  // if (currentMillis > previousMillis___POSE + 1000)
  // {
  //   previousMillis___POSE = currentMillis;
  //   String ArrayLine;
  //   ArrayLine = (String)millis() + "\t";
  //   ArrayLine += "X: " + (String)getXCoordinate() + " Y: " + (String)getYCoordinate() + " A: " + (String)getOrientation();
  //   // ArrayLine += "\nLeft Step Time: " + (String)L_step_time + " Right Step Time: " + (String)R_step_time;
  //   Serial.println(ArrayLine);
  //   // Serial.println("L: " + (String)L_delta_theta);
  //   // Serial.println("R: " + (String)R_delta_theta);
  // }
}

/*******************************************************
  Method: Override the current position
  In: New Orientation
  Out: none
*******************************************************/
void DDR_Control::setPosition(float angle)
{
  a_pos = angle;
}

/*******************************************************
  Method: Override the current position
  In: New X pos and Y pos
  Out: none
*******************************************************/
void DDR_Control::setPosition(float xPos, float yPos)
{
  x_pos = xPos;
  y_pos = yPos;
}

/*******************************************************
  Method: Override the current position
  In: New X pos, Y pos and Orientation
  Out: none
*******************************************************/
void DDR_Control::setPosition(float xPos, float yPos, float angle)
{
  x_pos = xPos;
  y_pos = yPos;
  a_pos = angle;
}

/*******************************************************
  Method: Get Current Position
  In: none
  Out: position
*******************************************************/
float DDR_Control::getXCoordinate() const
{
  return x_pos;
}

/*******************************************************
  Method: Get Current Position
  In: none
  Out: position
*******************************************************/
float DDR_Control::getYCoordinate() const
{
  return y_pos;
}

/*******************************************************
  Method: Get Current Position
  In: none
  Out: position (radians)
*******************************************************/
float DDR_Control::getOrientation() const
{
  return a_pos;
}

// Sets the speed of the robot in KM/h, !!NOTE!! you must set the steps per millimeter before setting the speed
void DDR_Control::setSpeedInKMH(float speed)
{
  setSpeedInMMS(speed * 277.7777777778);
}

// Sets the speed of the robot in mm/s, !!NOTE!! you must set the steps per millimeter before setting the speed
void DDR_Control::setSpeedInMMS(float speed)
{
  L_setSpeed_MMPS(speed);
  R_setSpeed_MMPS(speed);
}

//--  Public Functions ----------------------------------------

// Initialize the robot, for unit pick either KMH or MMS

void DDR_Control::setUpMotors(byte leftMotorStepPin, byte leftMotorDirPin, byte leftMotorEnablePin, byte rightMotorStepPin, byte rightMotorDirPin, byte rightMotorEnablePin)
{
  L_S_pin = leftMotorStepPin;
  L_D_pin = leftMotorDirPin;
  L_E_pin = leftMotorEnablePin;
  pinMode(L_S_pin, OUTPUT);
  pinMode(L_D_pin, OUTPUT);
  pinMode(L_E_pin, OUTPUT);
  digitalWrite(L_S_pin, LOW);
  digitalWrite(L_D_pin, LOW);
  digitalWrite(L_E_pin, LOW);

  R_S_pin = 17; // rightMotorStepPin;
  R_D_pin = rightMotorDirPin;
  R_E_pin = rightMotorEnablePin;
  pinMode(R_S_pin, OUTPUT);
  pinMode(R_D_pin, OUTPUT);
  pinMode(R_E_pin, OUTPUT);
  digitalWrite(R_S_pin, LOW);
  digitalWrite(R_D_pin, LOW);
  digitalWrite(R_E_pin, LOW);

  pinMode(MS1_pin, OUTPUT);
  pinMode(MS2_pin, OUTPUT);
  pinMode(MS3_pin, OUTPUT);

  if (MS1_pin != 0 && MS2_pin != 0 && MS3_pin != 0)
  {
    switch (micro_step)
    {
    case 2:
    {
      digitalWrite(MS1_pin, HIGH);
      digitalWrite(MS2_pin, LOW);
      digitalWrite(MS3_pin, LOW);
    }
    break;

    case 4:
    {
      digitalWrite(MS1_pin, LOW);
      digitalWrite(MS2_pin, HIGH);
      digitalWrite(MS3_pin, LOW);
    }
    break;

    case 8:
    {
      digitalWrite(MS1_pin, HIGH);
      digitalWrite(MS2_pin, HIGH);
      digitalWrite(MS3_pin, LOW);
    }
    break;

    case 16:
    {
      digitalWrite(MS1_pin, HIGH);
      digitalWrite(MS2_pin, HIGH);
      digitalWrite(MS3_pin, HIGH);
    }
    break;

    default:
    {
      digitalWrite(MS1_pin, LOW);
      digitalWrite(MS2_pin, LOW);
      digitalWrite(MS3_pin, LOW);
    }
    break;
    }
  }

  Serial.println("MOTOR SETUP COMPLETE");
}

void DDR_Control::setUpMotors(byte leftMotorStepPin, byte leftMotorDirPin, byte leftMotorEnablePin, byte rightMotorStepPin, byte rightMotorDirPin, byte rightMotorEnablePin, byte MS1Pin, byte MS2Pin, byte MS3Pin)
{
  MS1_pin = MS1Pin;
  MS2_pin = MS2Pin;
  MS3_pin = MS3Pin;

  setUpMotors(leftMotorStepPin, leftMotorDirPin, leftMotorEnablePin, rightMotorStepPin, rightMotorDirPin, rightMotorEnablePin);
}

void DDR_Control::setUpEncoders()
{

  if (L_ENCODER.detectMagnet() == 0)
  {
    send_ERROR(LEFT_ENCODER, 0x00);
  }
  else if (L_ENCODER.detectMagnet() == 1)
  {
    Serial.println("L ENCODER MAGNET FOUND");
  }

  if (R_ENCODER.detectMagnet() == 0)
  {
    send_ERROR(RIGHT_ENCODER, 0x00);
  }
  else if (R_ENCODER.detectMagnet() == 1)
  {
    Serial.println("R ENCODER MAGNET FOUND");
  }
  resetEncoders(both);

  Serial.println("ENCODER SETUP COMPLETE");
}

void DDR_Control::resetEncoders(motor selector)
{

  switch (selector)
  {
  case left:
    L_ENC_PREVIOUS = getEncoderAngle(left);
    break;

  case right:
    R_ENC_PREVIOUS = getEncoderAngle(right);
    break;

  default:
    L_ENC_PREVIOUS = getEncoderAngle(left);
    R_ENC_PREVIOUS = getEncoderAngle(right);
    break;
  }
}

float DDR_Control::getEncoderAngle(motor identifier)
{
  float x, in_min, in_max, out_min, out_max;
  in_min = 0;
  in_max = 4095;
  out_min = 0;
  out_max = 360;
  float newAngle = -1;
  switch (identifier)
  {
  case left:
    newAngle = L_ENCODER.getRawAngle();
    break;

  case right:
    newAngle = R_ENCODER.getRawAngle();
    break;

  default:
    send_ERROR(ENCODERS, 0x02);
    return 0;
  }
  x = newAngle;
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);

  /* Raw data reports 0 - 4095 segments, which is 0.087890625 of a degree */
  // return (float)map_f(newAngle,0.0,4095.0,0.0,360.0);//(newAngle * 0.087890625)
}



long previous___Millis = 0;

void DDR_Control::setUpMove(float target_x, float target_y)
{

  target.a_pos = calc_orientation(target_x, target_y);
  target.x_pos = target_x;
  target.y_pos = target_y;
  if (target.a_pos != getOrientation())
  {
    movementState = TURN;
  }
  else
  {
    movementState = STRAIGHT;
  }
}

void DDR_Control::setUpTurn(float target_a)
{
  target.a_pos = target_a;
  MoveState previous = movementState;
  if (target.a_pos != getOrientation())
  {
    movementState = TURN;
  }
  else
  {
    movementState = previous;
  }
}

// Stop the robot // This is a blocking function while to robot decelerates to come to a stop
void DDR_Control::stop()
{
  stopStepper(both);
}

void DDR_Control::resume()
{
  resumeStepper(both);
}

void DDR_Control::enableStepper(motor select)
{
  if (select == left)
  {
    digitalWrite(L_E_pin, LOW);
  }
  if (select == right)
  {
    digitalWrite(R_E_pin, LOW);
  }
  if (select == both)
  {
    digitalWrite(R_E_pin, LOW);
    digitalWrite(L_E_pin, LOW);
  }
}
void DDR_Control::disableStepper(motor select)
{
  if (select == left)
  {
    digitalWrite(L_E_pin, HIGH);
  }
  if (select == right)
  {
    digitalWrite(R_E_pin, HIGH);
  }
  if (select == both)
  {
    digitalWrite(R_E_pin, HIGH);
    digitalWrite(L_E_pin, HIGH);
  }
}

float DDR_Control::toDeg(float rad)
{
  return (int)(rad * (180.0 / PI));
}
float DDR_Control::toRad(float deg)
{
  return deg * (PI / 180.0);
}


float DDR_Control::L_getCurrentSpeed_MMS(void)
{
  return leftWheelSpeed;
}

float DDR_Control::R_getCurrentSpeed_MMS(void)
{
  return rightWheelSpeed;
}


void DDR_Control::loop()
{
  moveSteppers();
  updatePose();
  switch (movementState)
  {
  case IDLE:
  {
    setSpeedInMMS(defaultSpeed_MMS);
    if (!executeMoves())
    {

      break;
    }
    if (obsticale_avoiding)
    {
      obsticale_avoiding = false;
      target.a_pos = obsticale_prev_orientation;
      movementState = TURN;
      break;
    }
    movementState = RUNNING;
    target.a_pos = getOrientation();

    Serial.println("IN IDLE");
  }
  break;

  case BACK_OFF:
  {
    resumeStepper(both);
    setSpeedInMMS(-1 * defaultSpeed_MMS);
    backOff_current_millis = millis();
    if (backOff_current_millis >= backOff_previous_millis + 2000)
    {
      movementState = IDLE;
      stopStepper(both);
    }
  }
  break;
  case TURN:
  {
    int turningSpeed = defaultSpeed_MMS;
    if (abs(target.a_pos - getOrientation()) <= ThetaTolernce)
    {
      Serial.println("TURN COMPLETE");
      stopStepper(both);
      setSpeedInMMS(defaultSpeed_MMS);
      movementState = STRAIGHT;
      break;
    }
    float deltaTheta = target.a_pos - getOrientation();
    if (deltaTheta > PI)
    {
      deltaTheta = deltaTheta - (2 * PI);
    }
    else if (deltaTheta < -PI)
    {
      deltaTheta = deltaTheta + (2 * PI);
    }

    if (deltaTheta < 0)
    {
      L_setSpeed_MMPS(-turningSpeed);
      R_setSpeed_MMPS(turningSpeed);
      resumeStepper(both);
    }
    else if (deltaTheta > 0)
    {
      L_setSpeed_MMPS(turningSpeed);
      R_setSpeed_MMPS(-turningSpeed);
      resumeStepper(both);
    }
  }
  break;

  case STRAIGHT:
  {

    float diag_distance_to_target = calc_diagonal_distance(target.x_pos, target.y_pos);
    if (diag_distance_to_target <= DistanceTolerance)
    {
      Serial.println("STRAIGH COMPLETE");
      stopStepper(both);
      setSpeedInMMS(defaultSpeed_MMS);
      movementState = IDLE;
    }
    else if (diag_distance_to_target)
    {

      float desiredTheta = calc_orientation(target.x_pos, target.y_pos);
      float angleDiff = get_delta_theta(desiredTheta);
      // Adjust wheel speeds to correct orientation error
      leftWheelSpeed += 0.1 * angleDiff;   // You can adjust this factor
      rightWheelSpeed += -0.1 * angleDiff; // You can adjust this factor
      L_setSpeed_MMPS(leftWheelSpeed);
      R_setSpeed_MMPS(rightWheelSpeed);
      resumeStepper(both);
      // robot.setWheelSpeeds(leftWheelSpeed, rightWheelSpeed);
    }
    // Implement motor control logic here
    // Adjust motor speeds based on calculated speeds and desired movement
  }
  break;

  case RUNNING:
  {
    resumeStepper(both);
    float angleDiff = get_delta_theta(target.a_pos);
    leftWheelSpeed += 0.1 * angleDiff;   // You can adjust this factor
    rightWheelSpeed += -0.1 * angleDiff; // You can adjust this factor
    L_setSpeed_MMPS(leftWheelSpeed);
    R_setSpeed_MMPS(rightWheelSpeed);
    //Serial.println("RUNNING @ " + (String)leftWheelSpeed + ", " + (String)rightWheelSpeed);
  }
  break;

  case WAIT:
  {
    stopStepper(both);
  }
  break;

  case ERROR:
  {
    send_ERROR(MOVEMENT, 0x01);
  }
  break;
  }
}

void DDR_Control::calibrateMPU()
{
  // delay(5000);

  // // calibrate anytime you want to
  // Serial.println("Accel Gyro calibration will start in 5sec.");
  // Serial.println("Please leave the device still on the flat plane.");
  // mpu.verbose(true);
  // delay(5000);
  // mpu.calibrateAccelGyro();
  // mpu.setMagneticDeclination(20.11);
  Serial.println("Please move in a figure of eight while mag calibartes");
  delay(2000);
  mpu.calibrateMag();

  print_MPU_calibration();
  mpu.verbose(false);
}

void DDR_Control::setUp360(){
  L_setSpeed_MMPS(defaultSpeed_MMS);
  R_setSpeed_MMPS(-defaultSpeed_MMS);
  resumeStepper(both);
  updatePose();
  moveSteppers();
}

void DDR_Control::print_MPU_calibration()
{
  Serial.println("< calibration parameters >");
  Serial.println("accel bias [g]: ");
  Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
  Serial.println();
  Serial.println("gyro bias [deg/s]: ");
  Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.print(", ");
  Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
  Serial.println();
  Serial.println("mag bias [mG]: ");
  Serial.print(mpu.getMagBiasX());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasY());
  Serial.print(", ");
  Serial.print(mpu.getMagBiasZ());
  Serial.println();
  Serial.println("mag scale []: ");
  Serial.print(mpu.getMagScaleX());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleY());
  Serial.print(", ");
  Serial.print(mpu.getMagScaleZ());
  Serial.println();
}