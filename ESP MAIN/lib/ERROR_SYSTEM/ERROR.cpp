#include <ERROR.h>

const String ERROR_MSG_START = "\n------------------\nERROR FOUND\nMODULE TYPE: ";
void send_ERROR(int module_code, int error_code)
{

    switch (module_code)
    {
    case LiDAR_1:
    case LiDAR_2:
    case LiDAR_3:
    {
        serial_send_ERROR(getLiDAR_ERRORS(module_code, error_code));
    }
    break;

    case LEFT_STEPPER:
    case RIGHT_STEPPER:
    {
        serial_send_ERROR(getStepper_ERRORS(module_code, error_code));
    }
    break;

    case LEFT_ENCODER:
    case RIGHT_ENCODER:
    case ENCODERS:
    {
        serial_send_ERROR(getEncoder_ERRORS(module_code, error_code));
    }
    break;
    }
}

void serial_send_ERROR(String MSG)
{
    Serial.println(MSG);
    WebSerial.println(MSG);
    
}

String getLiDAR_ERRORS(int module_code, int error_code)
{
    String MSG = ERROR_MSG_START;

    switch (module_code)
    {
    case LiDAR_1:
        MSG += "LiDAR 1 --- ";
        break;
    case LiDAR_2:
        MSG += "LiDAR 2 --- ";
        break;

    case LiDAR_3:
        MSG += "LiDAR 3 --- ";
        break;
    default:
        MSG += (String)module_code + " --- ";
        break;
    }
    if (error_code < 16)
    {
        MSG += "ERROR CODE: 0x0" + String(error_code, HEX);
        MSG += " --- ";
    }
    else
    {
        MSG += "ERROR CODE: 0x" + String(error_code, HEX);
        MSG += " --- ";
    }

    switch (error_code)
    {
    case 0x00:
        MSG += "Frame Rate not set";
        break;

    case 0x01:
        MSG += "Trigger Mode not set";
        break;
    case 0x02:
        MSG += "Couldn't Trigger LiDAR";
        break;
    case 0x03:
        MSG += "Couldn't get LiDAR Data";
        break;
    default:
        MSG += "UNKOWN ERROR";
        break;
    }

    return MSG;
}

String getStepper_ERRORS(int module_code, int error_code)
{
    String MSG = ERROR_MSG_START;

    switch (module_code)
    {
    case LEFT_STEPPER:
        MSG += "LEFT STEPPER --- ";
        break;
    case RIGHT_STEPPER:
        MSG += "RIGHT STEPPER --- ";
        break;

    default:
        MSG += (String)module_code + " --- ";
        break;
    }
    if (error_code < 16)
    {
        MSG += "ERROR CODE: 0x0" + String(error_code, HEX);
        MSG += " --- ";
    }
    else
    {
        MSG += "ERROR CODE: 0x" + String(error_code, HEX);
        MSG += " --- ";
    }

    switch (error_code)
    {
    case 0x00:
        MSG += "INVALID DIRECTION, COULD NOT UPDATE DIR PIN";
        break;

    case 0x01:
        MSG += "Could not find valid stepper direction";
        break;
    case 0x02:
        MSG += "Couldn't Trigger LiDAR";
        break;
    case 0x03:
        MSG += "Couldn't get LiDAR Data";
        break;
    default:
        MSG += "UNKOWN ERROR";
        break;
    }

    return MSG;
}

String getEncoder_ERRORS(int module_code, int error_code)
{
    String MSG = ERROR_MSG_START;

    switch (module_code)
    {
    case LEFT_ENCODER:
        MSG += "LEFT ENCODER --- ";
        break;
    case RIGHT_ENCODER:
        MSG += "RIGHT ENCODER --- ";
        break;
    case ENCODERS:
        MSG += "ENCODERS --- ";
        break;

    default:
        MSG += module_code + " --- ";
        break;
    }
    if (error_code < 16)
    {
        MSG += "ERROR CODE: 0x0" + String(error_code, HEX);
        MSG += " --- ";
    }
    else
    {
        MSG += "ERROR CODE: 0x" + String(error_code, HEX);
        MSG += " --- ";
    }

    switch (error_code)
    {
    case 0x00:
        MSG += "MAGNET NOT DETECTED";
        break;

    case 0x02:
        MSG += "NO ENCODER SELECTED FOR ANGLE READ";
        break;
    default:
        MSG += "UNKOWN ERROR";
        break;
    }

    return MSG;
}

String getMovement_ERRORS(int module_code, int error_code)
{
    String MSG = ERROR_MSG_START;

    switch (module_code)
    {
    case MOVEMENT:
        MSG += "MOVEMENT --- ";
        break;

    default:
        MSG += module_code + " --- ";
        break;
    }
    if (error_code < 16)
    {
        MSG += "ERROR CODE: 0x0" + String(error_code, HEX);
        MSG += " --- ";
    }
    else
    {
        MSG += "ERROR CODE: 0x" + String(error_code, HEX);
        MSG += " --- ";
    }

    switch (error_code)
    {
    case 0x00:
        MSG += "MOVE QUEUE IS FULL";
        break;

    case 0x01:
        MSG += "MOVEMENT STATE ENCOUNTERED ERROR";
        break;

    case 0x02:
        MSG += "X is positive but Y is not matched";
        break;

    case 0x03:
        MSG += "X is negative but Y is not matched";
        break;

    case 0x04:
        MSG += "NO SUITABLE A_pos FOUND";
        break;

    case 0x05:
        MSG += "a_pos OUT OF BOUNDS, ABOVE 2PI";
        break;

    case 0x06:
        MSG += "a_pos OUT OF BOUNDS, below 0";
        break;

    case 0x07:
        MSG += "NO DELTA Y FOUND";
        break;

    case 0x08:
        MSG += "NO DELTA X FOUND";
        break;
    default:
        MSG += "UNKOWN ERROR";
        break;
    }

    return MSG;
}

String getIR_ERRORS(int module_code, int error_code)
{
    String MSG = ERROR_MSG_START;

    switch (module_code)
    {
    case IR1:
        MSG += "IR1 --- ";
        break;

    case IR2:
        MSG += "IR2 --- ";
        break;

    case IR3:
        MSG += "IR3 --- ";
        break;

    case IR4:
        MSG += "IR4 --- ";
        break;

    default:
        MSG += module_code + " --- ";
        break;
    }
    if (error_code < 16)
    {
        MSG += "ERROR CODE: 0x0" + String(error_code, HEX);
        MSG += " --- ";
    }
    else
    {
        MSG += "ERROR CODE: 0x" + String(error_code, HEX);
        MSG += " --- ";
    }

    switch (error_code)
    {
    case 0x00:
        MSG += "MOVE QUEUE IS FULL";
        break;

    default:
        MSG += "UNKOWN ERROR";
        break;
    }

    return MSG;
}

String getDC_ERRORS(int module_code, int error_code)
{
    String MSG = ERROR_MSG_START;

    switch (module_code)
    {
    case DC_MOTORS:
        MSG += "DC BOTH --- ";
        break;

    case L_DC_MOTOR:
        MSG += "L DC --- ";
        break;

    case R_DC_MOTOR:
        MSG += "R DC --- ";
        break;

    default:
        MSG += module_code + " --- ";
        break;
    }
    if (error_code < 16)
    {
        MSG += "ERROR CODE: 0x0" + String(error_code, HEX);
        MSG += " --- ";
    }
    else
    {
        MSG += "ERROR CODE: 0x" + String(error_code, HEX);
        MSG += " --- ";
    }

    switch (error_code)
    {
    case 0x00:
        MSG += "NO MOTOR SELECT TO RETURN STATUS";
        break;

    default:
        MSG += "UNKOWN ERROR";
        break;
    }

    return MSG;
}