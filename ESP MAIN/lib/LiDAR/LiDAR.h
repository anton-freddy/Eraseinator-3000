#ifndef LiDAR_H
#define LiDAR_H

#include <ERROR.h>
#include <Arduino.h>
#include <TFLI2C.h>
#include <Wire.h>





class TF_LUNA
{
    private:
        int16_t address;
        uint16_t frame_rate = 0x00;
        int LiDAR_id;
        TFLI2C luna;
        TwoWire* I2C_bus;
    
    public:
        TF_LUNA(int16_t address, uint16_t frame_rate, int id);
        TF_LUNA(int16_t address, uint16_t frame_rate, int id, TwoWire &I2C);
        bool begin();
        int16_t getDistance();
};

#endif  // LiDAR_H