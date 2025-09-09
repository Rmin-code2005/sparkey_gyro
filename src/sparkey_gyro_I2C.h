#ifndef sparkey_gyro_I2C_H
#define sparkey_gyro_I2C_H
#endif // sparkey_gyro_I2Cb_H
#include "Arduino.h"
#include <Wire.h>
class TDAxis12 {
private:
    uint8_t I2C_Address ;
    uint8_t callibration_pin ;
public:
    float x_angle , y_angle , z_angle;
    TDAxis12(uint8_t i2cadd , uint8_t callib_pin){
        I2C_Address = i2cadd;
        callibration_pin = callib_pin;
    }
    void init();
    void callibration();
    void read_x_axis();
    void read_y_axis();
    void read_z_axis();
    void read_all_axes();

};
