#ifndef sparkey_gyro_H
#define sparkey_gyro_H

#include "Arduino.h"
#include <Wire.h>
#include <SoftwareSerial.h>
//Armin Ghajari
enum CommType {
    COMM_I2C,
    COMM_HARD_SERIAL,
    COMM_SOFT_SERIAL
};

class TDAxis12 {
private:
    CommType comm_type;
    uint8_t I2C_Address;
    uint8_t callibration_pin;
    HardwareSerial* h_serial_port;
    SoftwareSerial* s_serial_port;
    unsigned long baudRate;

public:
    float x_angle, y_angle, z_angle;


    TDAxis12(uint8_t i2cadd, uint8_t callib_pin) {
        comm_type = COMM_I2C;
        I2C_Address = i2cadd;
        callibration_pin = callib_pin;
        h_serial_port = nullptr;
        s_serial_port = nullptr;
        baudRate = 0;
    }


    TDAxis12(HardwareSerial& port, unsigned long baud, uint8_t callib_pin) {
        comm_type = COMM_HARD_SERIAL;
        h_serial_port = &port;
        s_serial_port = nullptr;
        callibration_pin = callib_pin;
        I2C_Address = 0;
        baudRate = baud;
    }


    TDAxis12(SoftwareSerial& port, unsigned long baud, uint8_t callib_pin) {
        comm_type = COMM_SOFT_SERIAL;
        s_serial_port = &port;
        h_serial_port = nullptr;
        callibration_pin = callib_pin;
        I2C_Address = 0;
        baudRate = baud;
    }

    void init();
    void callibration();
    void read_x_axis();
    void read_y_axis();
    void read_z_axis();
    void read_all_axes();
    void WhatIsBuadrate();
    bool check_connection();
    uint8_t change_baudrate(unsigned long baudrate);

};

#endif // sparkey_gyro_H

