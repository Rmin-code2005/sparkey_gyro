#include "sparkey_gyro.h"
void TDAxis12::init() {
    pinMode(callibration_pin, OUTPUT);
    digitalWrite(callibration_pin, 1);

    if (comm_type == COMM_I2C) {
        Wire.begin();
        Wire.beginTransmission(I2C_Address);
        byte error = Wire.endTransmission();

        if (error == 0) {
            Serial.println("TDAxis12 connected successfully.");
        } else {
            Serial.print("Error connecting to TDAxis12. Error code: ");
            Serial.println(error);
            while (true) {
                Serial.println("Connection failed! Check wiring and power.");
                delay(1000);
            }
        }
    }
    else if (comm_type == COMM_HARD_SERIAL && h_serial_port) {
        h_serial_port->begin(baudRate);
    }
    else if (comm_type == COMM_SOFT_SERIAL && s_serial_port) {
        s_serial_port->begin(baudRate);
    }
}

void TDAxis12::callibration(){
    digitalWrite(callibration_pin , 0);
    delay(2000);
    digitalWrite(callibration_pin , 1);
}
void TDAxis12::read_x_axis() {
  if (comm_type == COMM_I2C) {

    Wire.beginTransmission(I2C_Address);
    Wire.write(0x08);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_Address, (uint8_t)2);

    if (Wire.available() == 2) {
      uint8_t high = Wire.read();
      uint8_t low = Wire.read();

      int16_t raw = (high << 8) | low;
      x_angle = raw / 100.0;
    }
  } else if (comm_type == COMM_HARD_SERIAL && h_serial_port) {
    if (h_serial_port->available()) {
      String rawData = h_serial_port->readStringUntil(';');
      rawData.trim();

      if (rawData.startsWith("#")) {
        rawData.remove(0, 1);
        int END = 0;
        String out;
        END = rawData.indexOf(',');
        rawData = rawData.substring(END + 1, rawData.length());
        END = rawData.indexOf(',');
        rawData = rawData.substring(END + 1, rawData.length());
        out = rawData.substring(0, rawData.length());
        x_angle = out.toFloat();
      }
    }
  } else if (comm_type == COMM_SOFT_SERIAL && s_serial_port) {
    if (s_serial_port->available()) {
      String rawData = s_serial_port->readStringUntil(';');
      rawData.trim();

      if (rawData.startsWith("#")) {
        rawData.remove(0, 1);
        int END = 0;
        String out;
        END = rawData.indexOf(',');
        rawData = rawData.substring(END + 1, rawData.length());
        END = rawData.indexOf(',');
        rawData = rawData.substring(END + 1, rawData.length());
        out = rawData.substring(0, rawData.length());
        x_angle = out.toFloat();
      }
    }
  }
  delay(10);
}
void TDAxis12::read_y_axis() {
  if (comm_type == COMM_I2C) {
    Wire.beginTransmission(I2C_Address);
    Wire.write(0x06);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_Address, (uint8_t)2);

    if (Wire.available() == 2) {
      uint8_t high = Wire.read();
      uint8_t low = Wire.read();

      int16_t raw = (high << 8) | low;
      y_angle = raw / 100.0;
    }
  } else if (comm_type == COMM_HARD_SERIAL && h_serial_port) {
    if (h_serial_port->available()) {
      String rawData = h_serial_port->readStringUntil(';');
      rawData.trim();
      if (rawData.startsWith("#")) {
        rawData.remove(0, 1);
        int END = 0;
        String out;
        float yaw;
        END = rawData.indexOf(',');
        rawData = rawData.substring(END + 1, rawData.length());
        END = rawData.indexOf(',');
        out = rawData.substring(0, END);
        y_angle=out.toFloat();
      }
    }
  } else if (comm_type == COMM_SOFT_SERIAL && s_serial_port) {
    if (s_serial_port->available()) {
      String rawData = s_serial_port->readStringUntil(';');
      rawData.trim();

      if (rawData.startsWith("#")) {
        rawData.remove(0, 1);
        int END = 0;
        String out;
        float yaw;
        END = rawData.indexOf(',');
        rawData = rawData.substring(END + 1, rawData.length());
        END = rawData.indexOf(',');
        out = rawData.substring(0, END);
        y_angle=out.toFloat();
      }
    }
  }
  delay(10);
}
void TDAxis12::read_z_axis() {
  if (comm_type == COMM_I2C) {
    Wire.beginTransmission(I2C_Address);
    Wire.write(0x04);
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_Address, (uint8_t)2);

    if (Wire.available() == 2) {
      uint8_t high = Wire.read();
      uint8_t low = Wire.read();

      int16_t raw = (high << 8) | low;
      z_angle = raw / 100.0;
    }
    delay(10);
  } else if (comm_type == COMM_HARD_SERIAL && h_serial_port){
    if (h_serial_port->available()) {
      String rawData = h_serial_port->readStringUntil(';');
      rawData.trim();

      if (rawData.startsWith("#")) {
        rawData.remove(0, 1);
        int END = 0;
        String out;
        END = rawData.indexOf(',');
        out = rawData.substring(0, END);
        z_angle = out.toFloat();
      }
    }
  }
  else if (comm_type == COMM_SOFT_SERIAL && s_serial_port){
    if (s_serial_port->available()) {
      String rawData = s_serial_port->readStringUntil(';');
      rawData.trim();

      if (rawData.startsWith("#")) {
        rawData.remove(0, 1);
        int END = 0;
        String out;
        END = rawData.indexOf(',');
        out = rawData.substring(0, END);
        z_angle = out.toFloat();
      }
    }
  }
}
void TDAxis12::read_all_axes(){
    read_x_axis();
    read_y_axis();
    read_z_axis();
}
void TDAxis12::WhatIsBuadrate() {
    if (comm_type != COMM_I2C) {
        Serial.println("Baudrate can only be read in I2C mode!");
        return;
    }
    Wire.beginTransmission(I2C_Address);
    Wire.write(0x02);   // UART_BAUD register
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_Address, (uint8_t)1);

    if (Wire.available()) {
        uint8_t baud_hex = Wire.read();

        Serial.print("UART Baudrate Register (HEX): 0x");
        Serial.println(baud_hex, HEX);

        unsigned long baud_values[] = {
            9600, 14400, 19200, 38400, 56000,
            57600, 115200, 128000, 256000
        };

        if (baud_hex <= 0x08) {
            Serial.print("Current UART Baudrate: ");
            Serial.println(baud_values[baud_hex]);
        } else {
            Serial.println("Unknown baudrate value!");
        }
    }
}

uint8_t TDAxis12::change_baudrate(unsigned long baudrate) {


    unsigned long baud_list[9] = {
        9600, 14400, 19200, 38400, 56000,
        57600, 115200, 128000, 256000
    };

    uint8_t hex_val = 0xFF;

    for (uint8_t i = 0; i < 9; i++) {
        if (baud_list[i] == baudrate) {
            hex_val = i;
            break;
        }
    }


    if (hex_val == 0xFF) {
        Serial.println("ERROR: Invalid baudrate value!");
        return 0;
    }

    if (comm_type != COMM_I2C) {
        Serial.println("ERROR: Baudrate config only possible in I2C mode!");
        return 0;
    }


    Wire.beginTransmission(I2C_Address);
    Wire.write(0x02);
    Wire.write(hex_val);
    if (Wire.endTransmission() != 0) {
        Serial.println("ERROR: I2C write failed!");
        return 0;
    }


    baudRate = baud_list[hex_val];


    if (comm_type == COMM_HARD_SERIAL && h_serial_port) {
        h_serial_port->begin(baudRate);
    }
    else if (comm_type == COMM_SOFT_SERIAL && s_serial_port) {
        s_serial_port->begin(baudRate);
    }

    Serial.print("Baudrate changed successfully to: ");
    Serial.println(baudRate);

    return 1;
}


bool TDAxis12::check_connection() {


    if (comm_type == COMM_I2C) {
        Wire.beginTransmission(I2C_Address);
        Wire.write(0x00);
        if (Wire.endTransmission(false) != 0)
            return 0;

        Wire.requestFrom(I2C_Address, (uint8_t)1);
        if (!Wire.available()) return 0;

        uint8_t who = Wire.read();

        if (who == 0x12) {
            return 1;
        } else {
            return 0;
        }
    }


    if (comm_type == COMM_HARD_SERIAL && h_serial_port) {
        if (h_serial_port->available()) {
            String raw = h_serial_port->readStringUntil(';');
            raw.trim();
            if (raw.startsWith("#"))
                return 1;
        }
        return 0;
    }

    if (comm_type == COMM_SOFT_SERIAL && s_serial_port) {
        if (s_serial_port->available()) {
            String raw = s_serial_port->readStringUntil(';');
            raw.trim();
            if (raw.startsWith("#"))
                return 1;
        }
        return 0;
    }

    return 0;
}
void TDAxis12::read_accelerometer() {

    if (comm_type != COMM_I2C) {
        // Serial mode does not provide accel data
        acc_x = acc_y = acc_z = 0;
        return;
    }

    int16_t raw;

    // X ACC
    Wire.beginTransmission(I2C_Address);
    Wire.write(0x14); // X_ACC_H
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_Address, (uint8_t)2);
    raw = (Wire.read() << 8) | Wire.read();
    acc_x = raw * 9.81 / 16384.0;   // typical ±2g scaling

    // Y ACC
    Wire.beginTransmission(I2C_Address);
    Wire.write(0x12); // Y_ACC_H
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_Address, (uint8_t)2);
    raw = (Wire.read() << 8) | Wire.read();
    acc_y = raw * 9.81 / 16384.0;

    // Z ACC
    Wire.beginTransmission(I2C_Address);
    Wire.write(0x10); // Z_ACC_H
    Wire.endTransmission(false);
    Wire.requestFrom(I2C_Address, (uint8_t)2);
    raw = (Wire.read() << 8) | Wire.read();
    acc_z = raw * 9.81 / 16384.0;
}

void TDAxis12::compute_linear_acceleration() {

    // Convert angles to radians
    float roll  = x_angle * DEG_TO_RAD;
    float pitch = y_angle * DEG_TO_RAD;

    // Gravity components in sensor frame
    float g_x = -9.81 * sin(pitch);
    float g_y =  9.81 * sin(roll) * cos(pitch);
    float g_z =  9.81 * cos(roll) * cos(pitch);

    // Subtract gravity
    lin_acc_x = acc_x - g_x;
    lin_acc_y = acc_y - g_y;
    lin_acc_z = acc_z - g_z;
}
