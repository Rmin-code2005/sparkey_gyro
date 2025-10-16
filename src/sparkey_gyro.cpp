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
