#include "sparkey_gyro_I2C.h"

void TDAxis12::init(){
    digitalWrite(callibration_pin , 1);
    Wire.begin();
    pinMode(callibration_pin , OUTPUT);
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
void TDAxis12::callibration(){
    digitalWrite(callibration_pin , 0);
    delay(2000);
    digitalWrite(callibration_pin , 1);
}
void TDAxis12::read_x_axis(){
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
  delay(10);
}
void TDAxis12::read_y_axis(){
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
  delay(10);
}
void TDAxis12::read_z_axis(){
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
}
void TDAxis12::read_all_axes(){
    read_x_axis();
    read_y_axis();
    read_z_axis();
}
