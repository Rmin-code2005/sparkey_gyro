# sparkey_gyro_i2c

An Arduino library for interfacing with the Sparkey 3-axis gyroscope sensor over the I2C bus.  
This library provides an easy-to-use API to initialize the sensor, calibrate it, and read angular data from the X, Y, and Z axes.

## Features
- Simple initialization with custom I2C address and calibration pin.
- Calibration function for accurate readings.
- Read individual axes (X, Y, Z).
- Read all axes at once.

## Example Usage
```cpp
#include <sparkey_gyro_i2c.h>

TDAxis12 gyro(0x68, 2); // I2C address and calibration pin

void setup() {
  Serial.begin(9600);
  gyro.init();
  gyro.callibration();
}

void loop() {
  gyro.read_all_axes();
  Serial.print("X: "); Serial.print(gyro.x_angle);
  Serial.print(" Y: "); Serial.print(gyro.y_angle);
  Serial.print(" Z: "); Serial.println(gyro.z_angle);
  delay(500);
}
