#include "sparkey_gyro.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(50, 51);  // RX, TX
TDAxis12 gyro(mySerial,9600,2);
// TDAxis12 gyro(0x10, 8); //I2C  ,  CAL/INT pin  for I2C
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gyro.init();
}

void loop() {
  gyro.read_all_axes();
  Serial.print("x:\t");
  Serial.print(gyro.x_angle);
  Serial.print("\ty:\t");
  Serial.print(gyro.y_angle);
  Serial.print("z:\t");
  Serial.println(gyro.z_angle);
  //if you want to read hust one axis , you can use "read_x_axis or read_y_axis or read_z_axis " instead of "read_all_axes" .
}
