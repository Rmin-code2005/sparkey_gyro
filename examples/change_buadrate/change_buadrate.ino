#include "sparkey_gyro.h"
TDAxis12 gyro(0x10, 8);  //I2C  ,  CAL/INT pin  for I2C
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gyro.init();
  gyro.change_baudrate(115200);  //change buadrate to 115200
  gyro.WhatIsBuadrate();         //print current baudrate
  delay(1000);
  gyro.change_baudrate(9600);  //change buadrate to 9600
  delay(1000);
  gyro.WhatIsBuadrate();  //print current baudrate
}

void loop() {
}
