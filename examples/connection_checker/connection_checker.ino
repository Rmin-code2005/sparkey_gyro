#include "sparkey_gyro.h"
#include <SoftwareSerial.h>
// SoftwareSerial mySerial(50, 51);  // RX, TX
// TDAxis12 gyro(mySerial,9600,2);
TDAxis12 gyro(0x10, 8); //I2C  ,  CAL/INT pin  for I2C
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gyro.init();
  Serial.print(gyro.check_connection());
}

void loop() {

}
