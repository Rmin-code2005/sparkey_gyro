#include "sparkey_gyro_I2C.h"
TDAxis12 gyro(0x10, 8); //I2C  ,  CAL/INT pin 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  gyro.init();
  // start the callibration :
  gyro.callibration();
  // Note : The ST LED will blink! If it doesn't, you definitely have a connection problem.
  Serial.println("If the LED does not blink, the calibration is complete.")
  //end of callibration
}

void loop() {
}
