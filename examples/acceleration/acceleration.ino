#include "sparkey_gyro.h"
#include <SoftwareSerial.h>
// SoftwareSerial mySerial(50, 51);  // RX, TX
// TDAxis12 imu(mySerial, 9600, 2);
TDAxis12 imu(0x10, 2); //I2C  ,  CAL/INT pin  for I2C
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  imu.init();
}

void loop() {
  imu.read_all_axes();       // angles
  imu.read_accelerometer();  // raw accel (I2C only)
  imu.compute_linear_acceleration();

  // Serial.println(imu.lin_acc_x);
  // Serial.println(imu.lin_acc_y);
  Serial.println(imu.lin_acc_z);

  //if you want to read hust one axis , you can use "read_x_axis or read_y_axis or read_z_axis " instead of "read_all_axes" .
}
