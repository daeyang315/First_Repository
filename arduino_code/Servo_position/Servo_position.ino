#include <Servo.h>
#include <MPU6050.h>

MPU6050 IMU(4,5);
Servo servol;

void setup(){
  IMU.initialize();
  IMU.calibrate();
  Serial.begin(9600);
  servol.attach(9);
  servol.write(90);
}

void loop() {
 int accelX = IMU.get_accel('x');
 int servoPos = 90 + ((accelX/1) * 90);
 IMU.update();
 servol.write(servoPos);
}
