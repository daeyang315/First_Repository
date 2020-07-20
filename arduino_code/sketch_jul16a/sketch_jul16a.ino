#include <HC_SR04.h>
#include <PID_v1.h>
#include <motor_control.h>
#include <Servo.h>

#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0



HC_SR04 sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);
Servo actuator;


double setpoint = 5;
double input;
double output;
double kp = 5;
double ki = 0;
double kd = 0;
PID controller(&input, &output, &setpoint, kp, ki, kd, REVERSE);

void setup() {
  // put your setup code here, to run once:

//setup motors
  motor_setup();
 
//setup servo and point forward
   actuator.attach(9);
  actuator.write(90);
 
//initalize distance sensor
  sensor.begin();
  sensor.start();
 
//set PID parameters
  controller.SetOutputLimits(-255, 255);
  controller.SetSampleTime(2);
  controller.SetMode(1);


  Serial.begin(9600);

}
void loop() {
  // put your main code here, to run repeatedly:
//get distance value from sensor, update input to be distance value
if(sensor.isFinished())
{
  input = sensor.get_ang_vel();
  sensor.start(); //restart sensor
  
}

//compute PID controller once input has been updated
controller.Compute();
//set motor power based on output from PID controller
raw_motor_control (output, output);

Serial.print("output: "); Serial.print(output); Serial.print("distance: "); Serial.println(input);
delay(500);
}
