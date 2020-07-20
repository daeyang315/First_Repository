#include <motor_control.h>
#include <PID_v1.h>
#include <MPU6050.h>
#define MPU6050_ADDRESS 0x68
//Global Variables
#define SDA 4
#define SDA 5

MPU6050 sensor(SDA,SCL);


double base_motor_speed = 140;
double setpoint = 0;
double input;
double output;
double kp = 1;
double ki = 0;
double kd = 0;

PID controller(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {

 motor_setup();
 
 Serial.begin(9600);
 sensor.initialize();
 

 
 controller.SetOutputLimits(-115,115);
 controller.SetSampleTime(50);
 controller.SetMode(1);
}

void loop() {
  sensor.update();
  sensor.get_ang_vel('z');
  input = sensor.get_ang_vel('z');
  Serial.println("input: "); Serial.print(input );
  sensor.update();

  controller.Compute();
  raw_motor_control(base_motor_speed - output, base_motor_speed + output);
  Serial.print ("output: "); Serial.print(output );
  
  
  
  
}
