#include <PID_v1.h>
#include <motor_control.h>
#include <MPU6050.h>
#include <Servo.h>
#include <SODAR.h>
#include <State.h>

 

#define SERVO_PIN 9
#define TRIG_PIN 10
#define ECHO_PIN 2
#define ECHO_INT 0
#define CENTER 90
#define RESOLUTION 45
#define UPPER_RANGE 180
#define LOWER_RANGE 0

 

SODAR sodar(TRIG_PIN, ECHO_PIN, ECHO_INT, SERVO_PIN, CENTER, RESOLUTION, UPPER_RANGE, LOWER_RANGE); 

 

HC_SR04 sensor(TRIG_PIN, ECHO_PIN, ECHO_INT);
MPU6050 IMU(SDA, SCL);
Servo actuator;
State control_state;

 

//Global Variable
double thresh = 20;
unsigned long prev_time = millis();
unsigned long delay_time = 1000;

 

double setpoint = 0;
double input;
double output;
double kp = 2;
double ki = 0;
double kd = 0.; 
PID controller(&input, &output, &setpoint, kp, ki, kd, REVERSE);

 

void setup() {
  // put your setup code here, to run once:
  motor_setup();

 

  //Make servo pin output
  pinMode(SERVO_PIN, OUTPUT);
  
  //Attaches Servo and points servo forward 
  actuator.attach(SERVO_PIN);
  actuator.write(90);
 
 
  //initializes Ultrasonic sensor for distance
  sensor.begin();
  sensor.start();

 

  //initializes MPU6050 
  IMU.initialize();
  IMU.update();

 

  //initializes State
  control_state.setLinearState(150); //Sets linear state to drive straight
  control_state.setRotationState(0); //Sets rotation state to drive straight

 

  //Estimated Response time for Servo
  sodar.estimated_servo_response_time = 500;

 

  //creates arrays and starts scanning process
  sodar.initialize_arrays();
  sodar.init();

 

  //Initialize PID controller hyper parameters
  controller.SetOutputLimits(-130, 130);
  controller.SetSampleTime(30);
  controller.SetMode(1);

 

  Serial.begin(9600);
}

 

void loop() {
  //updates sodar to move to next position
  sodar.update();
  
  //Updates IMU
  IMU.update();

 

  //updates PID parameters
  setpoint = control_state.getRotationState();
  input = IMU.get_ang_vel('z');

 

   //Algorithm to find farthest distance
   int maxIndex = findMax(sodar.distance, sodar.array_length);
   control_state.setRotationState(sodar.orientation[maxIndex] - 90);
   if(millis() - prev_time > 1000)
   {
    control_state.setRotationState(0);
    prev_time = millis();
   }
   
   if(maxIndex == 0)
   {
     turnLeft90();
   }
   else if(maxIndex == 1)
   {
      turnLeft45();
   }
   else if(maxIndex == 2)
   {
     moveBackwards();
   }
   else if(maxIndex == 3)
   {
    turnRight45();
   }
   else if(maxIndex == 4)
   {
    turnRight90();
   }
   else
   {
      moveForward();
   }
   
      
   //Change the orientation of the car to point towards farthest detected obstacle
  setpoint = control_state.getRotationState();
  input = IMU.get_ang_vel('z');
  controller.Compute();
  raw_motor_control(control_state.getLinearState() + output, control_state.getLinearState() - output);
}

 

int findMax(int arr[], int len){
  int maximum = arr[0];
  int max_index = 0;
  for(int i = 0; i <= len; i++)
  {
    if(arr[i] < maximum)
    {
      maximum = arr[i];
      max_index = i;
    }
  }
  return max_index;
}

 

void turnRight90()
{
  IMU.update();
  control_state.setRotationState(90);
  control_state.setLinearState(0);
  setpoint = control_state.getRotationState();
  input = IMU.get_ang_vel('z');
  controller.Compute();
  raw_motor_control(control_state.getLinearState() + output, control_state.getLinearState() - output);
}

 

void turnLeft90()
{
  IMU.update();
  control_state.setRotationState(-90);
  control_state.setLinearState(0);
  setpoint = control_state.getRotationState();
  input = IMU.get_ang_vel('z');
  controller.Compute();
  raw_motor_control(control_state.getLinearState() + output, control_state.getLinearState() - output);

 

}

 

void turnRight45()
{
  IMU.update();
  control_state.setRotationState(45);
  control_state.setLinearState(0);
}

 

void turnLeft45()
{
  IMU.update();
  control_state.setRotationState(-90);
  control_state.setLinearState(0);
}

 

void moveBackwards()
{
  IMU.update();
  control_state.setRotationState(0);
  control_state.setLinearState(-155);
  
}

 

void moveForward()
{
  IMU.update();
  control_state.setRotationState(0);
  control_state.setLinearState(155);
}
