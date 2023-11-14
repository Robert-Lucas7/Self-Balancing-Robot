#include "MPU9250.h"
//#include <SparkFun_TB6612.h>


// Pins for all inputs, keep in mind the PWM defines must be on PWM pins
#define AIN1 4
#define AIN2 3
#define PWMA 2
#define STBY 5
#define BIN1 6
#define BIN2 7
#define PWMB 8

MPU9250 mpu;

const int offsetA = 1;
const int offsetB = -1;
int kp;
int ki;
int kd;
int setpoint;
unsigned long prev_time;
float prev_error;
int dt;
float error;
float error_derivative;
float error_integral;
float correction;
int action;

//Motor motor1 = Motor( AIN1,  AIN2,  PWMA,  offsetA,  STBY);
//Motor motor2 = Motor( BIN1,  BIN2,  PWMB,  offsetB,  STBY);

void setup() {
  // put your setup code here, to run once:
  kp = 5;
  ki=2;
  kd = 0.5;
  setpoint = 0;
  prev_time = millis();
  prev_error = 0;
  dt=0;
  error = 0;
  //error_derivative = 0;
  error_integral = 0;
  //correction = 0;
  //action = 0;
  pinMode(PWMA,OUTPUT);
  pinMode(AIN1,OUTPUT);
  pinMode(AIN2,OUTPUT);
  pinMode(PWMB,OUTPUT);
  pinMode(BIN1,OUTPUT);
  pinMode(BIN2,OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  delay(2000);
  
  mpu.setup(0x68);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(mpu.update()){
    dt = (millis() - prev_time)/1000; //number of milliseconds since the board was powered up
    error = setpoint - mpu.getPitch(); //This measurement is in degrees
    Serial.println(mpu.getPitch());
    Serial.print(millis() - prev_time);
    error_derivative = (error - prev_error) / dt;
    error_integral += error * dt;

    correction = kp * error + ki * error + kd * error;
    prev_error = error;

    action = 1.0/(1.0 + exp(-correction));
    action = round(action); //action is either 0 or 1

    if(action == 1){
      digitalWrite(AIN1,HIGH); //Motor A Rotate Clockwise
      digitalWrite(AIN2,LOW);
      digitalWrite(BIN1,HIGH); //Motor B Rotate Clockwise
      digitalWrite(BIN2,LOW);
      
    } else{
      digitalWrite(AIN1,LOW); //Motor A Rotate Clockwise
      digitalWrite(AIN2,HIGH);
      digitalWrite(BIN1,LOW); //Motor B Rotate Clockwise
      digitalWrite(BIN2,HIGH);
    }
    analogWrite(PWMA,255); //Speed control of Motor A
    analogWrite(PWMB,255); //Speed control of Motor B
    
      
      prev_time = millis();
      
    }
    else{
      Serial.println(mpu.getPitch());
    
    }
    
}
