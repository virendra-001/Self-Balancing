#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include "Wire.h"       
#include "I2Cdev.h"     
#include "MPU6050.h" 
float pitch;
float pitchold=0;

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

void isr(){
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  
}

struct MyData {
  byte X;
  byte Y;
  byte Z;
};
MyData data;
AccelStepper stepper1(1,27,14);
AccelStepper stepper2(1,2,15);

unsigned long previous_time=0;
float kp=2;
float kd=0;
float ki=0;
float setpoint=0;
// int gx_offset_mean;
// int gy_offset_mean;
// int gz_offset_mean;
// int gx_offset_sum=0;
// int gy_offset_sum=0;
// int gz_offset_sum=0;
// int ax_offset_mean;
// int ay_offset_mean;
// int az_offset_mean;
// int ax_offset_sum=0;
// int ay_offset_sum=0;
// int az_offset_sum=0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  stepper1.setMaxSpeed(3000);
  stepper1.setAcceleration(2000);
  stepper2.setMaxSpeed(3000);
  stepper2.setAcceleration(2000);
  pinMode(21,INPUT_PULLUP);
  //pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(21),isr,CHANGE);
  

}
void loop()
{
  unsigned long present_time= millis();
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // gx=gx-gx_offset_mean;
  // gy=gy-gy_offset_mean;
  // gz=gz-gz_offset_mean;
  // ax=ax-ax_offset_mean;
  // ay=ay-ay_offset_mean;
  // az=az-az_offset_mean;
  float pitchmeasured = (atan(-1* ax/sqrt(pow(ay,2)+ pow(az,2)))*180/PI);
  float accel=map(abs(gy),0,32767,400,2000);
  pitch=(pitchold*0.95)+(pitchmeasured*0.05);
  
 if(pitch>25 || pitch<25){
  
  stepper1.setMaxSpeed(3000);
  
  stepper2.setMaxSpeed(3000);
  
 }
  if(accel>2000){
    accel=2000;
  }
  if(accel<400){
    accel=400;
  }

  float error= pitch-setpoint;
  float kp_term=kp*error;
  float kd_term=0;
  float ki_term=0;
  float pid_value=kp_term+kd_term+ki_term;

  if(present_time-previous_time>=500){
  previous_time=millis();
   Serial.print("gy=");
   Serial.println(gy);


   Serial.print("pitch");
   Serial.println(pitch);
   // Serial.println();

    Serial.print("accel=");
   Serial.println(accel);

    Serial.print("error=");
    Serial.println(error);

   Serial.print("pid_value=");
    Serial.println(pid_value);
  
//  Serial.print("pitchmeasured");
//  Serial.print(",");
//  Serial.println(pitchmeasured);
  Serial.print("pitchnew= ");
  Serial.println(pitch);

Serial.println();

// Serial.println(error);
// Serial.println(accel);
// Serial.println("  ");
// Serial.println(pitch);
  }
  // Serial.print("error=");
  // Serial.println(error);
  stepper1.move(-1*pid_value);
  stepper2.move(-1*pid_value);

  stepper1.run();
  stepper2.run();
  pitchold=pitch;
 }