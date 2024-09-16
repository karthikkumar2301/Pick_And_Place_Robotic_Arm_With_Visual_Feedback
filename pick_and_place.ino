#include <math.h>
#include <Servo.h>
#include <Arduino.h>
int base=6;
int elbow=10;
int shoulder=7;
int wrist=11;
int end_effector=12;

float x;
float y;
float z=0;
float max_limit=26;
float min_limit=16;
float a1=8.3;
float a2=20;
float y1=2.55;
float y2=2.4;
float theta_shoulder;
float theta_base;
float theta_elbow;
float base_pwm;
float elbow_pwm;
float shoulder_pwm;
float base_angle;
float shoulder_angle;
float elbow_angle;
float beta;
float end_eff_pwm;
float wrist_pwm;
float dum;
Servo motor;
float place;
unsigned long startTime;
void pwm_duration(float pwm,int part);
void setup() {
   Serial.begin(9600);
   Serial.setTimeout(1000);
   pinMode(base, OUTPUT);
   pinMode(elbow, OUTPUT);
   pinMode(shoulder, OUTPUT);
   pinMode(wrist,OUTPUT);
   pinMode(end_effector,OUTPUT);
   motor.attach(12);
}
void loop() {

if(Serial.available() > 0){

String data_x = Serial.readStringUntil('\n');

  y = data_x.toFloat();


  delay(20);

  String data_y = Serial.readStringUntil('\n');

  x=data_y.toFloat();

  delay(100);

  String data_place = Serial.readStringUntil('\n');
 
  place=data_place.toFloat();




dum=sqrt(y*y+(x+y1+y2)*(x+y1+y2))-1.8;
theta_base=90-abs(atan((x+4.2)/y)*180/PI);

theta_elbow=acos((dum*dum+z*z-a1*a1-a2*a2)/(2*a1*a2))*180/PI;

if(theta_elbow<0){

  theta_elbow=180+theta_elbow;

}
beta=atan((a2*sin(theta_elbow*PI/180))/(a1+a2*cos(theta_elbow*PI/180)))*180/PI;


if(beta<0){

  beta=180+beta;

  }

theta_shoulder=beta+atan(z/dum)*180/PI;


if(y==0){
  base_angle=90;
}
else{
 base_angle= 90+(y/abs(y))*theta_base;
}


 shoulder_angle=90-theta_shoulder;



 elbow_angle=114.75-theta_elbow;



base_pwm=(1.6*base_angle/180 + 0.6)*1000 -80;

wrist_pwm=470;
end_eff_pwm=600;
shoulder_pwm=(1.6*shoulder_angle/180 + 0.6)*1000;

elbow_pwm=(1.6*elbow_angle/180 + 0.6)*1000;


motor.write(80);
delay(100);
startTime = millis();
while(millis()-startTime<1400){
pwm_duration(base_pwm,base);
}

startTime = millis();
while(millis()-startTime<500){
pwm_duration(wrist_pwm,wrist);
}
motor.write(150);
delay(200);
startTime = millis();
while(millis()-startTime<1200){
pwm_duration(elbow_pwm,elbow);
}
delay(100);
startTime = millis();
while(millis()-startTime<1200){
pwm_duration(shoulder_pwm,shoulder);
}

motor.write(65);
delay(400);
shoulder_pwm=500;
startTime = millis();
while(millis()-startTime<800){
pwm_duration(shoulder_pwm,shoulder);
}
delay(200);
elbow_pwm=800;
startTime = millis();
while(millis()-startTime<800){
pwm_duration(elbow_pwm,elbow);
}
delay(200);
if(place==1){
base_pwm=2200;
startTime = millis();
while(millis()-startTime<1000){
pwm_duration(base_pwm,base);
}
motor.write(155);
}
if(place==2){
base_pwm=500;
startTime = millis();
while(millis()-startTime<1000){
pwm_duration(base_pwm,base);
}
motor.write(155);
}
Serial.println("Ready\n");

}

}

void pwm_duration(float pwm,int part){
unsigned long targetDelay = pwm;
digitalWrite(part, HIGH); // Turn on the LED (high voltage)
unsigned long start = micros();

while (micros() - start < targetDelay) {
  
}


digitalWrite(part, LOW); // Turn off the LED (low voltage)

 unsigned long startTime_1 = micros();

while (micros() - startTime_1 < 20000-targetDelay ) {

}
}

