#include <util/atomic.h> // For the ATOMIC_BLOCK macro
 
#define ENCA 2 
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7
 
volatile int posi = 0; 

long prevT = 0;
float eprev = 0;
float eintegral = 0;
float uprev=0;
 
void setup() {
  Serial.begin(9600);
  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA),readEncoder,RISING);
  
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  
  Serial.println("target pos");
}
 
void loop() {
delay(10);
  // set target position


  int target = 100*43.5;
  // ramp target:
//  int target = 100*43.5*millis()/1000;
// sin target:
  //int target = 100*43.5*sin(prevT/1e6);

 
 
  // PID constants
  float kp = 1;
  float kd = 0.05;
  float ki = 0.001;
 
  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;
  
 int pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;
  }
  
  // error
  int e = -pos + target;
 
  // derivative
  float dedt = (e-eprev)/(deltaT);
 
  // integral
  eintegral = eintegral + e*deltaT;
 
  // control signal
//   float u=e;
 
// lead compensator
     float u = 0.5781*uprev+1.5*(e-0.7902*eprev);
 
// PID controller
//  float u = kp*e + kd*dedt + ki*eintegral;
 
 
  // motor power
  float pwr = fabs(u);
  if( pwr > 255 ){
    pwr = 255;
  }
 
  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }
  if(fabs(e)<60){
      dir = 0;
    }
  
  // signal the motor
  setMotor(dir,pwr,PWM,IN1,IN2);
 
 
  // store previous error
  eprev = e;
  uprev=u;
 
  Serial.print(target/43.5);
  Serial.print(" ,");
 
  Serial.print(pos/43.5);
  Serial.println();
}
 
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
 
void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    posi++;
  }
  else{
    posi--;
  }
}
