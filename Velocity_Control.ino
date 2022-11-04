#include <util/atomic.h>

// Pins
#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN1 6
#define IN2 7

long prevT = 0;
int posPrev = 0;
volatile int pos_i = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float uprev=0;
float eprev=0;
float uprev2=0;
float eprev2=0;



float eintegral = 0;

void setup() {
  Serial.begin(2000000);

  pinMode(ENCA,INPUT);
  pinMode(ENCB,INPUT);
  pinMode(PWM,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);

  attachInterrupt(digitalPinToInterrupt(ENCA),
                  readEncoder,RISING);
}

void loop() {

  
  int pos = 0;
  float velocity2 = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    pos = pos_i;
    velocity2 = velocity_i;
  }
//method 1
  long currT = micros();
  float deltaT = ((float) (currT-prevT))/1.0e6;
  float velocity1 = (pos - posPrev)/deltaT;
  posPrev = pos;
  prevT = currT;

  float v1 = velocity1/10;
  float v2 = velocity2/10;

// TARGET 
  float vt = 30*25;

  float kp = 1.5;
  float ki = 5;

  
  float e = -vt+v2;
  eintegral = eintegral + e*deltaT;
  
// without controller
//   float u = e;
   // lead
//  float u = 0.8215*uprev+2.54*(e-0.9226*eprev);
// leag lad
  float u=1.8213*uprev-0.8213*uprev2+2.54*(e-1.9064*eprev+0.9077*eprev2);
// PID
// float u = kp*e + ki*eintegral;


  int dir = 1;
  if (u<0){
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if(pwr > 255){
    pwr = 255;
  }
  
  setMotor(dir,pwr,PWM,IN1,IN2);

  Serial.print(vt/25);
  Serial.print(" ,");
  Serial.print(v2/25);
  Serial.println();
  
  delay(1);

    uprev2=uprev;
    eprev2=eprev;
    uprev=u;
    eprev=e;
    

}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal); // Motor speed
  if(dir == 1){ 
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    // Turn the other way
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    // Or dont turn
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);    
  }
}

void readEncoder(){
  // Read encoder B when ENCA rises
  int b = digitalRead(ENCB);
  int increment = 0;
  if(b>0){
    increment = 1;
  }
  else{
    increment = -1;
  }
  pos_i = pos_i + increment;
//method 2
  long currT = micros();
  float deltaT = ((float) (currT - prevT_i))/1.0e6;
  velocity_i = increment/deltaT;
  prevT_i = currT;
}
