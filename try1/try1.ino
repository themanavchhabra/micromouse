//#include <util/atomic.h>
//#include <PIDController.h>

#define ENCA1 2 // YELLOW
#define ENCB1 3 // WHITE
#define ENCA2 14 // YELLOW
#define ENCB2 15
#define PWMA 5
#define PWMB 4
#define IN2 6
#define IN1 7
#define IR1 8
#define IR2 9
#define MOTOR1A 10
#define MOTOR1B 11
#define MOTOR2A 12
#define MOTOR2B 13


int encoder_pos = 0;
int posi = 0;


void setup() {
  // put your setup code here, to run onc

pinMode(IR1, INPUT);
pinMode(IR2, INPUT);
pinMode(ENCA1, INPUT);
pinMode(ENCB1, INPUT);
attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder,RISING);
pinMode(ENCA2, INPUT);
pinMode(ENCB2, INPUT);
attachInterrupt(digitalPinToInterrupt(ENCA2),readEncoder,RISING);
pinMode(PWMA, OUTPUT);
pinMode(PWMB, OUTPUT);
pinMode(MOTOR1A, OUTPUT);
pinMode(MOTOR1B, OUTPUT);
pinMode(MOTOR2A, OUTPUT);
pinMode(MOTOR2B, OUTPUT);
}
void loop(){
  path();
}
void path() {
  // put your main code here, to run repeatedly:
 while(1){
  int a = analogRead(IR1);
  int b = analogRead(IR2);
  if( a+b ==5 ){
    gostraight(a,b);
    }
  else{
    turn(a,b);
  }  

 }

}

void gostraight(int a, int b){
  if( a == b){
    setmotors(150);
  }
  else{
    pid(a,b);
  }
}

void setmotors(int pwm){
  digitalWrite(MOTOR1B, HIGH);
  digitalWrite(MOTOR1A, LOW);
  digitalWrite(MOTOR2B, HIGH);
  digitalWrite(MOTOR2A, LOW);
  analogWrite(PWMA, pwm);
  analogWrite(PWMB, pwm);
}

void pid(int a, int b){
  float kp, kd;
  float error ;
  if(a>b){
    error = ((a+b/2)-a);
    int pwmex = kp*error + kd*error;
    setmotorsa(150, pwmex);
  }
  else{
    error = ((a+b/2)-b);
    int pwmex = kp*error + kd*error;
    setmotorsb(150, pwmex);
  }
  
  
}

void setmotorsa(int n, int pwmex){
  digitalWrite(MOTOR2B, HIGH);
  digitalWrite(MOTOR2A, LOW);
  analogWrite(PWMB, n+pwmex);
}

void setmotorsb(int n, int pwmex){
  digitalWrite(MOTOR1B, HIGH);
  digitalWrite(MOTOR1A, LOW);
  analogWrite(PWMA, n+pwmex);
}

void readEncoder(){
  int e = digitalRead(ENCB1);
  if(e > 0){
    posi++;
  }
  else{
    posi--;
  }
}

void turn(int a, int b){
  if(a>b){
  }
}
