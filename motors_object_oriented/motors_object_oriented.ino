#include<SharpIR.h>
#include<util/atomic.h> //atomic block macro

#define irLeftPin A1
#define irMidPin 0
#define irRightPin A2

int baseSpeed = 70; //avg speed
int maxSpeed = 100;
int lastError = 0;

float irArr[] = {0,0,0};
int irArrBinary[] = {0,0,0};

volatile int posiA, posiB; //volatile variables for encoder readings
int posA, posB; 

class Motors{
  public :
  int enca, encb, m1, m2, pwm;

  Motors(int ENCA, int ENCB, int M1, int M2, int PWM){
    enca = ENCA;
    encb = ENCB;
    m1 = M1;
    m2 = M2;
    pwm = PWM;
    
    pinMode(enca, INPUT);
    pinMode(encb, INPUT);
    pinMode(m1, OUTPUT);
    pinMode(m2, OUTPUT);
    pinMode(pwm, OUTPUT);

    }

  void setMotor(int dira, int dirb, int pwmVal){
      Serial.println(pwmVal);
      digitalWrite(m1, dira);
      digitalWrite(m2, dirb);
      digitalWrite(pwm, pwmVal);
      }
};

Motors motorLeft(3,11,5,6,9);
Motors motorRight(2,12,4,8,10);

SharpIR irLeft(irLeftPin, 1080);
SharpIR irMid(irMidPin, 1080);
SharpIR irRight(irRightPin,1080);

void readEncoderMotorLeft(){
  int b = digitalRead(motorLeft.encb);
  if(b > 0){
    posiA++;
  }
  else{
    posiA--;
  }

//  Serial.print("posiA is ");
//  Serial.println(posiA);
}


void readEncoderMotorRight(){
  int b = digitalRead(motorRight.encb);
  if(b > 0){
    posiB++;
  }
  else{
    posiB--;
  }

//  Serial.print("posiB is ");
//  Serial.println(posiB);
}

void setup() {
  // put your setup code here, to run once:
  attachInterrupt(digitalPinToInterrupt(motorLeft.enca),readEncoderMotorLeft,RISING);
  attachInterrupt(digitalPinToInterrupt(motorRight.enca),readEncoderMotorRight,RISING);

  Serial.begin(9600);
}

void loop() {
  goStraight();

  int posA = 0;
  int posB = 0;

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    posA = posiA;
    posB =  posiB;
    }

}

float  readIR(){
    irArr[0] = irLeft.distance();
    Serial.print("ir left ");
    Serial.println(irArr[0]);
//    irArr[1] = irMid.distance();
    irArr[2] = irRight.distance();
    Serial.print("ir right ");
    Serial.println(irArr[2]);

    if(irArr[0]>0 && irArr[0]<10){irArrBinary[0] == 1;}  //basic ir array to check for walls, 1 if wall, 0 if not
    if(irArr[1]>0 && irArr[1]<10){irArrBinary[1] == 1;}
    if(irArr[2]>0 && irArr[2]<10){irArrBinary[2] == 1;}

    float err = irArr[0] - irArr[2];

    return err;
  }

void goStraight(){
  float kp = 0.75;
  float kd = 1;

  readIR();

//  Serial.print("average encoder readings ");
//  Serial.println((posiA+posiB)/2);

  while(1){
    int error = readIR();

    int motorSpeed = kp*error + kd*(error-lastError);

    int leftMotorSpeed = baseSpeed - motorSpeed;
    int rightMotorSpeed = baseSpeed + motorSpeed;

    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;

//    Serial.print("Left pwm ");
//    Serial.println(leftMotorSpeed);
//    Serial.print("Right pwm");
//    Serial.println(rightMotorSpeed);

    motorLeft.setMotor(HIGH, LOW, leftMotorSpeed);
    motorRight.setMotor(HIGH, LOW, rightMotorSpeed);

    lastError = error;

    }  
}
