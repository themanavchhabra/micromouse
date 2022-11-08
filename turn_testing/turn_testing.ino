#include<SharpIR.h>
#include<util/atomic.h> //atomic block macro
//#include<tuple.h> //python like tuple
#include <Wire.h>
#include <VL53L0X.h>

#define irLeftPin A2
//#define irMidPin 0
#define irRightPin A1

int baseSpeed = 70; //avg speed
int maxSpeed = 100;
int lastError = 0;

int minFrontIRValue = 18; //while turning , min value before go straight 
int backTurnDelay = 1000; //delay for when turn is back , if turning is done using IR


float irArr[] = {0,0,0};
int irArrBinary[] = {0,0,0};

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
//      Serial.println(pwmVal);
      digitalWrite(m1, dira);
      digitalWrite(m2, dirb);
      digitalWrite(pwm, pwmVal);
      }
};

Motors motorLeft(3,11,5,6,9);
Motors motorRight(2,12,4,11,10);

SharpIR irLeft(irLeftPin, 1080);
//SharpIR irMid(irMidPin, 1080);
SharpIR irRight(irRightPin,1080);


VL53L0X sensor;


void setup() {
  // put your setup code here, to run once:


  Serial.begin(9600);
  Wire.begin();

    sensor.setTimeout(1000);
    if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  #if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
}

float  readIR(){ //updates values of all distance sensors
    irArr[0] = irLeft.distance()/5;
    Serial.print("ir left ");
    Serial.println(irArr[0]);
    Serial.println(sensor.readRangeSingleMillimeters());
        Serial.print("sensor ");
    Serial.println(irArr[1]);
    irArr[2] = irRight.distance()/5;
        Serial.print("ir right ");
    Serial.println(irArr[2]);

    irArrBinary[0] = 0;
    irArrBinary[1] = 0;
    irArrBinary[2] = 0;

    if(irArr[0]>0 && irArr[0]<10){irArrBinary[0] = 1;}  //basic ir array to check for walls, 1 if wall, 0 if not
    if(irArr[1]>0 && irArr[1]<5){irArrBinary[1] = 1;}
    if(irArr[2]>0 && irArr[2]<10){irArrBinary[2] = 1;}

    Serial.print(irArrBinary[0]);
    Serial.print(irArrBinary[1]);
    Serial.print(irArrBinary[2]);

    float err = irArr[0] - irArr[2];

    return err;
}

char selectTurn(){
  if(irArrBinary[0] == 0){
    Serial.println('L');
    return 'L';}
    
  else if (irArrBinary[2] == 0){
    Serial.println('R');
    return 'R';}
  else{
    Serial.println('B');
    return 'B';}
  }

void turn(char dir){
  switch(dir){
    case 'L':
      while(sensor.readRangeSingleMillimeters() < minFrontIRValue){
        motorRight.setMotor(HIGH,LOW,baseSpeed);
        motorLeft.setMotor(HIGH,LOW,0);
    }
    break;
    case 'R':
    Serial.println("MOving right ");
    Serial.println(sensor.readRangeSingleMillimeters());
      while(sensor.readRangeSingleMillimeters() < minFrontIRValue){
        motorRight.setMotor(HIGH,LOW,0);
        motorLeft.setMotor(HIGH,LOW,baseSpeed);
    }
    break;
    case 'B':
      motorRight.setMotor(LOW,HIGH,baseSpeed);
      motorLeft.setMotor(LOW,HIGH,baseSpeed);

//      delay(backTurnDelay);

      while(sensor.readRangeSingleMillimeters() < minFrontIRValue){
        motorRight.setMotor(LOW,HIGH,0);
        motorLeft.setMotor(HIGH,LOW,baseSpeed);
      }
      
    };

//    motorRight.setMotor(HIGH,LOW,baseSpeed);
//    motorLeft.setMotor(HIGH,LOW,baseSpeed);
}

void loop() {
  // put your main code here, to run repeatedly:
//Serial.println(sensor.readRangeSingleMillimeters());
  goStraight();
}

void goStraight(){
  float kp = 0.75;
  float kd = 1;

  readIR(); 

    motorRight.setMotor(HIGH,LOW,baseSpeed);
    motorLeft.setMotor(HIGH,LOW,baseSpeed);

  while(1){
    Serial.print("AAAAAAAAAAA");
    Serial.print(sensor.readRangeSingleMillimeters());
    Serial.println("AAAAAAAAAAAAAAAAAAA");
    readIR();
    if(irArrBinary[0] == 0 or irArrBinary[2] == 0){
//      delay(5000);
      char dir = selectTurn();
      turn(dir);
    }
    
    int error = readIR();
    Serial.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
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
