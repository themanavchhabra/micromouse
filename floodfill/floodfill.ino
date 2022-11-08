#include<SharpIR.h>
#include<util/atomic.h> //atomic block macro
//#include<tuple.h> //python like tuple
#include <Wire.h>
#include <VL53L0X.h>

#define irLeftPin A1
#define irMidPin 0
#define irRightPin A2

VL53L0X lidar;

int baseSpeed = 70; //avg speed
int maxSpeed = 100;
int lastError = 0;

int minFrontIRValue = 18; //while turning , min value before go straight 
int backTurnDelay = 1000; //delay for when turn is back , if turning is done using IR

int distPerRev = 14; //TUNE THIS VALUE TUNE THIS VALUE TUNE THIS VALUE 
int encReadPerRev = 211;
int blockLen = 18;
int encReadPerBlock = encReadPerRev * int(blockLen/distPerRev); //should be roughly 271 +- 1

float irArr[] = {0,0,0};
int irArrBinary[] = {0,0,0};

int orient = 0;
int coordinates[2] = {0,0};

int cells[16][16];
int flood[16][16] = {
{14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14},
{13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
{12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
{11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
{10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
{9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
{8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
{7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7},
{7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7},
{8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
{9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
{10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
{11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
{12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
{13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
{14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14}
};

struct Surrounds{  
    int x3;
    int y3;
    int x0;
    int y0;
    int x1;
    int y1;
    int x2;
    int y2;
  };
Surrounds surrounds;

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
//SharpIR irMid(irMidPin, 1080);
SharpIR irRight(irRightPin,1080);

int avgEncoderReadings(){
  int avg;
  avg = (((posA+posB)/2) % encReadPerBlock);
  if(avg == 0){
    return 271;}
  else{
    return avg;}
  }

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

  for(int i = 0;i<=15;i++){
    for(int j = 0;j<=15;j++){
      cells[i][j] = 0;
      }
      }

//int *cellsPtr[16][16] = &cells;  
  Serial.begin(9600);
  Wire.begin();

  lidar.setTimeout(1000);
    if (!lidar.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
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

float  readIR(){ //updates values of all distance sensors
    irArr[0] = irLeft.distance()/5;
    irArr[1] = (lidar.readRangeSingleMillimeters()/10 - 2.3);
    irArr[2] = irRight.distance()/5;

    irArrBinary[0] = 0;
    irArrBinary[1] = 0;
    irArrBinary[2] = 0;

    if(irArr[0]>0 && irArr[0]<10){irArrBinary[0] == 1;}  //basic ir array to check for walls, 1 if wall, 0 if not
    if(irArr[1]>0 && irArr[1]<5){irArrBinary[1] == 1;}
    if(irArr[2]>0 && irArr[2]<10){irArrBinary[2] == 1;}

    float err = irArr[0] - irArr[2];

    return err;
  }

int updateOrientation(char turn){
  switch(turn){
    case 'L':
      orient-=1;
      if(orient == -1){orient = 3;}
      break;
    case 'R':
      orient+=1;
      if(orient == 4){orient = 0;}
      break;
    case 'B':
      if(orient==0){orient = 2;}
      else if(orient==1){orient = 3;}
      else if(orient==2){orient = 0;}
      else if(orient==3){orient = 1;}
      break;      
    }

    return(orient);
  }

int newCoordinates(int x,int y){
    
    if (orient==0){
        y+=1;}
    if (orient==1){
        x+=1;}
    if (orient==2){
        y-=1;}
    if (orient==3){
        x-=1;}

   coordinates[0] = x;
   coordinates[1] = y;

   return coordinates;
  }

void updateWalls(int x, int y){ //important! update the irArrBinary before calling this function
    if((irArrBinary[0] == 1 && irArrBinary[1] == 1) && irArrBinary[2] == 1){
      if (orient==0){ 
            cells[y][x]= 13;}
        else if  (orient==1){ 
            cells[y][x]= 12;}
        else if  (orient==2){ 
            cells[y][x]= 11;}
        else if  (orient==3){
            cells[y][x]= 14;}
      }

      else if ((irArrBinary[0] == 1 && irArrBinary[1] != 1) && irArrBinary[2] == 1){
        if (orient==0 or orient== 2){
            cells[y][x]= 9;}
        else if  (orient==1 or orient==3){ 
            cells[y][x]= 10;}
        }

      else if ((irArrBinary[0] == 1 && irArrBinary[1] == 1) && irArrBinary[2] != 1){
          if (orient==0){
            cells[y][x]= 8;}
        else if  (orient==1){
            cells[y][x]= 7;}
        else if  (orient==2){ 
            cells[y][x]= 6;}
        else if  (orient==3){ 
            cells[y][x]= 5;}
        }

      else if ((irArrBinary[0] != 1 && irArrBinary[1] == 1) && irArrBinary[2] == 1){
        if (orient==0){ 
            cells[y][x]= 7;}
        else if  (orient==1){ 
            cells[y][x]= 6;}
        else if  (orient==2){ 
            cells[y][x]= 5;}
        else if  (orient==3){ 
            cells[y][x]= 8;}
        }

      else if (irArrBinary[1] == 1){
        if (orient==0){
            cells[y][x]= 2;}
        else if  (orient==1){
            cells[y][x]= 3;}
        else if  (orient==2){ 
            cells[y][x]= 4;}
        else if  (orient==3){ 
            cells[y][x]= 1;}
        }

      else if (irArrBinary[0] == 1){
        if (orient==0){ 
            cells[y][x]= 1;}
        else if  (orient==1){ 
            cells[y][x]= 2;}
        else if  (orient==2){ 
            cells[y][x]= 3;}
        else if  (orient==3){ 
            cells[y][x]= 4;}
        }

      else if (irArrBinary[2] == 1){
        if (orient==0){ 
            cells[y][x]= 3;}
        else if  (orient==1){ 
            cells[y][x]= 4;}
        else if  (orient==2){ 
            cells[y][x]= 1;}
        else if  (orient==3){ 
            cells[y][x]= 2;}
        }

       else{
        cells[y][x]= 15;
        }
  }

bool isAccessible(int x, int y, int x1, int y1){
  
    if (x==x1){
        if(y>y1){
            if(cells[y][x]==4 or cells[y][x]==5 or cells[y][x]==6 or cells[y][x]==10 or cells[y][x]==11 or cells[y][x]==12 or cells[y][x]==14 or cells[y1][x1]==2 or cells[y1][x1]==7 or cells[y1][x1]==8 or cells[y1][x1]==10 or cells[y1][x1]==12 or cells[y1][x1]==13 or cells[y1][x1]==14 ){
                return (false);}
            else{
                return(true);}
        }
        else{
            if(cells[y][x]==2 or cells[y][x]==7 or cells[y][x]==8 or cells[y][x]==10 or cells[y][x]==12 or cells[y][x]==13 or cells[y][x]==14 or cells[y1][x1]==4 or cells[y1][x1]==5 or cells[y1][x1]==6 or cells[y1][x1]==10 or cells[y1][x1]==11 or cells[y1][x1]==12 or cells[y1][x1]==14 ){
                return (false);}
            else{
                return(true);}
}
    }
    
            

    else if  (y==y1){
        if(x>x1){
            if(cells[y][x]==1 or cells[y][x]==5 or cells[y][x]==8 or cells[y][x]==9 or cells[y][x]==11 or cells[y][x]==13 or cells[y][x]==14 or cells[y1][x1]==3 or cells[y1][x1]==6 or cells[y1][x1]==7 or cells[y1][x1]==9 or cells[y1][x1]==11 or cells[y1][x1]==12 or cells[y1][x1]==13 ){
                return (false);}
            else{
                return (true);}
        }
        else{
            if(cells[y][x]==3 or cells[y][x]==6 or cells[y][x]==7 or cells[y][x]==9 or cells[y][x]==11 or cells[y][x]==12 or cells[y][x]==13 or cells[y1][x1]==1 or cells[y1][x1]==5 or cells[y1][x1]==8 or cells[y1][x1]==9 or cells[y1][x1]==11 or cells[y1][x1]==13 or cells[y1][x1]==14 ){
                return (false);}
            else{
                return (true);}
        }
        }

  }

void getSurrounds(int x , int y){

//Surrounds surrounds;
 
   surrounds.x3 = x-1;
   surrounds.y3=y;
   surrounds.x0=x;
   surrounds.y0=y+1;
   surrounds.x1=x+1;
   surrounds.y1=y;
   surrounds.x2=x;
   surrounds.y2=y-1;

     if(surrounds.x1 >= 16){surrounds.x1 = -1;}
     if(surrounds.y0 >= 16){surrounds.y0 =- 1;}

    return(surrounds);
}

char selectTurn(int x, int y){
    getSurrounds(x,y);

//    int flag = 0;
    int val =  flood[x][y];
    int minCell = 0;

  if (isAccessible(x,y,surrounds.x0,surrounds.y0)){
        if (flood[surrounds.y0][surrounds.x0]==val-1){
            minCell=0;
        }
  }

    if (isAccessible(x,y,surrounds.x1,surrounds.y1)){
        if (flood[surrounds.y1][surrounds.x1]==val-1){
            minCell=1;
        }
    }

    if (isAccessible(x,y,surrounds.x2,surrounds.y2)){
        if (flood[surrounds.y2][surrounds.x2]==val-1){
            minCell=2;
        }
    }

    if (isAccessible(x,y,surrounds.x3,surrounds.y3)){
        if (flood[surrounds.y3][surrounds.x3]==val-1){
            minCell=3  ;  
        }
    }

    if (minCell==orient){
        return ('F');
    }
    else if((minCell==orient-1) or (minCell== orient+3)){
        return('L');
    }
    else if ((minCell==orient+1) or (minCell== orient-3)){
        return('R');
    }
    else{
        return('B');
    }
}

void turn(char dir){
  switch(dir){
    case 'L':
      while(irArr[2] < minFrontIRValue){
        motorRight.setMotor(HIGH,LOW,baseSpeed);
        motorLeft.setMotor(HIGH,LOW,0);
    }
    break;
    case 'R':
      while(irArr[2] < minFrontIRValue){
        motorRight.setMotor(HIGH,LOW,0);
        motorLeft.setMotor(HIGH,LOW,baseSpeed);
    }
    break;
    case 'B':
      motorRight.setMotor(LOW,HIGH,baseSpeed);
      motorLeft.setMotor(LOW,HIGH,baseSpeed);

      delay(backTurnDelay);

      while(irArr[2] < minFrontIRValue){
        motorRight.setMotor(LOW,HIGH,0);
        motorLeft.setMotor(HIGH,LOW,baseSpeed);
      }
      
    };

    updateOrientation(dir);
}

void goStraight(){
  float kp = 0.75;
  float kd = 1;

  readIR();
//  int encAvg = avgEncoderReadings();

//  if(irArrBinary[0] == 0 or irArrBinary[2] == 0){
//    char dir = selectTurn(coordinates[0],coordinates[1]);
//    turn(dir);
//    }

    

  while(1){
    readIR();
    if(irArrBinary[0] == 0 or irArrBinary[2] == 0){
      char dir = selectTurn(coordinates[0],coordinates[1]);
      turn(dir);
    }

    if(avgEncoderReadings > 269 and avgEncoderReadings() < 273){
        newCoordinates(coordinates[0], coordinates[1]);
        updateWalls(coordinates[0],coordinates[1]);
      }
    
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
