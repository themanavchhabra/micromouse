#define pwma 9
#define AIN1 5
#define AIN2 6
#define pwmb 10
#define BIN2 8
#define BIN1 4

void setup() {
  // put your setup code here, to run once:
  pinMode(pwma, OUTPUT);
  pinMode(pwmb, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:   
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(pwma,100);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(pwmb,100);
  delay(2000);

//  digitalWrite(AIN1, LOW);
//  digitalWrite(AIN2, HIGH);
//  analogWrite(pwma,100);
//  digitalWrite(BIN1, LOW);
//  digitalWrite(BIN2, HIGH);
//  analogWrite(pwmb,100);
//  delay(2000);
//  digitalWrite(AIN1, LOW);
//  digitalWrite(AIN2, HIGH);
//  analogWrite(pwma,0);
//  digitalWrite(BIN1, LOW);
//  digitalWrite(BIN2, HIGH);
//  analogWrite(pwmb,0);
//  delay(2000);
  
}
