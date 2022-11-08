int a = 6;
int b = 7;
int pwm = 8;

void setup() {
  // put your setup code here, to run once:
  pinMode(a,OUTPUT);
  pinMode(b,OUTPUT);
  pinMode(pwm,OUTPUT);

  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(a, HIGH);
  digitalWrite(b, LOW);
  analogWrite(pwm, 50);

}
