int pwmA = 5;
int pwmB = 6;
int ain1 = 7;
int bin1 = 8;
int stby = 3;
unsigned int vit = 250;

void setup() {
  pinMode(pwmA,OUTPUT);
  pinMode(pwmB,OUTPUT);
  pinMode(ain1,OUTPUT);
  pinMode(bin1,OUTPUT);
  pinMode(stby,OUTPUT);
}

void loop() {
  //avancer
  analogWrite(pwmA,vit);
  analogWrite(pwmB,vit);
  digitalWrite(ain1,1);
  digitalWrite(bin1,1);
  digitalWrite(stby,1);
  delay(2000);

  //reculer
  analogWrite(pwmA,vit);
  analogWrite(pwmB,vit);
  digitalWrite(ain1,0);
  digitalWrite(bin1,0);
  digitalWrite(stby,1);
  delay(2000);

  //toupie gauche
  analogWrite(pwmA,vit);
  analogWrite(pwmB,vit);
  digitalWrite(ain1,0);
  digitalWrite(bin1,1);
  digitalWrite(stby,1);
  delay(2000);

  //toupie droite
  analogWrite(pwmA,vit);
  analogWrite(pwmB,vit);
  digitalWrite(ain1,1);
  digitalWrite(bin1,0);
  digitalWrite(stby,1);
  delay(2000);

  //avant gauche
  analogWrite(pwmA,vit);
  analogWrite(pwmB,vit/2);
  digitalWrite(ain1,1);
  digitalWrite(bin1,1);
  digitalWrite(stby,1);
  delay(2000);

  //arrière gauche
  analogWrite(pwmA,vit);
  analogWrite(pwmB,vit/2);
  digitalWrite(ain1,0);
  digitalWrite(bin1,0);
  digitalWrite(stby,1);
  delay(2000);

  //avant droite
  analogWrite(pwmA,vit/2);
  analogWrite(pwmB,vit);
  digitalWrite(ain1,1);
  digitalWrite(bin1,1);
  digitalWrite(stby,1);
  delay(2000);

  //arrière droite
  analogWrite(pwmA,vit/2);
  analogWrite(pwmB,vit);
  digitalWrite(ain1,0);
  digitalWrite(bin1,0);
  digitalWrite(stby,1);
  delay(2000);

  //stop
  digitalWrite(stby,0);
  delay(2000);
}
