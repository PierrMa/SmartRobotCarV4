#include <Wire.h>
//motor driver parameter
unsigned int pwmA = 5;
unsigned int pwmB = 6;
unsigned int ain1 = 7;
unsigned int bin1 = 8;
unsigned int stby = 3;
unsigned int vit = 150;
//MPU parameters
unsigned int FSEL = 0; //0=> 250°/s; 1=>500°/s; 2=>1000°/s; 3=>2000°/s
unsigned int MPU_addr = 0b1101000;
unsigned int gyroz_out_h = 0X47;
unsigned int gyroz_out_l = 0X48;
float gz0,gz1=0.0;

//function to read one byte in I²C protocol
int lire_byte_i2c(int addr, int RA){
  int data = 0;
  Wire.beginTransmission(addr);
  Wire.write(RA);
  Wire.endTransmission();
  Wire.requestFrom(addr,1);
  if(Wire.available()!=0){
    data = Wire.read();
    //Serial.println(data);
  }
  return data;
}

//function to write one byte in I²C protocol
void ecrire_byte_i2c(int addr,int RA,int data){
  Wire.beginTransmission(addr);
  Wire.write(RA);
  Wire.write(data);
  Wire.endTransmission();
}

//function to setup the motor driver
void motor_driver_setup(int vit){
  pinMode(pwmA,OUTPUT);
  pinMode(pwmB,OUTPUT);
  pinMode(ain1,OUTPUT);
  pinMode(bin1,OUTPUT);
  pinMode(stby,OUTPUT);
}

//function to setup the MPU6050
void mpu_setup(){
  //initialisaiton de la communication I²C
  Wire.begin();
  //configuration de l'horloge source et désactivation du capteur de température, du mode sleep et cycle (registre 0X6B)
  ecrire_byte_i2c(MPU_addr,0X6B,0X09); 
  //configuration du registre du gyroscope (registre 0x1B)
  ecrire_byte_i2c(MPU_addr,0X1B,0b00<<3);
  //calibrage du gyroscope
  for (int i = 0; i < 100; i++) {
    gz1 = (lire_byte_i2c(MPU_addr,gyroz_out_h)<<8) | (lire_byte_i2c(MPU_addr,gyroz_out_l));
    gz0 += gz1;
  }
  gz0 /= 100; 
}

//function to get the deviation angle
float get_dev_angle(float tol){
  static float dev_cumul = 0.0;
  unsigned long t1,t2 = 0;
  //enregistrement de la date de début de la mesure
  t1 = millis();
  //délai pour respecter la fréquence d'échantillonnage du capteur
  delay (30);
  //lecture de la valeur de la vitesse de rotation angulaire
  gz1 = (lire_byte_i2c(MPU_addr,gyroz_out_h)<<8) | (lire_byte_i2c(MPU_addr,gyroz_out_l));
  //enregistrement de la date de fin de la mesure
  t2 = millis();
  //calcul de l'angle de déviation instantanée
  float dev_inst = (gz1-gz0)/131*(t2-t1)/1000;
  //tolérance à l'erreur de 0.05°
  if (fabs(dev_inst)<tol) dev_inst = 0.0;
  //Serial.print("dev_inst: ");
  //Serial.println(dev_inst);
  //déviation cumulée
  dev_cumul +=dev_inst;
  //Serial.print("déviation cumulé: ");
  //Serial.println(dev_cumul);
  return dev_cumul;
}

void setup() {
  //initialisation de la communication série
  Serial.begin(9600);
  motor_driver_setup(vit);
  mpu_setup();
}

void loop() {
  float tol = 0.05; //tolérance de l'erreur de déviation
  float ref_angle; //angle de départ
  float dev_angle; //angle après mouvement
  unsigned int checking_period = 10; //période de réalisation des contrôls en ms
  unsigned long t1,t2=0; //variable pour mesure de durée
  unsigned long mvt_duration = 2000; //durée du mouvement en ms
  unsigned int Kp = 10; //paramètre proportionnel du correcteur PID

  //Relever l'angle de rotation par rapport à l'axe Z avant un mouvement en ligne droite
  //ref_angle = get_dev_angle(tol);
  ref_angle = 0;
  //relever la date de départ
  t1 = millis();
  //Faire avancer le robot
  analogWrite(pwmA,vit);
  analogWrite(pwmB,vit);
  digitalWrite(ain1,1);
  digitalWrite(bin1,1);
  digitalWrite(stby,1);
  t2 = millis();//pour s'assurer que lors de la première itération, t2-t1<mvt_duration
  while((t2-t1)<mvt_duration){
    //Toutes les checking_period ms, contrôler la présence d'une déviation
    delay(checking_period);
    dev_angle = get_dev_angle(tol);
    //Corriger la trajectoire si déviation
    float error = dev_angle-ref_angle;
    Serial.print("error = ");
    Serial.println(error);
    if(error!=0){
      unsigned int vit_rectif = error*Kp + vit; //nouvelle vitesse à appliquer à l'une des roues
      //restriction de la variation de vit_rectif à [0;255]
      if(vit_rectif>255) vit_rectif=255;
      if(vit_rectif<0) vit_rectif=0;
      Serial.print("vit_rectif = ");
      Serial.println(vit_rectif);
      if(error>0){//si déviation vers la gauche
        //tourner à droite
        analogWrite(pwmA,vit_rectif);
        analogWrite(pwmB,vit);
        digitalWrite(ain1,1);
        digitalWrite(bin1,1);
        digitalWrite(stby,1);
      }
      if(error<0){//si déviation vers la droite
        //tourner à gauche
        analogWrite(pwmA,vit);
        analogWrite(pwmB,vit_rectif);
        digitalWrite(ain1,1);
        digitalWrite(bin1,1);
        digitalWrite(stby,1);
      }
    }
    delay(checking_period);
    analogWrite(pwmA,vit);
    analogWrite(pwmB,vit);
    digitalWrite(ain1,1);
    digitalWrite(bin1,1);
    digitalWrite(stby,1);
    
    
    //date de fin de l'intération
    t2 = millis();
  }
  Serial.println('2ms');
}
