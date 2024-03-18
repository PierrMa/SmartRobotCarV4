#include <Wire.h>
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

void setup() {
  //initialisation de la communication série
  Serial.begin(9600);
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

void loop() {
  static float dev_cumul = 0.0;
  unsigned long t1,t2 = 0;
  float tol = 0.05;
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
  Serial.print("déviation cumulé: ");
  Serial.println(dev_cumul);
}
