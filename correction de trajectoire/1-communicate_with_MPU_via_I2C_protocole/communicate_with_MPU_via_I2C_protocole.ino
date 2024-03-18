#include <Wire.h>
int FSEL = 0; //0=> 250°/s; 1=>500°/s; 2=>1000°/s; 3=>2000°/s
int MPU_addr = 0b1101000;
int gyroz_out_h = 0X47;
int gyroz_out_l = 0X48;
float gz=0.0;


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
}

void loop() {
  //lecture de la valeur de la vitesse de rotation angulaire
  gz = (lire_byte_i2c(MPU_addr,gyroz_out_h)<<8) | (lire_byte_i2c(MPU_addr,gyroz_out_l));
  Serial.println(gz);
}
