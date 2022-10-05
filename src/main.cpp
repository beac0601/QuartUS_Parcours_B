#include <Arduino.h>
#include <LibRobus.h>
float Deplacement(float cmdVitesse);

void setup() {
  // put your setup code here, to run once:
}

void loop() {
int commandeVitesse;
int EncoGauche;
int EncoDroit;

//mise a zéro des encodeur par un read
ENCODER_ReadReset(EncoGauche);
ENCODER_ReadReset(EncoDroit);

commandeVitesse = Deplacement(commandeVitesse);



}

float Deplacement(float cmdVitesse)
{
  
int MotGauche;
int MotDroit;
int pulseRecuD;
int pulseRecuG;
int pulseAttenduD;
int pulseAttenduG;

MOTOR_SetSpeed(MotGauche, cmdVitesse);
MOTOR_SetSpeed(MotDroit, cmdVitesse);

//Ajouter un timer eventuellement

pulseAttenduG = calculPulseAttendu;
pulseAttenduD = calculPulseAttendu;

pulseRecuG = ENCODER_Read(EncoGauche);
pulseRecuD = ENCODER_Read(EncoDroit);

if (pulseAttenduG != pulseRecuG && pulseAttenduD != pulseRecuD) 
{
cmdVitesse = PID(a voir);
}
return cmdVitesse;
}