#include <Arduino.h>
#include <LibRobus.h>
#include "../Logger/data.h"

#define GAUCHE 0
#define DROITE 1
#define ANTI_WINDUP 30000

#define KP 0.8
#define KI 0.3


#define NBR_ETAPES 6

#define TEMPS_DE_SCAN 50


float calculVitesse(float distanceActuelle, float distanceAncienne);
float calculErreurVitesse(float vitesseActuelle, float vitesseDesiree);
float calculErreurCumuluee(float erreurVitesse, float erreurCumulee);
void gestionVitesseMoteur(int moteur, float vitesseDesiree, float Kp, float Ki, float distanceAncienne, float distanceActuelle, float* erreurCumulee);
float calculCorrection(float erreurVitesse, float erreurCumulee, float Ki, float Kp);
void donnerVitesse(int moteur, float vitesseDesiree, float correction);


void cycle();
void debug();


//float listeDistance[] = {100,45,65,172,44,100};
//float listeAngle[] = {90,-90,-45,90,-45,0};

float listeDistance[] = {50,50,50,50,50,50};
float listeAngle[] = {0,0,0,0,0,0};


int arreterProgramme =0;
int relecture =0;

int etapeEnCours = 0;


int moteurG_motionTerminee = 0;
int moteurD_motionTerminee = 0;

float moteurG_distanceActuelle =0;
float moteurG_distanceAncienne =0;
float moteurG_erreurCumulee =0;
float moteurG_vitesseDesiree =0;

float moteurD_distanceActuelle =0;
float moteurD_distanceAncienne =0;
float moteurD_erreurCumulee =0;
float moteurD_vitesseDesiree =0;


unsigned long tempsCycle = 0;

void setup() {
  // put your setup code here, to run once:
  BoardInit();
  ENCODER_Reset(GAUCHE);
  ENCODER_Reset(DROITE);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  
  if(millis() - tempsCycle > TEMPS_DE_SCAN ){
    tempsCycle = millis();
    cycle();
  }

  while(arreterProgramme){
    MOTOR_SetSpeed(GAUCHE,0);
    MOTOR_SetSpeed(DROITE,0);
  }
  
}

void cycle(){
  moteurG_distanceActuelle = ENCODER_Read(GAUCHE);
  moteurD_distanceActuelle = ENCODER_Read(DROITE);

  if(relecture == 0){
    if(ROBUS_IsBumper(3)) relecture =1;
    if(ROBUS_IsBumper(2)) Serial.println("z");
  } else{
    gestionVitesseMoteur(GAUCHE,pgm_read_float(moteurG_ListeVitesse + etapeEnCours),KP,KI,moteurG_distanceAncienne,moteurG_distanceActuelle,&moteurG_erreurCumulee);
    gestionVitesseMoteur(DROITE,pgm_read_float(moteurD_ListeVitesse + etapeEnCours),KP,KI,moteurD_distanceAncienne,moteurD_distanceActuelle,&moteurD_erreurCumulee);
    etapeEnCours++;
    if (etapeEnCours >= sizeof(moteurD_ListeVitesse)/4) {
      arreterProgramme =1;
    }
  }

  debug();

  moteurD_distanceAncienne = moteurD_distanceActuelle;
  moteurG_distanceAncienne = moteurG_distanceActuelle;
}


void debug(){
  Serial.print(moteurG_distanceActuelle - moteurG_distanceAncienne);
  Serial.print(",");
  Serial.print(moteurD_distanceActuelle - moteurD_distanceAncienne);
  Serial.print(",");
  Serial.print(pgm_read_float(moteurD_ListeVitesse + etapeEnCours));
  Serial.println("");
}


//distanceActuelle(reel): lecture de l'encodeur la plus recente
//distanceAncienne(reel): lecture de l'encodeur precedente
//vitesse(reel): vitesse de rotation en pulse par cycle
float calculVitesse(float distanceActuelle, float distanceAncienne){
  float vitesse = distanceActuelle - distanceAncienne;
  return vitesse;
}



//vitesseActuelle(reel): en pulses par cycle
//vitesseDesiree(reel): en pulses par cycle
//erreurVitesse(reel): en pulses par cycle
float calculErreurVitesse(float vitesseActuelle, float vitesseDesiree){
  float erreurVitesse = vitesseDesiree-vitesseActuelle;
  return erreurVitesse;
}

//distanceActuelle(reel): en pulses
//distanceDesiree(reel): en pulses
//erreurDistance(reel): en pulses
float calculErreurCumuluee(float erreurVitesse, float erreurCumulee){

  //Anti windup

  if((erreurCumulee + erreurVitesse) >= ANTI_WINDUP) return erreurCumulee;
  if((erreurCumulee + erreurVitesse) <= -ANTI_WINDUP) return erreurCumulee;


  erreurCumulee =erreurCumulee+erreurVitesse;
  return erreurCumulee;
}


//moteur(entier):numero du moteur
//vitesseDesiree(reel): en pulses par cycle
//Kp(reel)
//Ki(reel)
void gestionVitesseMoteur(int moteur, float vitesseDesiree, float Kp, float Ki, float distanceAncienne, float distanceActuelle, float* erreurCumulee){
  float vitesseActuelle=calculVitesse(distanceActuelle,distanceAncienne);
  float erreurVitesse= calculErreurVitesse(vitesseActuelle,vitesseDesiree);
  *erreurCumulee = calculErreurCumuluee(erreurVitesse,*erreurCumulee);
  float correction = calculCorrection(erreurVitesse,*erreurCumulee,Kp,Ki);
  donnerVitesse(moteur,vitesseDesiree,correction);
  distanceAncienne = distanceActuelle;

}


float calculCorrection(float erreurVitesse, float erreurCumulee, float Ki, float Kp){
  float correction = erreurCumulee * Ki + erreurVitesse * Kp;
  return correction;
}


//Si cycle = 50ms 50% = 295 pulses/cycle 100%=590 pulses/cycle
void donnerVitesse(int moteur, float vitesseDesiree, float correction){
  float vitesse = (vitesseDesiree + correction)/590;
  MOTOR_SetSpeed(moteur, vitesse);
}




