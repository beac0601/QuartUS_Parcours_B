#include <Arduino.h>
#include <LibRobus.h>

#define GAUCHE 0
#define DROITE 1
#define ANTI_WINDUP 30000

#define KP 1
#define KI 0.5

#define FACTEUR_ROT 2

float calculVitesse(float distanceActuelle, float distanceAncienne);
float calculErreurVitesse(float vitesseActuelle, float vitesseDesiree);
float calculErreurCumuluee(float erreurVitesse, float erreurCumulee);
void gestionVitesseMoteur(int moteur, float vitesseDesiree, float Kp, float Ki, float distanceAncienne, float distanceActuelle, float* erreurCumulee);
float calculCorrection(float erreurVitesse, float erreurCumulee, float Ki, float Kp);
void donnerVitesse(int moteur, float vitesseDesiree, float correction);


void cycle();
void debug();

float listeAngle[] = {15,-30,45};
float listeDistance[] = {30,45,60};

int etapeEnCours = 0;
int rotationEnCours = 0;

float moteurG_distanceDepartMotion = 0;
float moteurD_distanceDepartMotion = 0;

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
  

  if(millis() - tempsCycle > 50 ){
    cycle();
  }


}


void cycle(){
  moteurG_distanceActuelle = ENCODER_Read(GAUCHE);
  moteurD_distanceActuelle = ENCODER_Read(DROITE);


  int moteurG_motionTerminee = 0;
  int moteurD_motionTerminee = 0;

  if(!rotationEnCours){

    //En deplacement lineaire
    //Si a la premiere etape, accelerer doucement
    if(etapeEnCours == 0){
      moteurD_vitesseDesiree = calculVitesseAcceleration(moteurG_distanceDepartMotion, moteurG_distanceActuelle);
      moteurG_vitesseDesiree = calculVitesseAcceleration(moteurD_distanceDepartMotion, moteurD_distanceActuelle);
    }else{
        moteurD_vitesseDesiree = 52;
        moteurG_vitesseDesiree = 52;
    }

    if(moteurG_distanceActuelle >= moteurG_distanceDepartMotion + listeDistance[etapeEnCours]){
      moteurG_vitesseDesiree = 0;
      moteurG_motionTerminee = 1;
    }

    if(moteurD_distanceActuelle >= moteurD_distanceDepartMotion + listeDistance[etapeEnCours]){
      moteurD_vitesseDesiree = 0;
      moteurD_motionTerminee = 1;
    }

    if(moteurG_motionTerminee && moteurG_motionTerminee){
      rotationEnCours = 1;
    }


  }else{
    //En rotation

    //rotation a gauche
    if(listeAngle[etapeEnCours] > 0){
      moteurD_vitesseDesiree = 52;
      moteurG_vitesseDesiree = 52 * FACTEUR_ROT;



    }else{
      //rotation a droite
      moteurD_vitesseDesiree = 52 * FACTEUR_ROT;
      moteurG_vitesseDesiree = 52;
    }
  }


  gestionVitesseMoteur(GAUCHE,moteurG_vitesseDesiree,KP,KI,moteurG_distanceAncienne,moteurG_distanceActuelle,&moteurG_erreurCumulee);
  gestionVitesseMoteur(DROITE,moteurD_vitesseDesiree,KP,KI,moteurD_distanceAncienne,moteurD_distanceActuelle,&moteurD_erreurCumulee);


  debug();
  moteurD_distanceAncienne = moteurD_distanceActuelle;
  moteurG_distanceAncienne = moteurG_distanceActuelle;
}

void debug(){
  Serial.print(moteurD_vitesseDesiree);
  Serial.print(",");
  Serial.print(moteurD_distanceActuelle - moteurD_distanceAncienne);
  Serial.print(",");
  Serial.print(moteurD_erreurCumulee);
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


//Si cycle = 50ms 50% = 260 pulses/cycle 100%=520 pulses/cycle
void donnerVitesse(int moteur, float vitesseDesiree, float correction){
  float vitesse = (vitesseDesiree + correction)/520;
  MOTOR_SetSpeed(moteur, vitesse);
}
