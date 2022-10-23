#include <Arduino.h>
#include <LibRobus.h>

#define GAUCHE 0
#define DROITE 1
#define ANTI_WINDUP 30000

#define KP 0.5
#define KI 0.2

//#define FACTEUR_ROT 0.39
//#define FACTEUR_ANGLE_INTERNE 30.1022
//#define FACTEUR_ANGLE_EXTERNE 74.9055
//#define FACTEUR_CORRECTION_ROTATION 0.99
//#define DISTANCE_ROTATION_180 30.15928
//#define DISTANCE_ROTATION_180 29.2

#define FACTEUR_ROT 0.415
#define FACTEUR_ANGLE_INTERNE 30.8
#define FACTEUR_ANGLE_EXTERNE 74.2
#define FACTEUR_CORRECTION_ROTATION 1.00
#define DISTANCE_ROTATION_180 29.25

#define NBR_ETAPES 17

#define TEMPS_DE_SCAN 50
#define PULSECYLE 560
#define AMAX 12
#define VMAX 350

float calculVitesse(float distanceActuelle, float distanceAncienne);
float calculErreurVitesse(float vitesseActuelle, float vitesseDesiree);
float calculErreurCumuluee(float erreurVitesse, float erreurCumulee);
void gestionVitesseMoteur(int moteur, float vitesseDesiree, float Kp, float Ki, float distanceAncienne, float distanceActuelle, float* erreurCumulee);
float calculCorrection(float erreurVitesse, float erreurCumulee, float Ki, float Kp);
void donnerVitesse(int moteur, float vitesseDesiree, float correction);
float cmApulses(float cm);
float degApulsesExterieur(float deg);
float degApulsesInterieur(float deg);
float calculVitesseDesiree(float distanceDepartMotion, float distanceActuelle, float distanceAParcourir, float amax, float vmax, int* cyclesAccelEnCours,int* cyclesDecelEnCours);
void cycle();
void debug();


//float listeDistance[] = {100,45,65,172,44,100};
//float listeAngle[] = {90,-90,-45,90,-45,0};

float listeDistance[] =   {200,   55,   0,    0,    60,   0,    110/**/,     215,    0,       0,     60,    0,   60,    10,    0,    55,   210};
float listeAngle[] =      {90,    -90,  -90,  90,   -90,  90,   -180/**/,    -180,   -180,    -180,  90,   90,  -90,   90,   90,   -90,  0};




int arreterProgramme =1;

int etapeEnCours = 0;
int rotationEnCours = 0;

float moteurG_distanceDepartMotion = 0;
float moteurD_distanceDepartMotion = 0;
int moteurG_cyclesAccelEnCours =0;
int moteurD_cyclesAccelEnCours =0;
int moteurG_cyclesDecelEnCours =0;
int moteurD_cyclesDecelEnCours =0;


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
  moteurG_distanceDepartMotion = ENCODER_Read(GAUCHE);
  moteurD_distanceDepartMotion = ENCODER_Read(DROITE);
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
    if(ROBUS_IsBumper(3)){
      arreterProgramme = 0;
    }
  }
}


void cycle(){
  moteurG_distanceActuelle = ENCODER_Read(GAUCHE);
  moteurD_distanceActuelle = ENCODER_Read(DROITE);


  if(moteurD_motionTerminee && moteurG_motionTerminee){
    if(rotationEnCours){
      if(etapeEnCours + 1  >= NBR_ETAPES){
        arreterProgramme =1;
      }else{
        etapeEnCours++;
      }
      rotationEnCours = 0;
    }else{
      rotationEnCours =1;
    }

    moteurG_motionTerminee =0;
    moteurD_motionTerminee =0;
    moteurG_distanceDepartMotion = moteurG_distanceActuelle;
    moteurD_distanceDepartMotion = moteurD_distanceActuelle;
    moteurG_cyclesAccelEnCours =0;
    moteurD_cyclesAccelEnCours =0;
    moteurG_cyclesDecelEnCours =0;
    moteurD_cyclesDecelEnCours =0;


    
  }



  if(!rotationEnCours){

 
    moteurG_vitesseDesiree = calculVitesseDesiree(moteurG_distanceDepartMotion,moteurG_distanceActuelle,cmApulses(listeDistance[etapeEnCours]),AMAX,VMAX, &moteurG_cyclesAccelEnCours,&moteurG_cyclesDecelEnCours);
    moteurD_vitesseDesiree = calculVitesseDesiree(moteurD_distanceDepartMotion,moteurD_distanceActuelle,cmApulses(listeDistance[etapeEnCours]),AMAX,VMAX, &moteurD_cyclesAccelEnCours,&moteurD_cyclesDecelEnCours);


    if(moteurG_distanceActuelle >= moteurG_distanceDepartMotion + cmApulses(listeDistance[etapeEnCours])){
      moteurG_vitesseDesiree = 0;
      moteurG_motionTerminee = 1;
    }

    if(moteurD_distanceActuelle >= moteurD_distanceDepartMotion + cmApulses(listeDistance[etapeEnCours])){
      moteurD_vitesseDesiree = 0;
      moteurD_motionTerminee = 1;
    }


  }else{
    //En rotation

    //rotation a gauche
    if(listeAngle[etapeEnCours] > 0){
      moteurG_vitesseDesiree = calculVitesseDesiree(moteurG_distanceDepartMotion,moteurG_distanceActuelle,degApulsesInterieur(abs(listeAngle[etapeEnCours])),AMAX,VMAX*FACTEUR_ROT, &moteurG_cyclesAccelEnCours,&moteurG_cyclesDecelEnCours);;
      moteurD_vitesseDesiree = calculVitesseDesiree(moteurD_distanceDepartMotion,moteurD_distanceActuelle,degApulsesExterieur(abs(listeAngle[etapeEnCours])),AMAX,VMAX, &moteurD_cyclesAccelEnCours,&moteurD_cyclesDecelEnCours);

      if(moteurG_distanceActuelle >= moteurG_distanceDepartMotion + degApulsesInterieur(abs(listeAngle[etapeEnCours]))){
        moteurG_vitesseDesiree = 0;
        moteurG_motionTerminee = 1;
      }

      if(moteurD_distanceActuelle >= moteurD_distanceDepartMotion + degApulsesExterieur(abs(listeAngle[etapeEnCours]))){
        moteurD_vitesseDesiree = 0;
        moteurD_motionTerminee = 1;
      }

    }else if(listeAngle[etapeEnCours] == -180){
      //rotation 180
      moteurG_vitesseDesiree = calculVitesseDesiree(moteurG_distanceDepartMotion,moteurG_distanceActuelle,cmApulses(DISTANCE_ROTATION_180),AMAX,VMAX, &moteurG_cyclesAccelEnCours,&moteurG_cyclesDecelEnCours);;
      moteurD_vitesseDesiree = -calculVitesseDesiree(moteurD_distanceDepartMotion,moteurD_distanceActuelle,cmApulses(DISTANCE_ROTATION_180),AMAX,VMAX, &moteurD_cyclesAccelEnCours,&moteurD_cyclesDecelEnCours);

      if(moteurG_distanceActuelle >= moteurG_distanceDepartMotion + cmApulses(DISTANCE_ROTATION_180)){
        moteurG_vitesseDesiree = 0;
        moteurG_motionTerminee = 1;
      }

      if(moteurD_distanceActuelle <= moteurD_distanceDepartMotion - cmApulses(DISTANCE_ROTATION_180)){
        moteurD_vitesseDesiree = 0;
        moteurD_motionTerminee = 1;
      }
    
    }else{
      //rotation a droite
      moteurG_vitesseDesiree = calculVitesseDesiree(moteurG_distanceDepartMotion,moteurG_distanceActuelle,degApulsesExterieur(abs(listeAngle[etapeEnCours])),AMAX,VMAX, &moteurG_cyclesAccelEnCours,&moteurG_cyclesDecelEnCours);;
      moteurD_vitesseDesiree = calculVitesseDesiree(moteurD_distanceDepartMotion,moteurD_distanceActuelle,degApulsesInterieur(abs(listeAngle[etapeEnCours])),AMAX,VMAX*FACTEUR_ROT, &moteurD_cyclesAccelEnCours,&moteurD_cyclesDecelEnCours);


      if(moteurG_distanceActuelle >= moteurG_distanceDepartMotion + degApulsesExterieur(abs(listeAngle[etapeEnCours]))){
        moteurG_vitesseDesiree = 0;
        moteurG_motionTerminee = 1;
      }

      if(moteurD_distanceActuelle >= moteurD_distanceDepartMotion + degApulsesInterieur(abs(listeAngle[etapeEnCours]))){
        moteurD_vitesseDesiree = 0;
        moteurD_motionTerminee = 1;
      }


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
  Serial.print(moteurG_distanceActuelle - moteurG_distanceAncienne);
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


//Si cycle = 50ms  100%=590 pulses/cycle
void donnerVitesse(int moteur, float vitesseDesiree, float correction){
  float vitesse = (vitesseDesiree + correction)/PULSECYLE;
  MOTOR_SetSpeed(moteur, vitesse);
}


float cmApulses(float cm){
  return cm * 133.7;
}

float degApulsesExterieur(float deg){
  return deg * FACTEUR_ANGLE_EXTERNE * FACTEUR_CORRECTION_ROTATION;
}

float degApulsesInterieur(float deg){
  return deg * FACTEUR_ANGLE_INTERNE * FACTEUR_CORRECTION_ROTATION;
}

float calculVitesseDesiree(float distanceDepartMotion, float distanceActuelle, float distanceAParcourir, float amax, float vmax, int* cyclesAccelEnCours,int* cyclesDecelEnCours){
  float distanceParcourue = abs(distanceActuelle-distanceDepartMotion);
  int tier =0;
  float deltax = (vmax*vmax)/(2*amax);
  float vitesse = 0;

  if((deltax * 2) >= distanceAParcourir){
    deltax = distanceAParcourir/2;
    vmax = sqrt( 2 * deltax * amax);
  }

  if(distanceParcourue <= deltax) tier = 0;
  if(distanceParcourue > deltax) tier = 1;
  if(distanceParcourue >= (distanceAParcourir-deltax)) tier = 2;


  if(tier == 0){
    vitesse = (*cyclesAccelEnCours) * amax;
    if(vitesse >= vmax) vitesse = vmax;
    *cyclesAccelEnCours = *cyclesAccelEnCours + 1;
  }else if (tier == 1)
  {
    vitesse = vmax;
  }else if (tier == 2)
  {
    vitesse = vmax - ((*cyclesDecelEnCours) * amax);
    if(vitesse <= 20) vitesse = 20;
    *cyclesDecelEnCours = *cyclesDecelEnCours + 1;
  }

  return vitesse;
  




}