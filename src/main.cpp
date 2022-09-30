#include <Arduino.h>
#include <LibRobus.h>

#define GAUCHE 0
#define DROITE 1
#define ANTI_WINDUP 30000

#define KP 1
#define KI 0.5

#define FACTEUR_ROT 0.39
#define FACTEUR_ANGLE_INTERNE 26.835
#define FACTEUR_ANGLE_EXTERNE 68.838

#define NBR_ETAPES 6

#define TEMPS_DE_SCAN 50
#define ACCEL_MAX 10

float calculVitesse(float distanceActuelle, float distanceAncienne);
float calculErreurVitesse(float vitesseActuelle, float vitesseDesiree);
float calculErreurCumuluee(float erreurVitesse, float erreurCumulee);
void gestionVitesseMoteur(int moteur, float vitesseDesiree, float Kp, float Ki, float distanceAncienne, float distanceActuelle, float* erreurCumulee);
float calculCorrection(float erreurVitesse, float erreurCumulee, float Ki, float Kp);
void donnerVitesse(int moteur, float vitesseDesiree, float correction);
float cmApulses(float cm);
float degApulsesExterieur(float deg);
float degApulsesInterieur(float deg);
float calculVitesseAcceleration(float vitesse,float vitesseActuelle, float distanceDepartMotion, float distanceActuelle, float ditanceAParcourir);

void cycle();
void debug();


//float listeDistance[] = {100,45,65,172,44,100};
//float listeAngle[] = {90,-90,-45,90,-45,0};

float listeDistance[] = {50,50,50,50,50,50};
float listeAngle[] = {0,0,0,0,0,0};


int arreterProgramme =0;

int etapeEnCours = 0;
int rotationEnCours = 0;

float moteurG_distanceDepartMotion = 0;
float moteurD_distanceDepartMotion = 0;

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
    moteurG_distanceDepartMotion = ENCODER_Read(GAUCHE);
    moteurD_distanceDepartMotion = ENCODER_Read(DROITE);
    
  }



  if(!rotationEnCours){

    //En deplacement lineaire
    //Si a la premiere etape, accelerer doucement
    if((etapeEnCours == 0) && 0){
      //moteurD_vitesseDesiree = calculVitesseAcceleration(moteurG_distanceDepartMotion, moteurG_distanceActuelle);
      //moteurG_vitesseDesiree = calculVitesseAcceleration(moteurD_distanceDepartMotion, moteurD_distanceActuelle);
    }else{
        moteurD_vitesseDesiree = 52;
        moteurG_vitesseDesiree = 52;
    }

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
      moteurD_vitesseDesiree = 52;
      moteurG_vitesseDesiree = 52 * FACTEUR_ROT;

    if(moteurG_distanceActuelle >= moteurG_distanceDepartMotion + degApulsesInterieur(abs(listeAngle[etapeEnCours]))){
      moteurG_vitesseDesiree = 0;
      moteurG_motionTerminee = 1;
    }

    if(moteurD_distanceActuelle >= moteurD_distanceDepartMotion + degApulsesExterieur(abs(listeAngle[etapeEnCours]))){
      moteurD_vitesseDesiree = 0;
      moteurD_motionTerminee = 1;
    }

    }else{
      //rotation a droite
      moteurD_vitesseDesiree = 52 * FACTEUR_ROT;
      moteurG_vitesseDesiree = 52;


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


float cmApulses(float cm){
  return cm * 133.7;
}

float degApulsesExterieur(float deg){
  return deg * FACTEUR_ANGLE_EXTERNE;
}

float degApulsesInterieur(float deg){
  return deg * FACTEUR_ANGLE_INTERNE;
}

/*float calculVitesseAcceleration(float vitesse,float vitesseActuelle, float distanceDepartMotion, float distanceActuelle, float distanceAParcourir){
  if(vitesseDesiree >= vitesseActuelle + 4){
    return vitesseDesiree +4;
  }else if(vitesseDesiree <= vitesseActuelle - 2)

}*/