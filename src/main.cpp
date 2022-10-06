#include <Arduino.h>
#include <LibRobus.h>

#define GAUCHE 0
#define DROITE 1
#define ANTI_WINDUP 30000

#define KP 0.8
#define KI 0.3

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

const PROGMEM float moteurG_ListeVitesse[] = {.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,0.00,0.00,0.00,0.00,2.00,0.00,0.00,0.00,0.00,0.00,0.00,4.00,27.00,44.00,53.00,55.00,49.00,39.00,25.00,17.00,17.00,22.00,29.00,30.00,30.00,30.00,28.00,26.00,26.00,25.00,27.00,29.00,29.00,25.00,26.00,26.00,31.00,39.00,46.00,44.00,47.00,49.00,44.00,35.00,40.00,46.00,44.00,28.00,16.00,12.00,11.00,27.00,39.00,49.00,53.00,60.00,70.00,77.00,73.00,71.00,70.00,69.00,72.00,72.00,69.00,64.00,66.00,67.00,69.00,70.00,74.00,79.00,79.00,79.00,78.00,80.00,79.00,82.00,86.00,89.00,90.00,91.00,96.00,102.00,107.00,112.00,121.00,128.00,128.00,129.00,132.00,134.00,129.00,121.00,118.00,116.00,109.00,104.00,110.00,114.00,116.00,123.00,125.00,123.00,121.00,123.00,122.00,124.00,123.00,125.00,129.00,129.00,126.00,123.00,119.00,116.00,120.00,121.00,115.00,106.00,103.00,103.00,98.00,102.00,104.00,109.00,115.00,120.00,125.00,120.00,112.00,107.00,110.00,112.00,125.00,131.00,130.00,125.00,120.00,121.00,121.00,114.00,113.00,113.00,115.00,125.00,130.00,122.00,114.00,106.00,102.00,98.00,94.00,97.00,99.00,102.00,100.00,104.00,103.00,99.00,101.00,101.00,99.00,89.00,79.00,73.00,77.00,78.00,75.00,72.00,70.00,67.00,64.00,64.00,70.00,74.00,80.00,80.00,77.00,65.00,56.00,50.00,43.00,34.00,29.00,24.00,27.00,30.00,30.00,26.00,20.00,18.00,17.00,14.00,7.00,0.00,0.00,0.00,0.00,0.00,1.00,10.00,9.00,1.00,0.00,0.00,0.00,2.00,21.00,27.00,27.00,32.00,41.00,46.00,51.00,50.00,56.00,64.00,70.00,64.00,52.00,45.00,34.00,29.00,23.00,24.00,24.00,27.00,36.00,46.00,49.00,44.00,48.00,56.00,56.00,48.00,43.00,36.00,29.00,26.00,29.00,31.00,32.00,36.00,39.00,40.00,43.00,42.00,39.00,45.00,57.00,63.00,65.00,61.00,65.00,67.00,66.00,69.00,79.00,89.00,92.00,95.00,99.00,96.00,90.00,86.00,90.00,94.00,95.00,94.00,91.00,94.00,100.00,106.00,108.00,112.00,109.00,106.00,104.00,106.00,106.00,105.00,108.00,112.00,119.00,121.00,123.00,121.00,115.00,109.00,105.00,100.00,101.00,104.00,104.00,104.00,107.00,105.00,104.00,96.00,88.00,84.00,83.00,77.00,80.00,90.00,96.00,101.00,103.00,114.00,126.00,130.00,122.00,126.00,126.00,120.00,117.00,120.00,117.00,111.00,107.00,104.00,102.00,113.00,121.00,129.00,135.00,134.00,133.00,129.00,121.00,118.00,118.00,115.00,114.00,112.00,111.00,111.00,117.00,123.00,121.00,115.00,114.00,116.00,115.00,111.00,105.00,100.00,98.00,96.00,96.00,98.00,100.00,98.00,100.00,103.00,103.00,104.00,100.00,93.00,86.00,81.00,76.00,71.00,73.00,72.00,75.00,76.00,83.00,97.00,114.00,127.00,131.00,126.00,108.00,98.00,90.00,80.00,69.00,63.00,55.00,50.00,54.00,60.00,63.00,65.00,67.00,70.00,72.00,76.00,78.00,73.00,73.00,76.00,70.00,63.00,69.00,70.00,70.00,75.00,84.00,87.00,96.00,103.00,112.00,124.00,133.00,131.00,126.00,123.00,114.00,105.00,93.00,87.00,85.00,83.00,85.00,80.00,79.00,70.00,62.00,52.00,44.00,40.00,40.00,39.00,30.00,13.00,-2.00,0.00,0.00,0.00,0.00,-3.00,-7.00,-24.00,-30.00,-48.00,-61.00,-77.00,-86.00,-83.00,-66.00,-53.00,-43.00,-39.00,-37.00,-37.00,-41.00,-46.00,-50.00,-49.00,-51.00,-56.00,-61.00,-64.00,-59.00,-50.00,-39.00,-34.00,-30.00,-27.00,-35.00,-52.00,-65.00,-71.00,-72.00,-74.00,-77.00,-77.00,-79.00,-75.00,-72.00,-76.00,-81.00,-85.00,-85.00,-85.00,-81.00,-82.00,-81.00,-87.00,-91.00,-86.00,-83.00,-83.00,-87.00,-86.00,-88.00,-89.00,-87.00,-90.00,-91.00,-91.00,-88.00,-80.00,-77.00,-75.00,-75.00,-74.00,-73.00,-73.00,-82.00,-81.00,-73.00,-73.00,-80.00,-83.00,-84.00,-81.00,-81.00,-82.00,-81.00,-77.00,-74.00,-74.00,-78.00,-81.00,-80.00,-80.00,-84.00,-85.00,-84.00,-81.00,-83.00,-85.00,-85.00,-82.00,-78.00,-78.00,-79.00,-78.00,-78.00,-74.00,-75.00,-75.00,-69.00,-65.00,-63.00,-61.00,-58.00,-51.00,-38.00,-23.00,-10.00,-1.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
const PROGMEM float moteurD_ListeVitesse[] = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,1.00,1.00,0.00,0.00,0.00,2.00,21.0,55.0,80.0,81.0,79.0,70.0,67.0,71.0,68.0,62.0,54.0,51.0,50.0,49.0,49.0,51.0,57.0,51.0,49.0,52.0,53.0,51.0,58.0,65.0,72.0,75.0,72.0,73.0,73.0,73.0,80.0,83.0,76.0,67.0,64.0,65.0,54.0,57.0,68.0,73.0,67.0,61.0,66.0,71.0,67.0,57.0,40.0,34.0,38.0,42.0,48.0,51.0,65.0,78.0,84.0,89.0,91.0,92.0,91.0,88.0,88.0,90.0,87.0,83.0,79.0,70.0,64.0,64.0,67.0,71.0,72.0,68.0,62.0,61.0,58.0,58.0,56.0,54.0,58.0,58.0,56.0,57.0,60.0,60.0,58.0,57.0,59.0,61.0,60.0,61.0,59.0,59.0,57.0,51.0,49.0,50.0,48.0,45.0,45.0,42.0,38.0,32.0,26.0,19.0,17.0,20.0,21.0,25.0,27.0,29.0,34.0,42.0,48.0,57.0,63.0,62.0,58.0,53.0,48.0,43.0,39.0,35.0,33.0,34.0,35.0,35.0,36.0,36.0,39.0,43.0,45.0,53.0,58.0,62.0,66.0,68.0,74.0,77.0,74.0,66.0,62.0,61.0,65.0,68.0,67.0,68.0,68.0,70.0,70.0,68.0,64.0,65.0,64.0,66.0,78.0,82.0,87.0,94.0,101.,103.,103.,100.,98.0,95.0,88.0,88.0,88.0,94.0,105.,117.,133.,147.,160.,169.,177.,181.,169.,159.,146.,135.,121.,122.,122.,121.,122.,116.,111.,104.,102.,105.,114.,122.,127.,128.,125.,125.,119.,113.,103.,94.0,92.0,90.0,84.0,75.0,75.0,80.0,89.0,105.,113.,113.,116.,118.,114.,120.,125.,130.,129.,129.,130.,123.,119.,108.,106.,103.,105.,109.,112.,108.,101.,99.0,93.0,96.0,94.0,89.0,85.0,78.0,77.0,73.0,74.0,73.0,76.0,81.0,89.0,102.,108.,116.,119.,128.,132.,133.,126.,123.,120.,112.,101.,86.0,79.0,77.0,81.0,83.0,81.0,85.0,81.0,81.0,79.0,75.0,72.0,68.0,66.0,69.0,77.0,80.0,79.0,75.0,71.0,67.0,63.0,58.0,51.0,44.0,41.0,36.0,33.0,30.0,30.0,30.0,29.0,28.0,23.0,16.0,17.0,20.0,25.0,28.0,29.0,29.0,24.0,19.0,18.0,22.0,25.0,26.0,34.0,43.0,55.0,66.0,81.0,87.0,94.0,84.0,69.0,62.0,56.0,51.0,47.0,43.0,39.0,41.0,45.0,49.0,59.0,66.0,69.0,71.0,71.0,74.0,75.0,70.0,68.0,67.0,64.0,61.0,59.0,60.0,60.0,62.0,63.0,66.0,66.0,66.0,68.0,64.0,61.0,54.0,50.0,50.0,50.0,45.0,44.0,41.0,37.0,31.0,32.0,30.0,35.0,34.0,36.0,38.0,39.0,36.0,38.0,37.0,38.0,38.0,31.0,27.0,27.0,27.0,24.0,17.0,5.00,0.00,0.00,2.00,8.00,8.00,10.0,9.00,4.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,4.00,23.0,26.0,19.0,15.0,18.0,20.0,27.0,31.0,30.0,31.0,36.0,42.0,51.0,53.0,49.0,42.0,39.0,35.0,33.0,26.0,20.0,16.0,16.0,14.0,12.0,7.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,-3.0,0.00,-1.0,0.00,-3.0,-30.,-68.,-98.,-88.,-56.,-36.,-27.,-35.,-45.,-49.,-48.,-45.,-42.,-41.,-39.,-46.,-59.,-65.,-61.,-48.,-36.,-30.,-36.,-44.,-51.,-57.,-62.,-64.,-67.,-75.,-87.,-94.,-102,-106,-106,-104,-107,-112,-118,-123,-125,-122,-125,-121,-118,-119,-117,-121,-125,-125,-123,-126,-129,-128,-127,-126,-128,-129,-130,-129,-130,-126,-129,-124,-117,-117,-116,-113,-113,-113,-112,-108,-104,-102,-103,-99.,-96.,-97.,-98.,-101,-101,-101,-96.,-98.,-96.,-97.,-96.,-97.,-96.,-93.,-94.,-87.,-87.,-84.,-83.,-81.,-76.,-76.,-76.,-71.,-70.,-68.,-66.,-60.,-52.,-40.,-25.,-7.0,4.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};

int arreterProgramme =0;
int relecture =0;

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
    tempsCycle = millis();
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
    gestionVitesseMoteur(GAUCHE,moteurG_ListeVitesse[etapeEnCours],KP,KI,moteurG_distanceAncienne,moteurG_distanceActuelle,&moteurG_erreurCumulee);
    gestionVitesseMoteur(DROITE,moteurD_ListeVitesse[etapeEnCours],KP,KI,moteurD_distanceAncienne,moteurD_distanceActuelle,&moteurD_erreurCumulee);
    etapeEnCours++;
    if (etapeEnCours >= sizeof(moteurD_ListeVitesse)/4) {
      arreterProgramme =1;
    }
  }

  debug();

  moteurD_distanceAncienne = moteurD_distanceActuelle;
  moteurG_distanceAncienne = moteurG_distanceActuelle;
}


/*void cycle(){
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
*/

void debug(){
  Serial.print(moteurG_distanceActuelle - moteurG_distanceAncienne);
  Serial.print(",");
  Serial.print(moteurD_distanceActuelle - moteurD_distanceAncienne);
  Serial.print(",");
  Serial.print(moteurD_ListeVitesse[etapeEnCours]);
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


float cmApulses(float cm){
  return cm * 133.7;
}

float degApulsesExterieur(float deg){
  return deg * FACTEUR_ANGLE_EXTERNE;
}

float degApulsesInterieur(float deg){
  return deg * FACTEUR_ANGLE_INTERNE;
}

