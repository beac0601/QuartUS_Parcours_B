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

float moteurG_ListeVitesse[] = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,1.00,0.00,0.00,0.00,1.00,10.00,26.00,51.00,72.00,79.00,84.00,91.00,90.00,92.00,94.00,91.00,88.00,83.00,79.00,75.00,77.00,74.00,72.00,73.00,75.00,77.00,77.00,79.00,81.00,81.00,83.00,86.00,89.00,94.00,103.00,107.00,114.00,118.00,110.00,107.00,112.00,108.00,112.00,113.00,98.00,92.00,96.00,87.00,70.00,70.00,73.00,69.00,68.00,68.00,68.00,68.00,73.00,75.00,74.00,78.00,80.00,80.00,82.00,85.00,82.00,76.00,75.00,73.00,71.00,71.00,72.00,73.00,79.00,81.00,88.00,93.00,98.00,103.00,98.00,100.00,101.00,97.00,99.00,99.00,92.00,84.00,86.00,86.00,86.00,84.00,80.00,77.00,77.00,83.00,83.00,80.00,80.00,83.00,83.00,83.00,86.00,85.00,86.00,87.00,86.00,85.00,84.00,85.00,87.00,89.00,83.00,81.00,83.00,86.00,84.00,86.00,90.00,96.00,95.00,89.00,103.00,106.00,100.00,104.00,105.00,104.00,100.00,99.00,96.00,90.00,85.00,84.00,76.00,63.00,57.00,55.00,50.00,43.00,40.00,42.00,46.00,46.00,49.00,56.00,60.00,64.00,65.00,70.00,73.00,70.00,67.00,63.00,61.00,58.00,56.00,52.00,48.00,49.00,52.00,54.00,50.00,45.00,40.00,38.00,35.00,30.00,28.00,33.00,40.00,47.00,59.00,71.00,78.00,79.00,78.00,76.00,77.00,73.00,63.00,58.00,55.00,52.00,48.00,46.00,44.00,45.00,46.00,47.00,50.00,49.00,50.00,53.00,54.00,62.00,67.00,70.00,73.00,74.00,76.00,73.00,70.00,69.00,68.00,64.00,61.00,58.00,58.00,60.00,62.00,62.00,57.00,52.00,48.00,44.00,40.00,41.00,44.00,48.00,52.00,51.00,54.00,52.00,49.00,47.00,46.00,44.00,45.00,47.00,51.00,56.00,58.00,57.00,55.00,53.00,53.00,54.00,56.00,53.00,56.00,58.00,59.00,61.00,62.00,68.00,69.00,69.00,67.00,66.00,64.00,66.00,70.00,74.00,82.00,85.00,91.00,97.00,100.00,104.00,106.00,106.00,103.00,109.00,105.00,103.00,101.00,98.00,92.00,84.00,82.00,80.00,84.00,87.00,91.00,95.00,101.00,110.00,118.00,123.00,122.00,116.00,103.00,96.00,92.00,87.00,85.00,85.00,82.00,78.00,77.00,79.00,80.00,84.00,90.00,87.00,87.00,92.00,97.00,100.00,104.00,107.00,109.00,102.00,105.00,111.00,118.00,117.00,125.00,130.00,129.00,126.00,121.00,123.00,123.00,119.00,112.00,118.00,121.00,124.00,125.00,125.00,128.00,129.00,126.00,125.00,125.00,122.00,117.00,110.00,107.00,112.00,111.00,104.00,102.00,101.00,102.00,106.00,111.00,115.00,124.00,123.00,118.00,116.00,114.00,113.00,118.00,112.00,107.00,114.00,119.00,114.00,110.00,110.00,113.00,110.00,101.00,89.00,90.00,96.00,95.00,93.00,96.00,106.00,113.00,112.00,104.00,101.00,96.00,93.00,94.00,92.00,96.00,99.00,105.00,105.00,109.00,108.00,103.00,96.00,88.00,90.00,83.00,75.00,70.00,74.00,79.00,82.00,82.00,81.00,81.00,82.00,75.00,71.00,68.00,63.00,61.00,53.00,45.00,40.00,39.00,40.00,40.00,43.00,49.00,55.00,56.00,56.00,58.00,56.00,55.00,56.00,56.00,55.00,56.00,56.00,56.00,59.00,56.00,56.00,61.00,64.00,64.00,64.00,69.00,74.00,73.00,71.00,71.00,73.00,69.00,64.00,64.00,62.00,59.00,57.00,54.00,53.00,49.00,46.00,51.00,55.00,61.00,68.00,82.00,97.00,104.00,106.00,104.00,108.00,105.00,103.00,95.00,91.00,88.00,78.00,68.00,65.00,66.00,62.00,50.00,34.00,23.00,21.00,16.00,18.00,19.00,24.00,29.00,32.00,35.00,37.00,38.00,40.00,43.00,43.00,45.00,44.00,44.00,42.00,39.00,34.00,29.00,28.00,24.00,22.00,22.00,24.00,24.00,23.00,25.00,21.00,18.00,16.00,11.00,2.00,0.00,0.00,9.00,7.00,2.00,0.00,0.00,0.00,0.00,-2.00,-1.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,0.00,0.00,0.00,2.00,16.00,36.00,50.00,68.00,83.00,83.00,83.00,88.00,88.00,79.00,70.00,59.00,47.00,38.00,41.00,44.00,46.00,51.00,64.00,69.00,75.00,78.00,83.00,87.00,89.00,93.00,93.00,96.00,97.00,100.00,103.00,105.00,107.00,108.00,109.00,106.00,107.00,109.00,110.00,111.00,114.00,116.00,113.00,112.00,114.00,112.00,112.00,113.00,111.00,111.00,114.00,112.00,116.00,118.00,120.00,117.00,116.00,114.00,110.00,109.00,100.00,91.00,88.00,89.00,87.00,87.00,92.00,92.00,94.00,93.00,96.00,92.00,89.00,89.00,88.00,87.00,84.00,84.00,82.00,86.00,89.00,93.00,97.00,97.00,96.00,94.00,93.00,87.00,82.00,76.00,63.00,58.00,52.00,45.00,46.00,45.00,47.00,54.00,60.00,61.00,65.00,69.00,76.00,78.00,77.00,78.00,79.00,78.00,78.00,76.00,76.00,80.00,79.00,73.00,72.00,72.00,69.00,67.00,67.00,65.00,66.00,65.00,64.00,62.00,65.00,68.00,71.00,72.00,72.00,75.00,78.00,78.00,73.00,72.00,72.00,72.00,70.00,65.00,65.00,69.00,68.00,68.00,68.00,68.00,68.00,64.00,62.00,66.00,61.00,59.00,60.00,64.00,62.00,60.00,59.00,53.00,49.00,43.00,38.00,34.00,35.00,36.00,33.00,23.00,14.00,14.00,9.00,-1.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
float moteurD_ListeVitesse[] = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,0.00,0.00,0.00,0.00,0.00,1.00,1.00,0.00,0.00,1.00,19.0,43.0,59.0,72.0,79.0,85.0,93.0,91.0,89.0,86.0,82.0,78.0,74.0,69.0,67.0,71.0,76.0,79.0,84.0,83.0,88.0,90.0,91.0,90.0,88.0,86.0,83.0,81.0,79.0,80.0,77.0,84.0,94.0,93.0,86.0,96.0,100.,105.,113.,116.,113.,115.,115.,104.,97.0,97.0,94.0,94.0,93.0,87.0,86.0,91.0,86.0,83.0,82.0,75.0,73.0,73.0,76.0,75.0,74.0,78.0,84.0,84.0,81.0,78.0,76.0,76.0,81.0,85.0,86.0,91.0,95.0,90.0,92.0,92.0,91.0,91.0,92.0,89.0,86.0,89.0,90.0,82.0,79.0,77.0,78.0,76.0,76.0,76.0,76.0,76.0,75.0,77.0,79.0,83.0,85.0,89.0,89.0,90.0,90.0,91.0,88.0,87.0,92.0,95.0,102.,105.,110.,114.,110.,105.,107.,99.0,88.0,101.,112.,109.,113.,116.,120.,130.,141.,149.,157.,162.,164.,157.,143.,131.,124.,117.,107.,105.,106.,103.,97.0,93.0,98.0,99.0,99.0,96.0,99.0,106.,108.,111.,112.,119.,124.,131.,132.,132.,130.,128.,126.,118.,126.,136.,143.,147.,144.,144.,150.,151.,136.,131.,128.,128.,125.,123.,123.,126.,131.,126.,126.,130.,135.,134.,132.,134.,132.,132.,128.,128.,127.,126.,124.,116.,114.,108.,102.,95.0,96.0,99.0,100.,100.,101.,108.,112.,113.,115.,117.,119.,120.,125.,126.,130.,133.,133.,130.,126.,121.,114.,108.,104.,105.,101.,98.0,99.0,98.0,97.0,99.0,98.0,94.0,97.0,97.0,101.,104.,107.,109.,112.,113.,111.,114.,116.,116.,117.,116.,121.,122.,120.,117.,115.,110.,108.,103.,99.0,99.0,93.0,95.0,90.0,88.0,88.0,88.0,89.0,87.0,90.0,88.0,88.0,93.0,98.0,95.0,91.0,98.0,104.,114.,118.,122.,129.,129.,130.,133.,133.,132.,125.,107.,99.0,86.0,77.0,71.0,66.0,62.0,55.0,56.0,54.0,55.0,58.0,58.0,61.0,77.0,84.0,88.0,105.,118.,118.,109.,102.,108.,117.,122.,118.,126.,135.,139.,140.,141.,149.,144.,138.,127.,124.,124.,126.,124.,121.,124.,124.,118.,119.,120.,119.,117.,114.,112.,115.,118.,115.,118.,118.,116.,120.,118.,118.,119.,119.,118.,110.,102.,94.0,93.0,83.0,81.0,86.0,86.0,81.0,91.0,108.,113.,109.,110.,118.,129.,129.,125.,122.,121.,126.,118.,111.,103.,101.,94.0,93.0,90.0,84.0,81.0,81.0,81.0,80.0,81.0,81.0,82.0,87.0,93.0,102.,106.,112.,121.,126.,129.,131.,132.,130.,135.,137.,136.,134.,130.,128.,130.,128.,121.,120.,120.,120.,115.,112.,109.,108.,110.,115.,119.,121.,127.,132.,134.,137.,139.,143.,145.,147.,144.,147.,150.,150.,151.,150.,153.,149.,151.,148.,148.,153.,150.,154.,159.,161.,160.,165.,168.,170.,169.,164.,168.,163.,157.,149.,144.,141.,135.,134.,132.,134.,142.,141.,136.,142.,148.,150.,145.,138.,141.,139.,131.,123.,118.,114.,106.,100.,91.0,92.0,89.0,92.0,91.0,93.0,92.0,91.0,89.0,88.0,91.0,90.0,92.0,94.0,94.0,96.0,95.0,95.0,87.0,81.0,70.0,60.0,48.0,38.0,31.0,25.0,20.0,19.0,19.0,18.0,19.0,21.0,21.0,20.0,18.0,16.0,19.0,20.0,24.0,24.0,2.00,1.00,-1.0,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,0.00,0.00,0.00,1.00,0.00,8.00,26.0,29.0,31.0,35.0,34.0,32.0,40.0,54.0,64.0,79.0,90.0,91.0,91.0,98.0,109.,115.,111.,107.,100.,94.0,84.0,80.0,74.0,70.0,74.0,81.0,86.0,84.0,86.0,86.0,84.0,81.0,80.0,83.0,84.0,81.0,81.0,83.0,90.0,96.0,101.,109.,109.,112.,115.,114.,115.,117.,114.,110.,110.,110.,110.,110.,108.,111.,113.,114.,116.,122.,120.,119.,121.,118.,116.,114.,115.,112.,109.,103.,97.0,89.0,83.0,82.0,81.0,80.0,76.0,77.0,80.0,82.0,84.0,80.0,79.0,74.0,71.0,68.0,69.0,70.0,72.0,73.0,73.0,76.0,75.0,72.0,70.0,70.0,71.0,73.0,72.0,72.0,75.0,75.0,73.0,71.0,71.0,70.0,69.0,70.0,74.0,77.0,77.0,78.0,77.0,76.0,76.0,78.0,77.0,74.0,75.0,76.0,78.0,76.0,74.0,73.0,73.0,71.0,71.0,75.0,78.0,79.0,76.0,72.0,74.0,74.0,72.0,71.0,73.0,73.0,71.0,68.0,66.0,67.0,65.0,63.0,61.0,58.0,54.0,49.0,48.0,49.0,52.0,50.0,48.0,47.0,47.0,48.0,48.0,48.0,47.0,47.0,45.0,45.0,45.0,47.0,44.0,35.0,23.0,21.0,20.0,10.0,0.00,0.00,0.00,-5.0,1.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};

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

