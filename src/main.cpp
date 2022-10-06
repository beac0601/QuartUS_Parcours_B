#include <Arduino.h>
#include <LibRobus.h>
#include "../Logger/data.h"

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

//const PROGMEM float moteurG_ListeVitesse[] = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,2.00,14.00,38.00,58.00,69.00,85.00,92.00,97.00,92.00,89.00,93.00,101.00,110.00,114.00,114.00,112.00,112.00,116.00,116.00,113.00,112.00,113.00,109.00,109.00,103.00,100.00,114.00,116.00,114.00,117.00,112.00,120.00,125.00,136.00,148.00,157.00,166.00,172.00,172.00,172.00,179.00,187.00,191.00,189.00,184.00,177.00,175.00,168.00,164.00,159.00,155.00,153.00,151.00,147.00,142.00,142.00,138.00,132.00,123.00,123.00,120.00,119.00,117.00,119.00,127.00,138.00,147.00,151.00,167.00,171.00,177.00,182.00,181.00,181.00,171.00,169.00,170.00,164.00,158.00,153.00,149.00,140.00,142.00,138.00,134.00,138.00,147.00,158.00,162.00,161.00,154.00,148.00,145.00,145.00,133.00,128.00,124.00,126.00,127.00,125.00,119.00,117.00,115.00,109.00,98.00,96.00,97.00,91.00,80.00,78.00,81.00,81.00,82.00,82.00,93.00,100.00,97.00,90.00,85.00,79.00,74.00,72.00,70.00,76.00,82.00,92.00,95.00,97.00,97.00,93.00,91.00,88.00,95.00,105.00,115.00,128.00,133.00,135.00,144.00,143.00,141.00,138.00,147.00,154.00,157.00,166.00,162.00,165.00,163.00,155.00,156.00,155.00,146.00,141.00,141.00,134.00,130.00,123.00,117.00,117.00,117.00,125.00,128.00,135.00,141.00,148.00,149.00,139.00,144.00,148.00,146.00,155.00,161.00,165.00,165.00,155.00,142.00,132.00,126.00,121.00,122.00,127.00,133.00,133.00,128.00,118.00,132.00,137.00,148.00,153.00,155.00,168.00,166.00,163.00,169.00,167.00,168.00,167.00,172.00,168.00,160.00,165.00,176.00,179.00,185.00,186.00,173.00,168.00,168.00,166.00,166.00,177.00,177.00,178.00,183.00,181.00,187.00,182.00,173.00,158.00,145.00,147.00,155.00,156.00,148.00,150.00,143.00,140.00,147.00,143.00,142.00,147.00,148.00,155.00,168.00,178.00,188.00,190.00,186.00,174.00,170.00,184.00,182.00,177.00,185.00,204.00,209.00,223.00,229.00,228.00,237.00,234.00,224.00,219.00,216.00,222.00,232.00,233.00,233.00,247.00,248.00,247.00,241.00,232.00,230.00,226.00,215.00,205.00,201.00,190.00,189.00,179.00,175.00,174.00,167.00,171.00,179.00,178.00,176.00,174.00,173.00,181.00,195.00,202.00,218.00,235.00,246.00,258.00,269.00,269.00,268.00,271.00,269.00,264.00,257.00,255.00,254.00,252.00,249.00,232.00,232.00,237.00,238.00,232.00,243.00,259.00,270.00,281.00,280.00,280.00,264.00,252.00,247.00,250.00,245.00,262.00,285.00,295.00,316.00,321.00,313.00,314.00,316.00,316.00,315.00,307.00,289.00,289.00,287.00,282.00,273.00,269.00,254.00,238.00,232.00,223.00,231.00,226.00,222.00,215.00,216.00,207.00,205.00,199.00,178.00,169.00,153.00,135.00,127.00,126.00,124.00,116.00,116.00,112.00,116.00,117.00,114.00,114.00,117.00,126.00,128.00,132.00,131.00,140.00,146.00,149.00,158.00,163.00,163.00,164.00,167.00,166.00,172.00,175.00,175.00,181.00,179.00,184.00,181.00,184.00,192.00,199.00,204.00,206.00,207.00,201.00,193.00,180.00,161.00,146.00,140.00,128.00,131.00,125.00,121.00,124.00,118.00,121.00,137.00,149.00,162.00,182.00,185.00,181.00,186.00,200.00,216.00,223.00,216.00,217.00,212.00,200.00,192.00,195.00,183.00,174.00,176.00,176.00,184.00,189.00,183.00,180.00,194.00,196.00,186.00,179.00,172.00,170.00,164.00,158.00,155.00,150.00,146.00,142.00,131.00,117.00,104.00,78.00,55.00,40.00,23.00,9.00,-3.00,0.00,0.00,0.00,0.00,0.00,-2.00,-7.00,-18.00,-24.00,-24.00,-27.00,-35.00,-36.00,-32.00,-30.00,-38.00,-29.00,2.00,16.00,18.00,24.00,25.00,35.00,35.00,34.00,39.00,46.00,58.00,66.00,71.00,74.00,69.00,67.00,58.00,52.00,49.00,47.00,43.00,43.00,45.00,48.00,54.00,54.00,60.00,64.00,59.00,61.00,64.00,63.00,57.00,49.00,42.00,29.00,16.00,-5.00,0.00,0.00,0.00,0.00,0.00,-1.00,-12.00,-39.00,-48.00,-62.00,-73.00,-64.00,-63.00,-73.00,-78.00,-83.00,-88.00,-93.00,-93.00,-94.00,-98.00,-100.00,-102.00,-91.00,-90.00,-89.00,-87.00,-84.00,-71.00,-50.00,-18.00,5.00,10.00,11.00,15.00,10.00,2.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,0.00,2.00,0.00,0.00,0.00,20.00,59.00,95.00,121.00,134.00,138.00,138.00,148.00,168.00,183.00,198.00,211.00,214.00,209.00,203.00,206.00,214.00,228.00,243.00,243.00,237.00,240.00,233.00,213.00,201.00,195.00,184.00,177.00,172.00,172.00,184.00,201.00,215.00,230.00,231.00,226.00,223.00,211.00,212.00,209.00,211.00,205.00,211.00,211.00,210.00,213.00,198.00,203.00,209.00,205.00,203.00,210.00,217.00,224.00,227.00,225.00,230.00,227.00,230.00,224.00,225.00,230.00,239.00,241.00,235.00,238.00,217.00,187.00,171.00,158.00,139.00,125.00,117.00,109.00,102.00,96.00,78.00,36.00,4.00,0.00,0.00,0.00,0.00,0.00,1.00,7.00,23.00,44.00,75.00,96.00,113.00,127.00,136.00,138.00,140.00,143.00,139.00,142.00,148.00,163.00,176.00,191.00,204.00,209.00,204.00,201.00,196.00,177.00,155.00,130.00,141.00,143.00,145.00,156.00,161.00,166.00,175.00,187.00,196.00,202.00,191.00,194.00,198.00,193.00,206.00,219.00,223.00,232.00,232.00,227.00,223.00,212.00,208.00,209.00,196.00,189.00,177.00,178.00,176.00,176.00,172.00,173.00,184.00,193.00,198.00,211.00,207.00,207.00,210.00,212.00,206.00,211.00,216.00,220.00,219.00,208.00,210.00,213.00,206.00,200.00,204.00,201.00,206.00,207.00,208.00,221.00,221.00,229.00,238.00,237.00,233.00,233.00,237.00,243.00,259.00,260.00,259.00,261.00,256.00,259.00,261.00,260.00,249.00,249.00,242.00,241.00,240.00,242.00,233.00,222.00,214.00,201.00,183.00,154.00,145.00,145.00,141.00,145.00,145.00,148.00,138.00,144.00,147.00,148.00,153.00,155.00,160.00,168.00,175.00,183.00,200.00,218.00,225.00,240.00,248.00,248.00,242.00,237.00,231.00,227.00,225.00,229.00,234.00,231.00,239.00,239.00,239.00,236.00,238.00,233.00,229.00,222.00,214.00,222.00,217.00,222.00,233.00,236.00,239.00,237.00,231.00,219.00,219.00,215.00,214.00,210.00,198.00,181.00,160.00,117.00,90.00,72.00,65.00,67.00,62.00,62.00,64.00,63.00,58.00,53.00,55.00,45.00,40.00,42.00,50.00,46.00,57.00,58.00,57.00,69.00,77.00,83.00,95.00,99.00,110.00,114.00,119.00,120.00,124.00,127.00,131.00,130.00,129.00,127.00,124.00,124.00,125.00,139.00,149.00,157.00,165.00,168.00,178.00,189.00,198.00,200.00,196.00,196.00,200.00,193.00,179.00,179.00,174.00,167.00,166.00,167.00,179.00,192.00,199.00,197.00,216.00,222.00,214.00,207.00,198.00,188.00,191.00,193.00,177.00,162.00,152.00,143.00,133.00,126.00,124.00,122.00,130.00,124.00,123.00,96.00,31.00,24.00,36.00,45.00,65.00,90.00,118.00,139.00,143.00,151.00,172.00,177.00,180.00,182.00,189.00,194.00,204.00,209.00,215.00,219.00,215.00,212.00,203.00,203.00,207.00,212.00,216.00,220.00,217.00,209.00,208.00,211.00,217.00,228.00,242.00,263.00,282.00,298.00,308.00,316.00,313.00,296.00,274.00,261.00,257.00,255.00,259.00,268.00,286.00,292.00,290.00,280.00,273.00,269.00,268.00,268.00,260.00,256.00,246.00,246.00,246.00,234.00,232.00,239.00,249.00,248.00,263.00,273.00,282.00,282.00,271.00,281.00,283.00,274.00,260.00,253.00,234.00,196.00,156.00,120.00,85.00,43.00,8.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
//const PROGMEM float moteurD_ListeVitesse[] = {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,2.00,8.00,27.0,45.0,62.0,70.0,71.0,77.0,87.0,92.0,115.,128.,126.,115.,101.,105.,106.,103.,98.0,97.0,98.0,98.0,99.0,98.0,105.,115.,124.,125.,125.,136.,147.,137.,136.,146.,147.,148.,151.,149.,164.,174.,175.,180.,182.,180.,181.,192.,191.,192.,187.,179.,174.,172.,165.,159.,155.,144.,141.,135.,126.,118.,119.,118.,114.,110.,109.,117.,125.,128.,133.,145.,156.,171.,176.,177.,191.,194.,197.,194.,188.,189.,183.,181.,169.,166.,161.,162.,172.,178.,179.,181.,193.,195.,203.,201.,207.,211.,218.,221.,217.,207.,206.,209.,200.,189.,177.,172.,168.,156.,152.,152.,160.,156.,146.,146.,151.,154.,150.,142.,139.,141.,134.,140.,145.,139.,146.,154.,158.,161.,164.,164.,175.,178.,177.,175.,164.,148.,140.,148.,161.,174.,184.,189.,192.,190.,185.,179.,178.,165.,154.,156.,158.,160.,163.,162.,152.,144.,135.,131.,121.,111.,108.,111.,111.,115.,108.,110.,104.,100.,91.0,85.0,90.0,91.0,83.0,81.0,82.0,83.0,79.0,71.0,66.0,67.0,76.0,80.0,75.0,74.0,83.0,81.0,66.0,65.0,64.0,51.0,49.0,54.0,59.0,63.0,57.0,63.0,63.0,65.0,71.0,68.0,71.0,72.0,74.0,77.0,84.0,91.0,92.0,98.0,115.,128.,143.,145.,147.,145.,140.,139.,137.,144.,141.,142.,142.,142.,139.,142.,141.,142.,142.,130.,137.,148.,153.,159.,155.,157.,166.,171.,175.,184.,187.,188.,189.,183.,184.,183.,177.,164.,172.,180.,183.,185.,184.,198.,212.,213.,212.,200.,201.,197.,174.,150.,149.,143.,143.,137.,131.,125.,126.,126.,122.,117.,112.,113.,110.,114.,124.,129.,130.,136.,149.,167.,177.,187.,189.,204.,213.,222.,237.,257.,266.,273.,263.,262.,269.,269.,267.,260.,261.,261.,259.,255.,242.,248.,250.,245.,241.,251.,261.,269.,279.,275.,282.,280.,291.,285.,270.,256.,262.,264.,249.,255.,267.,280.,279.,281.,296.,308.,322.,325.,341.,344.,335.,312.,307.,300.,292.,286.,274.,264.,260.,259.,266.,277.,268.,268.,272.,262.,257.,252.,253.,243.,233.,233.,223.,227.,214.,200.,203.,212.,213.,213.,209.,210.,222.,220.,232.,238.,235.,235.,245.,252.,249.,252.,240.,226.,213.,199.,187.,177.,184.,176.,171.,174.,173.,167.,157.,143.,134.,140.,141.,136.,132.,128.,118.,116.,107.,95.0,97.0,104.,99.0,106.,114.,114.,117.,123.,131.,142.,143.,137.,128.,126.,127.,136.,139.,130.,130.,133.,141.,148.,139.,137.,138.,140.,137.,142.,149.,150.,153.,153.,163.,167.,172.,175.,170.,167.,166.,163.,150.,148.,144.,121.,84.0,67.0,36.0,-5.0,0.00,0.00,0.00,0.00,1.00,1.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,1.00,9.00,31.0,59.0,91.0,106.,117.,125.,132.,135.,130.,127.,117.,120.,127.,129.,127.,111.,100.,92.0,87.0,84.0,87.0,97.0,106.,113.,118.,123.,123.,131.,138.,138.,135.,140.,144.,119.,100.,62.0,31.0,9.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,-1.0,-8.0,0.00,0.00,1.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,2.00,6.00,32.0,59.0,88.0,102.,115.,123.,140.,158.,173.,185.,197.,200.,204.,202.,206.,201.,191.,184.,173.,164.,157.,156.,157.,160.,165.,167.,180.,194.,196.,199.,203.,216.,229.,240.,242.,248.,251.,259.,258.,256.,261.,259.,267.,279.,288.,286.,279.,274.,273.,276.,267.,260.,249.,251.,251.,260.,272.,270.,288.,293.,294.,287.,286.,275.,260.,253.,238.,228.,211.,195.,180.,170.,162.,145.,123.,111.,117.,123.,122.,118.,129.,124.,131.,129.,123.,118.,104.,83.0,72.0,63.0,54.0,46.0,38.0,33.0,19.0,0.00,-1.0,1.00,0.00,0.00,0.00,0.00,1.00,0.00,0.00,2.00,28.0,42.0,49.0,58.0,66.0,70.0,69.0,74.0,84.0,91.0,92.0,95.0,115.,118.,113.,121.,136.,139.,138.,141.,147.,150.,150.,148.,145.,144.,145.,150.,151.,159.,168.,178.,180.,186.,180.,179.,183.,184.,190.,196.,201.,211.,218.,221.,225.,217.,204.,203.,202.,203.,193.,194.,193.,186.,185.,189.,204.,206.,202.,200.,199.,197.,192.,199.,193.,199.,205.,206.,213.,209.,217.,221.,205.,200.,204.,203.,211.,213.,207.,222.,230.,233.,240.,245.,246.,245.,248.,250.,263.,257.,251.,249.,236.,232.,232.,248.,247.,253.,259.,261.,265.,271.,276.,273.,275.,273.,282.,280.,282.,277.,276.,275.,259.,249.,230.,221.,201.,182.,182.,192.,204.,201.,211.,217.,235.,256.,267.,277.,280.,290.,278.,269.,257.,249.,242.,238.,243.,230.,232.,232.,234.,231.,229.,229.,231.,229.,224.,232.,232.,240.,241.,247.,260.,261.,253.,246.,260.,267.,257.,259.,255.,245.,246.,240.,241.,246.,235.,233.,231.,225.,217.,210.,198.,187.,184.,179.,180.,177.,169.,144.,119.,119.,130.,150.,158.,159.,160.,161.,164.,156.,159.,157.,163.,153.,152.,152.,152.,150.,151.,153.,150.,156.,165.,170.,171.,163.,169.,169.,170.,173.,171.,174.,176.,181.,174.,178.,168.,166.,179.,186.,197.,205.,208.,199.,201.,193.,178.,183.,172.,141.,114.,105.,94.0,79.0,65.0,51.0,36.0,30.0,18.0,2.00,0.00,0.00,2.00,0.00,4.00,7.00,-1.0,1.00,0.00,0.00,1.00,2.00,9.00,25.0,31.0,37.0,46.0,57.0,67.0,76.0,84.0,95.0,106.,110.,116.,130.,150.,172.,199.,210.,203.,198.,203.,208.,223.,235.,234.,237.,239.,231.,237.,245.,245.,259.,276.,285.,300.,307.,309.,303.,295.,282.,274.,272.,273.,267.,274.,277.,269.,256.,246.,249.,253.,261.,271.,281.,296.,298.,304.,305.,317.,312.,296.,280.,259.,245.,234.,217.,205.,194.,177.,150.,125.,105.,83.0,52.0,20.0,8.00,-4.0,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};

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

