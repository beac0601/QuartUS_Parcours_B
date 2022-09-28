#include <Arduino.h>
#include <LibRobus.h>

#define Droite = 0
#define Gauche = 1

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  float distance, sp, cycle, vitesse, pulsecycle, correction, distanceActuelle, distanceAncienne, erreurVitesse ;
  float vitesseActuelle, vitesseDesiree, SommeErreur, Moteur, Kp, Ki; 
If (distance == 0)
  {
    cycle = 0.5 ;
    sp = 1600 ;
    SommeErreur = 0 ;
//Vitesse transmise
    pulsecycle = 3200 * 0.5 ;
    vitesse = (sp + correction)*cycle ;
    MOTOR_SetSpeed = vitesse ;
//Calcul de vitesse
    vitesse = distanceActuelle - distanceAncienne ;
//Calcul erreu de vitesse
    erreurVitesse = vitesseDesiree - vitesseActuelle ;
//Calcul somme d'erreur
    SommeErreur = erreurVitesse + SommeErreur ;
//Correction de vitesse
    distanceActuelle = int32_t ENCODER_Read(Droit)



  }


}