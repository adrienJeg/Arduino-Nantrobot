#include <Arduino.h>

// --- Time ---
long currentTime, previousTime;
float sampleTime = 0.02; // period of the main loop, in seconds (here, frequency of 100 Hz)

// --- Moteurs ---
// Définition PIN moteurs
#define PIN_MOTOR_R_DIR 7
#define PIN_MOTOR_L_DIR 8
#define PIN_MOTOR_R_PWM 9
#define PIN_MOTOR_L_PWM 10
#define PIN_CODEUR_D_VOIE_A 19
#define PIN_CODEUR_D_VOIE_B 18
#define PIN_CODEUR_G_VOIE_A 3
#define PIN_CODEUR_G_VOIE_B 2

#define motorDriverEnablingPin 4

// Consignes envoyées aux moteurs 
int pwmMotorR = 0;
signed int refSensRotationGauche = -1;
int pwmMotorL = 0;
signed int refSensRotationDroite = 1;

// --- Robot ---
// Caractéristiques
const float rRoue = 0.04; // Rayon de la roue (en m)
const float eRoues = 0.23; // Ecart entre les roues (en m)

// --- Position et orientation actuelles du robot ---
// Pose du rrobot (après fusion des données)
float xR = 0.;
float yR = 0.;
float orientationR = 0.;
// Avec les codeurs
float xCodeur = 0.;
float yCodeur = 0.;
float orientationCodeur = 0.;
// Avec la caméra
float xCam = 0.;
float yCam = 0.;
float orientationCam = 0.;
// --- Nouvelle position et nouvelle orientation --- 
// Avec les codeurs
float newXCodeur = 0.;
float newYCodeur = 0.;
float newOrientationCodeur = 0.;
// Avec la caméra
float newXCam = 0.;
float newYCam = 0.;
float newOrientationCam = 0.;
// Distance parcourue par les roues 
float distGauche;
float distDroit;
// Distance entre le robot et la cible
float distanceCible = 0.; // Distance réelle
float distanceMini = 0.02; // Distance acceptable - à adapter 
// Angle entre le robot et la cible
float consigneOrientation = 0.;

// Savoir si cible est à gauche ou à droite du robot.
int signe = 1;

// --- Codeurs ---
volatile int indiceCodeurGauche, indiceCodeurDroit, valueGauche, valueDroit;
volatile signed long positionCodeurGauche = 0, positionCodeurDroit = 0;
volatile signed long oldPositionCodeurGauche = 0, oldPositionCodeurDroit = 0; 
volatile signed long transitionsCodeur[] = {0, -1, +1, 0, +1, 0, 0, -1, -1, 0, 0, +1, 0, +1, -1, 0};
// Variations de position
const float coeffLong = 2.0*PI*rRoue/1632.0; // Coeficient pour détermination de variations de position
float dDist = 0.; // Variation de position en distance
float dAngl = 0.; // Variation de position en angle

// --- Communication ---
String tmp = ""; // Message reçu
char param; 
char key; // Distinction des cas 
int indice = 1; // Indice de lecture du message

// --- Asservissement ---
// Coefficients du PD 
float coeffP = 50.0;
float coeffI = 20.0;
float coeffD = 5.0;
// Variables d'asservissement du robot
float erreurAngle = 0.;
float erreurAnglePrec = 0.;
float sommeErreur = 0.;
float deltaErreur = 0.;
int deltaCommande = 0; // Commande envoyée aux moteurs pour prise en compte de l'angle

// --- Environnement ---
// Position de la cible à atteindre - à adapter 
float xC = 0.0;
float yC = 0.0;
float tabXC[] = {0.0, 0.0, 0.5, 0.5, 0.0}; // en m
float tabYC[] = {0.0, 0.5, 0.5, 0.0, 0.0}; // en m
int indicePosCourante = 0;
bool aTermine = false;

// --- Affichage ---
bool printValue = false;
int cptAffichage = 0;

//// --- ROS ---
//ros::NodeHandle nh;
//
//std_msgs::String str_msg;
//ros::Publisher debugNode("debugNode", &str_msg);




float v0 = 1.0; // en m/s non pas du tout
float vr, vl; // en m/s
float omega; // en rad/s

float L_updatePWM=0, R_updatePWM=0;

// ---------- Fin déclaration Variables ----------- 


// ---------- Fonctions utilisées ----------

void updateVelocities(float v, float omega)
{
  vr = (2*v + omega * eRoues) / (2.0 * rRoue);
  vl = (2*v - omega * eRoues) / (2.0 * rRoue);
}


void setMotor(int command, int DIRpin, int PWMpin, signed int refSensRotation)
{
  command = constrain(command, -250, 250); // to make sure we don't overflow
  command = command * refSensRotation; // différent selon moteur droite et gauche
  if (command > 0) 
  {
    digitalWrite(DIRpin, HIGH); // Set the rotation direction 
  }
  if (command <= 0) 
  {
    digitalWrite(DIRpin, LOW); // Set the rotation direction 
  }
  analogWrite(PWMpin, abs(command));   // Send command to motor driver pwm
}


void newPoseRobot()
{
  // Actualiser la position du robot en distance et angle à partir des infos des codeurs 
  distGauche = coeffLong*(positionCodeurGauche - oldPositionCodeurGauche); // à vérifier 
  distDroit = coeffLong*(positionCodeurDroit - oldPositionCodeurDroit);
  
  // Détermination des variations de position en distance et angle
  dDist = (distDroit + distGauche)/2;
  dAngl = (distDroit - distGauche)/eRoues;
  
  // Détermination de la nouvelle pose du robot
  newXCodeur = xR + dDist*cos(orientationR + dAngl/2);
  newYCodeur = yR + dDist*sin(orientationR + dAngl/2);
  newOrientationCodeur = orientationR + dAngl;
  
  // Actualisation de la pose du robot
  xCodeur = newXCodeur;
  yCodeur = newYCodeur;
  orientationCodeur = newOrientationCodeur;
  
  // Sauvegarde des valeurs de PositionCodeurGauche et PositionCodeurDroit au tour précédent
  oldPositionCodeurGauche = positionCodeurGauche;
  oldPositionCodeurDroit = positionCodeurDroit;
}



// --------------------------- Fin déclaration Fonctions ----------------------------


void setup()
{
//  // Initialisation Serial pour affichage 
  Serial.begin(115200);
  Serial.println("Coucou");
// --- Moteurs ---l
  // Initialisation PIN moteurs
  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_DIR, OUTPUT);
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_DIR, OUTPUT);
  // Initialisation du sens de rotation des moteurs 
  digitalWrite(PIN_MOTOR_R_DIR, HIGH);
  digitalWrite(PIN_MOTOR_L_DIR, LOW);

  // Set PWM frequency to 3921.16 Hz
  setPwmFrequency(PIN_MOTOR_R_PWM, 8); 
  setPwmFrequency(PIN_MOTOR_L_PWM, 8);

  // We need to set that pin in high to enable the output of the motor driver
  pinMode(motorDriverEnablingPin, OUTPUT);  
  digitalWrite(motorDriverEnablingPin, HIGH);

  // Interruption des codeurs
  // Codeur droit
  attachInterrupt(digitalPinToInterrupt(PIN_CODEUR_D_VOIE_B), ISRCodeurDroit, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CODEUR_D_VOIE_A), ISRCodeurDroit, CHANGE);
  indiceCodeurDroit=(digitalRead(PIN_CODEUR_D_VOIE_B) << 1 | digitalRead(PIN_CODEUR_D_VOIE_A));

  // Codeur gauche
  attachInterrupt(digitalPinToInterrupt(PIN_CODEUR_G_VOIE_A), ISRCodeurGauche, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CODEUR_G_VOIE_B), ISRCodeurGauche, CHANGE);
  indiceCodeurGauche=(digitalRead(PIN_CODEUR_G_VOIE_B) << 1 | digitalRead(PIN_CODEUR_G_VOIE_A));
  
  currentTime = micros();
}


void loop()
{
  previousTime = currentTime;
  newPoseRobot();

  // --- Fusion des données --- // sans les infos caméra pour le moment - à modifier
  xR = xCodeur;
  yR = yCodeur;
  orientationR = orientationCodeur;

	// Calcul de la nouvelle distance séparant le robot de sa cible
	distanceCible = sqrt((xC-xR) * (xC-xR) + (yC-yR) * (yC-yR));

  
  // Si elle est assez faible alors acceptable - à modifier 
  if(distanceCible <= distanceMini)
  {
    indicePosCourante++;
    //indicePosCourante = indicePosCourante % 4; // On parcourt les 4 mêmes positions
    
    
    //Serial.print("Youpi !");
    if (indicePosCourante == 5) // on a fait un tour complet
    {
      aTermine = true;
    }

    xC = tabXC[indicePosCourante];
    yC = tabYC[indicePosCourante];
    
  }

  consigneOrientation = atan2(yC-yR, xC-xR); // - orientationR
  consigneOrientation = atan2(sin(consigneOrientation), cos(consigneOrientation));

  Serial.print("\t consOri ");
  Serial.print(consigneOrientation);
  Serial.print("\t xC ");
  Serial.print(xC);
  Serial.print("\t yC ");
  Serial.print(yC);
  Serial.print("\t xR ");
  Serial.print(xR);
  Serial.print("\t yR ");
  Serial.print(yR);
  Serial.print("\t theta ");
  Serial.print(orientationR);


// --- Asservissement ---

  // Calcul de l'erreur sur l'angle
	erreurAngle = consigneOrientation - orientationR;
									   
	//Calcul de la différence entre l'erreur au coup précédent et l'erreur actuelle.
	deltaErreur = erreurAngle - erreurAnglePrec;

	// Sauvegarde de la valeur actuelle de l'erreur
	erreurAnglePrec = erreurAngle;

  // Calcul du terme intégral
  sommeErreur = sommeErreur + erreurAngle * sampleTime;
  
	// Calcul du PID
  omega = round(coeffP * erreurAngle + coeffD * deltaErreur + coeffI * sommeErreur);

  //deltaCommande = deltaCommande * distanceCible * 3.0; // On accorde plus de poids à l'orientation quand on est loin
  Serial.print("\t omega ");
  Serial.print(omega);

// --- Commandes envoyées aux moteurs ---  
  updateVelocities(v0, omega);//  L_updatePWM = vl * 5.5;
//  R_updatePWM = vr * 5.5;
//
//  // On bride l'accélération maximale pour ne pas que les roues patinent
//  L_updatePWM = constrain(L_updatePWM, -20, 20); 
//  R_updatePWM = constrain(R_updatePWM, -20, 20);
//  
//  pwmMotorL = pwmMotorL + L_updatePWM;
//  pwmMotorR = pwmMotorR + R_updatePWM;

  pwmMotorL = vl * 5.5;
  pwmMotorR = vr * 5.5;
  Serial.print("\t distC ");
  Serial.print(distanceCible);

	// Recadre une première fois les valeurs de PWM 
  pwmMotorL = constrain(pwmMotorL, -70, 70);
  pwmMotorR = constrain(pwmMotorR, -70, 70);

  Serial.print("\t pwmL ");
  Serial.print(pwmMotorL);
  Serial.print("\t pwmR ");
  Serial.println(pwmMotorR);

  // Envoi des commandes aux moteurs :
  if (!aTermine)
  {
    setMotor(pwmMotorR, PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, refSensRotationDroite);
    setMotor(pwmMotorL, PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, refSensRotationGauche);
  }
  else
  {
    setMotor(0, PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, refSensRotationDroite);
    setMotor(0, PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, refSensRotationGauche);
  }

 // --- Time ---
  do
  {
    currentTime = micros(); // Temps absolu depuis le demarrage de la carte (en μs)
  } 
  while (currentTime - previousTime < (sampleTime * 1000000)) ;
  
  // On fait clignoter la led
  digitalWrite(13, LOW);
}

  
