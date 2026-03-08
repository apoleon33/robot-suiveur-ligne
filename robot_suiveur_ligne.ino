// Robot Suiveur de Ligne - Arduino Mega
// Basé sur l'algorithme algo.md et la nomenclature composants.pdf

// propriete du robot (en m)
#define ROBOT_L 0.2   // entraxe des roues
#define ROBOT_r 0.07  // diametre d'une roue
#define PPR 11        // ticks de l'encodeur (je sort cette valeur parce que j'ai pas mieux et c generalement celle la)

// Encodeur en QUADRATURE : on exploite les 4 transitions (A+ A- B+ B-)
// Ratio réducteur = 53, résolution brute du disque = 3 fentes
// donc  53 * 3 * 4 = 636 ticks par tour de roue
const float TICKS_PAR_TOUR = 636.0f;

// ==========================================================================
// PARAMETRES DU CORRECTEUR PI
// On a Tr ≈ 0,3 s et les dépassement sont ≤ 10 %
// Modèle moteur  : F(s) = K / (1 + τ·s)  avec K = 0,011 et τ = 0,25 s
// ==========================================================================

const float Kp = 607.3f;
const float Ki = 15331.4f;
const float dt = 0.01f;    // Période d'échantillonnage (10 ms)

// Configuration des broches
const int LEFT_SENSOR_PIN = 50;
const int RIGHT_SENSOR_PIN = 52;
const int LEFT_MOTOR_PIN = 10;
const int RIGHT_MOTOR_PIN = 11;
const int LEFT_ENCODER_PIN = 30; // a verifier donc prions
#define ENC1_A  18   // INT5
#define ENC1_B  19   // INT4
#define ENC2_A  20   // INT3
#define ENC2_B  21   // INT2
const int RIGHT_ENCODER_PIN = 31;
const int START_BUTTON_PIN = 2;  // À confirmer
const int OBSTACLE_SENSOR_PIN = A0;  // Capteur IR obstacle

// Driver moteur PmodHB5 
#define PWM_MOT1  10
#define DIR_MOT1  30
#define PWM_MOT2  11
#define DIR_MOT2  31


// Variables d'état
bool started = false;
bool obstacleDetected = false;

struct q {
  float v;
  float omega;
}; // Format que renvoie la lecture des encodeurs

// Variables associées aux encodeurs et aux interruptions
#define TIMEOUT 100000 // si valeur superieur, roues a l'arret (0.1s)
volatile unsigned long dt_L = 0;
volatile unsigned long lastTimeL = 0;
volatile int sens_l = 1;

volatile unsigned long dt_R = 0;
volatile unsigned long lastTimeR = 0;
volatile int sens_R = 1;

// Compteurs de ticks qui est volatile car modifiés dans les ISRs
volatile long ticks_mot1 = 0;
volatile long ticks_mot2 = 0;

// Termes correcteurs PI
float integral_mot1 = 0.0f;
float integral_mot2 = 0.0f;

// Horodatage boucle temps réel
unsigned long previousMillis = 0;

// =========================================================================
// ROUTINES D'INTERRUPTION (ISRs) — Décodage en quadrature complète
// En comparant l'état de A et de B à chaque front, on détermine le sens de
// rotation et on incrémente ou décrémente le compteur.
// 4 ISRs (2 par encodeur) => résolution maximale.
// =========================================================================

// Encodeur 1
void ISR_ENC1_A() {
  // Front sur A : si A == B => sens +, sinon sens -
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) ticks_mot1++;
  else ticks_mot1--;
}

void ISR_ENC1_B() {
  // Front sur B : si A != B => sens +, sinon sens -
  if (digitalRead(ENC1_A) != digitalRead(ENC1_B)) ticks_mot1++;
  else ticks_mot1--;
}

// Encodeur 2
void ISR_ENC2_A() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) ticks_mot2++;
  else ticks_mot2--;
}

void ISR_ENC2_B() {
  if (digitalRead(ENC2_A) != digitalRead(ENC2_B)) ticks_mot2++;
  else ticks_mot2--;
}

void setup() {
  // Initialisation des broches
  pinMode(LEFT_SENSOR_PIN, INPUT);
  pinMode(RIGHT_SENSOR_PIN, INPUT);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(OBSTACLE_SENSOR_PIN, INPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);

  // Arrêt initial des moteurs
  digitalWrite(LEFT_MOTOR_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_PIN, LOW);

  // setup des interruptions pour l'encodeur
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), handleLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), handleRightEncoder, RISING);

  // Interruptions sur les 4 canaux (CHANGE = front montant ET descendant)
  attachInterrupt(digitalPinToInterrupt(ENC1_A), ISR_ENC1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), ISR_ENC1_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), ISR_ENC2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), ISR_ENC2_B, CHANGE);

  Serial.begin(9600);
  Serial.println("Robot Suiveur de Ligne - Initialisation");
}

void loop() {


  // Vérification du bouton start
  if (!started && digitalRead(START_BUTTON_PIN) == LOW) {
    started = true;
    Serial.println("Début du suivi de ligne...");
    delay(500);  // Anti-rebond
  }

  if (!started) {
    return;  // Attendre le bouton start
  }

  // Détection d'obstacle
  obstacleDetected = (digitalRead(OBSTACLE_SENSOR_PIN) == LOW);

  if (obstacleDetected) {
    stopMotors();
    Serial.println("Obstacle détecté - Arrêt");
    delay(100);
    return;
  }

  // Lecture des capteurs de ligne
  int leftSensor = digitalRead(LEFT_SENSOR_PIN);
  int rightSensor = digitalRead(RIGHT_SENSOR_PIN);

  Serial.print("Capteurs: Gauche=");
  Serial.print(leftSensor);
  Serial.print(" Droit=");
  Serial.println(rightSensor);

  // Logique de suivi de ligne
  if (leftSensor == LOW && rightSensor == LOW) {
    // Les deux capteurs sur la ligne noire - Arrêt
    stopMotors();
    Serial.println("Arrêt - Les deux capteurs sur la ligne");
  } else if (leftSensor == LOW) {
    // Capteur gauche sur la ligne - Tourner à gauche
    turnLeft();
    Serial.println("Tourner à gauche");
  } else if (rightSensor == LOW) {
    // Capteur droit sur la ligne - Tourner à droite
    turnRight();
    Serial.println("Tourner à droite");
  } else {
    // Les deux capteurs sur le blanc - Avancer tout droit
    moveForward();
    Serial.println("Avancer tout droit");
  }

  float V_robot_voulu     = 0.5f;   // Vitesse linéaire désirée (m/s)
  float Omega_robot_voulu = 0.0f;   // Vitesse angulaire désirée (rad/s)

  // =========================================================================
  // PARTIE DU G4 : Cinématique inverse + Régulation PI
  // =========================================================================


  // Modèle Cinématique Inverse
  // Traduit (V, Omega) en vitesses angulaires de roue (rad/s), puis en RPS
  // ω_droite = (V + L/2 · Ω) / R
  // ω_gauche = (V - L/2 · Ω) / R
  float consigne_RPS_mot1 = ((V_robot_voulu + (ROBOT_L / 2.0f) * Omega_robot_voulu) / ROBOT_r) / (2.0f * PI);
  float consigne_RPS_mot2 = ((V_robot_voulu - (ROBOT_L / 2.0f) * Omega_robot_voulu) / ROBOT_r) / (2.0f * PI);

  // Lecture atomique des compteurs d'encodeurs
  // noInterrupts() garantit qu'aucune ISR ne modifie les compteurs
  // entre la lecture et la remise à zéro (évite la perte de ticks !).
  noInterrupts();
  long delta_ticks1 = ticks_mot1;
  long delta_ticks2 = ticks_mot2;
  ticks_mot1 = 0;
  ticks_mot2 = 0;
  interrupts();

  // Conversion ticks en tr/s
  float vit_mesuree_mot1 = ((float)delta_ticks1 / TICKS_PAR_TOUR) / dt;
  float vit_mesuree_mot2 = ((float)delta_ticks2 / TICKS_PAR_TOUR) / dt;

  // Correcteur PI (avec saturation et anti-windup)
  int pwm_mot1 = computePID(consigne_RPS_mot1, vit_mesuree_mot1, integral_mot1);
  int pwm_mot2 = computePID(consigne_RPS_mot2, vit_mesuree_mot2, integral_mot2);

  // Envoi des commandes aux drivers
  setMotorPower(PWM_MOT1, DIR_MOT1, pwm_mot1);
  setMotorPower(PWM_MOT2, DIR_MOT2, pwm_mot2);

  delay(50);  // Petit délai pour stabilité
}

// Fonctions de contrôle des moteurs
void moveForward() {
  analogWrite(LEFT_MOTOR_PIN, 200);  // PWM pour vitesse
  analogWrite(RIGHT_MOTOR_PIN, 200);
}

void turnLeft() {
  analogWrite(LEFT_MOTOR_PIN, 100);  // Ralentir le moteur gauche
  analogWrite(RIGHT_MOTOR_PIN, 200);
}

void turnRight() {
  analogWrite(LEFT_MOTOR_PIN, 200);
  analogWrite(RIGHT_MOTOR_PIN, 100);  // Ralentir le moteur droit
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_PIN, LOW);
}

// =========================================================================
// FONCTIONS DE LA COMMANDE
// =========================================================================

/**
   Correcteur PI discret avec Saturation PWM et Anti-Windup (Clamping)

   @param consigne_RPS       Vitesse de consigne en tours/seconde
   @param vitesse_mesuree_RPS Vitesse mesurée par l'encodeur en tours/seconde
   @param integral           Référence vers l'accumulateur intégral du moteur
   @return                   Commande PWM dans [-255 ; 255]

   Correction du bug d'origine : l'intégrale candidate est calculée UNE SEULE
   fois, puis acceptée ou rejetée selon la logique anti-windup, sans double
   accumulation possible.
*/
int computePID(float consigne_RPS, float vitesse_mesuree_RPS, float &integral) {

  // Erreur de vitesse
  float erreur = consigne_RPS - vitesse_mesuree_RPS;

  // Intégrale candidate (si pas de saturation, on l'acceptera)
  float integral_candidate = integral + Ki * erreur * dt;

  // Commande théorique (linéaire, sans limite)
  float commande_theorique = Kp * erreur + integral_candidate;

  // Saturation matérielle + Anti-Windup par Clamping
  int commande_pwm;

  if (commande_theorique > 255.0f) {
    commande_pwm = 255;
    // Anti-windup : on n'accepte la nouvelle intégrale que si l'erreur
    // est négative (ce qui tendrait à réduire la commande => désaturer)
    if (erreur < 0.0f) integral = integral_candidate;

  } else if (commande_theorique < -255.0f) {
    commande_pwm = -255;
    // Saturation basse : on accepte seulement si l'erreur est positive
    if (erreur > 0.0f) integral = integral_candidate;

  } else {
    // Pas de saturation : fonctionnement linéaire normal
    commande_pwm = (int)commande_theorique;
    integral = integral_candidate;
  }

  return commande_pwm;
}


// Envoie de consignes de vitesses au robot
void moveRobot(float Wl, float Wr) {
  // A CHANGER
  digitalWrite(LEFT_MOTOR_PIN, LOW);
  digitalWrite(RIGHT_MOTOR_PIN, LOW);
}


// Convertir des donnees de la forme v, omega en vitesses de moteur droit/gauche
void commandToSpeed(float v, float omega) {
  float w_L = (2 * v - ROBOT_L * omega) / (2 * ROBOT_r);
  float w_R = (2 * v + ROBOT_L * omega) / (2 * ROBOT_r);
  moveRobot(w_L, w_R);
}

// Conversion inverse -> vitesse des roues into v, omega (a noter qu'on a pas le sens du robot genre avant/arriere, a deduire de la commande)
q GetQFromWheel() {
  // Calcul de la vitesse des deux roues
  float vL, vR;
  if (dt_L > TIMEOUT) {
    vL = 0.0;
  } else {
    vL = (2.0 * PI * ROBOT_r * 1000000.0) / (PPR * (float)dt_L);
  }

  if (dt_R > TIMEOUT) {
    vR = 0.0;
  } else {
    vR = (2.0 * PI * ROBOT_r * 1000000.0) / (PPR * (float)dt_R);
  }

  // Calcul de la vitesse lineaire et angulair globale du robot
  float Vrobot = (vL + vR) / 2;
  float Omegarobot = (vR - vL) / ROBOT_L;

  return q{Vrobot, Omegarobot};
}



// fonctions qui actualisent a chaque interruptions le delai entre chaques ticks d'encodeurs (pour déduire la vitesse des roues)
void handleLeftEncoder() {
  unsigned long current_time = micros();
  dt_L = current_time - lastTimeL; // Temps écoulé depuis le dernier tick
  lastTimeL = current_time;
}

void handleRightEncoder() {
  unsigned long current_time = micros();
  dt_R = current_time - lastTimeR; // Temps écoulé depuis le dernier tick
  lastTimeR = current_time;
}

/**
   Envoi de la consigne de puissance au driver moteur PmodHB5
   Traduit la sortie du PID [-255 ; 255] en signal PWM + direction.
*/
void setMotorPower(int pwm_pin, int dir_pin, int commande_pid) {
  if (commande_pid >= 0) {
    digitalWrite(dir_pin, HIGH);           // Sens positif donc en avant
    analogWrite(pwm_pin, commande_pid);
  } else {
    digitalWrite(dir_pin, LOW);            // Sens négatif donc en arrière
    analogWrite(pwm_pin, abs(commande_pid));
  }
}
