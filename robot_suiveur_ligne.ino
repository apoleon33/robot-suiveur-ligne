// Robot Suiveur de Ligne - Arduino Mega
// Basé sur l'algorithme algo.md et la nomenclature composants.pdf

// propriete du robot (en m)
#define ROBOT_L 0.2   // entraxe des roues
#define ROBOT_r 0.07  // diametre d'une roue

// Encodeur en QUADRATURE : on exploite les 4 transitions (A+ A- B+ B-)
// Ratio réducteur = 53, résolution brute du disque = 3 fentes
// donc  53 * 3 * 4 = 636 ticks par tour de roue
#define TICKS_PAR_TOUR 636.0

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
const int LEFT_MOTOR_PIN = 10; // PWM_MOT1
const int RIGHT_MOTOR_PIN = 11; // PWM_MOT2
const int DIR_MOT1 = 30; // sens de direction des roues
const int DIR_MOT2 = 31;
const int ENC1_A = 18;   // INT5
const int ENC1_B = 19;  // INT4
const int ENC2_A = 20; // INT3
const int ENC2_B = 21;   // INT2
const int START_BUTTON_PIN = 2;  // À confirmer
const int OBSTACLE_SENSOR_PIN = A0;  // Capteur IR obstacle

// variables commande boucle fermee
const float k_rho = 0.6f;     // Doit être > 0
const float k_alpha = 1.5f;   // Doit être > 0
const float k_beta = -0.6f;   // Doit être < 0

// Position du robot (Odométrie)
float x = 0.0f;
float y = 0.0f;
float theta = 0.0f;

// Cible à atteindre (x_p, y_p, theta_p)
float xp = 1.0f;
float yp = 1.0f;
float thetap = 0.0f;


// Variables d'état
bool started = false;
bool obstacleDetected = false;

// Termes correcteurs PI
float integral_mot1 = 0.0f;
float integral_mot2 = 0.0f;

// Horodatage boucle temps réel
unsigned long previousMillis = 0;

// Compteurs de ticks qui est volatile car modifiés dans les ISRs
volatile long ticks_mot1 = 0;
volatile long ticks_mot2 = 0;

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

float wrap_to_pi(float angle) {
  while (angle > PI) angle -= 2.0 * PI;
  while (angle <= -PI) angle += 2.0 * PI;
  return angle;
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

  // Lecture des capteurs de ligne
  int leftSensor = digitalRead(LEFT_SENSOR_PIN);
  int rightSensor = digitalRead(RIGHT_SENSOR_PIN);  
  
  // Lecture des encodeurs de manière sécurisée
  noInterrupts();
  long delta_ticks1 = ticks_mot1; // Moteur Droit
  long delta_ticks2 = ticks_mot2; // Moteur Gauche
  ticks_mot1 = 0;
  ticks_mot2 = 0;
  interrupts();
  
  
  // odometrie
  float dist_droit = ((float)delta_ticks1 / TICKS_PAR_TOUR) * (PI * ROBOT_r);
  float dist_gauche = ((float)delta_ticks2 / TICKS_PAR_TOUR) * (PI * ROBOT_r);

  float d_center = (dist_droit + dist_gauche) / 2.0f;
  float d_theta = (dist_droit - dist_gauche) / ROBOT_L;

  x += d_center * cos(theta + d_theta / 2.0f);
  y += d_center * sin(theta + d_theta / 2.0f);
  theta = wrap_to_pi(theta + d_theta);

  // commande en boucle fermée (implementation du TP ROS2)
  float dx = xp - x;
  float dy = yp - y;
  float rho = sqrt(dx * dx + dy * dy);
  
  float V_robot_voulu = 0.0f;
  float Omega_robot_voulu = 0.0f;

  const float rho_stop = 0.05f; // Rayon d'arrêt autour de la cible

  if (rho > rho_stop) {
    float angle_to_goal = atan2(dy, dx);
    float alpha = wrap_to_pi(angle_to_goal - theta);
    float beta = wrap_to_pi(thetap - theta - alpha);

    V_robot_voulu = k_rho * rho;
    Omega_robot_voulu = k_alpha * alpha + k_beta * beta;

    // Saturation des vitesses de commande (Sécurité)
    if (V_robot_voulu > 0.4f) V_robot_voulu = 0.4f;
    if (V_robot_voulu < -0.4f) V_robot_voulu = -0.4f;
  } 
  

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
  delta_ticks1 = ticks_mot1;
  delta_ticks2 = ticks_mot2;
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
  setMotorPower(LEFT_MOTOR_PIN, DIR_MOT1, pwm_mot1);
  setMotorPower(RIGHT_MOTOR_PIN, DIR_MOT2, pwm_mot2);

  delay(50);  // Petit délai pour stabilité
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
