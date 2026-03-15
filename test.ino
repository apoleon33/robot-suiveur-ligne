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


/// ENCODEURS
// Encodeur en QUADRATURE : on exploite les 4 transitions (A+ A- B+ B-)
// Ratio réducteur = 53, résolution brute du disque = 3 fentes
// donc  53 * 3 * 4 = 636 ticks par tour de roue
#define TICKS_PAR_TOUR 636.0

const float dt = 0.01f;    // Période d'échantillonnage (10 ms)

// Compteurs de ticks qui est volatile car modifiés dans les ISRs
volatile long ticks_mot1 = 0;
volatile long ticks_mot2 = 0;
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

  // Interruptions sur les 4 canaux (CHANGE = front montant ET descendant)
  attachInterrupt(digitalPinToInterrupt(ENC1_A), ISR_ENC1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_B), ISR_ENC1_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), ISR_ENC2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_B), ISR_ENC2_B, CHANGE);

  Serial.begin(9600);
  Serial.println("Robot Suiveur de Ligne - lancement tests");

}

void loop() {
  // DÉCOMMENTER LES FONCTION QU'ON VEUT TESTER (PAS EN MÊME TEMPS CE SERAIT STUPIDE)
  // Test 1: les moteurs tournent et on récupère leurs vitesse a partir des encodeurs
  moveForward();
  // turnLeft();
  // turnRight();
  // stopMotors();
  odometrie();


  // Test 2: detecteur d'obstacle
  detectionObstacle();

  // Test 3: suivi de ligne
  suiviLigne();
  

  delay(100); // on calme le jeux
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



void odometrie(){
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

  Serial.print("vitesse de moteur 1: ");
  Serial.print(vit_mesuree_mot1);
  Serial.println(" tr/s.");

  Serial.print("vitesse de moteur 2: ");
  Serial.print(vit_mesuree_mot2);
  Serial.println(" tr/s.");
}

void detectionObstacle(){
  bool obstacleDetected = (digitalRead(OBSTACLE_SENSOR_PIN) == LOW);

  Serial.println(obstacleDetected? "Obstacle détecté": "Pas d'obstacle détecté");
}

void suiviLigne(){
  // Lecture des capteurs de ligne
  int leftSensor = digitalRead(LEFT_SENSOR_PIN);
  int rightSensor = digitalRead(RIGHT_SENSOR_PIN);

  Serial.print("Capteurs: Gauche=");
  Serial.print(leftSensor);
  Serial.print(" Droit=");
  Serial.println(rightSensor);
  
}
