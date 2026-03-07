// Robot Suiveur de Ligne - Arduino Mega
// Basé sur l'algorithme algo.md et la nomenclature composants.pdf

// Configuration des broches
const int LEFT_SENSOR_PIN = 50;
const int RIGHT_SENSOR_PIN = 52;
const int LEFT_MOTOR_PIN = 10;
const int RIGHT_MOTOR_PIN = 11;
const int START_BUTTON_PIN = 2;  // À confirmer
const int OBSTACLE_SENSOR_PIN = A0;  // Capteur IR obstacle

// Variables d'état
bool started = false;
bool obstacleDetected = false;

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