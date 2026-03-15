#include <Arduino.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>


#define STEP1_PIN D5
#define DIR1_PIN  D2
#define EN1_PIN   D4

#define STEP2_PIN D6
#define DIR2_PIN  D3
#define EN2_PIN   D9

#define US_TRIG_PIN 12
#define US_ECHO_PIN 11

#define MODE_SWITCH_PIN  GPIO_NUM_42


#define SERVO_PIN D10

#define SPEED_HZ     5000
#define ACCELERATION 4000
#define OBSTACLE_CM  16

#define SERVO_MIN 30
#define SERVO_MAX 150
#define SERVO_DELAY 15
FastAccelStepperEngine engine;
FastAccelStepper *motorLeft;
FastAccelStepper *motorRight;

Servo headServo;

TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;

volatile bool pauseMotors = false;

long targetLeft = 0;
long targetRight = 0;

enum PathMode {
  PATH_YELLOW,
  PATH_BLUE
};

PathMode selectedPath;

void updateServo() {
  static int pos = (SERVO_MIN + SERVO_MAX) / 2;
  static int dir = 1;
  static unsigned long t = 0;

  if (millis() - t < SERVO_DELAY) return;
  t = millis();

  pos += dir;
  if (pos >= SERVO_MAX) { pos = SERVO_MAX; dir = -1; }
  if (pos <= SERVO_MIN) { pos = SERVO_MIN; dir = 1; }

  headServo.write(pos);
}

float readUltrasonic() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  long duration = pulseIn(US_ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 999.0f;

  return duration * 0.0343f / 2.0f;
}

void sensorTask(void *param) {

  bool obstacle = false;

  while (true) {

    float d = readUltrasonic();

    if (d < OBSTACLE_CM && !obstacle) {
      obstacle = true;
      pauseMotors = true;

      motorLeft->stopMove();
      motorRight->stopMove();

      Serial.println("⛔ Obstacle détecté → arrêt moteurs");
    }

    if (d >= OBSTACLE_CM && obstacle) {
      obstacle = false;
      pauseMotors = false;

      Serial.println("✅ Obstacle disparu → reprise trajectoire");
    }

    vTaskDelay(pdMS_TO_TICKS(80));
  }
}


void waitMotors() {
  while (motorLeft->isRunning() || motorRight->isRunning()) {

    while (pauseMotors) {
      vTaskDelay(pdMS_TO_TICKS(20));  // pause propre
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void safeMove(long leftDelta, long rightDelta) {

  targetLeft  = motorLeft->getCurrentPosition()  + leftDelta;
  targetRight = motorRight->getCurrentPosition() + rightDelta;

  motorLeft->moveTo(targetLeft);
  motorRight->moveTo(targetRight);

  while (motorLeft->isRunning() || motorRight->isRunning()) {

    if (pauseMotors) {
      motorLeft->stopMove();
      motorRight->stopMove();

      while (pauseMotors) {
        vTaskDelay(pdMS_TO_TICKS(20));
      }

      // reprise EXACTE
      motorLeft->moveTo(targetLeft);
      motorRight->moveTo(targetRight);
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void runYellowPath() {
  safeMove(16000, -16000);   // avancer
  safeMove(700, 700);  // tourner G
  safeMove(9000, -9000);
  safeMove(-700, -700);
  safeMove(2000, -2000);
}

void runBluePath() {
  safeMove(16000, -16000);   // avancer
  safeMove(-700, -700);  // tourner D
  safeMove(10000, -10000);
  safeMove(700, 700);
  safeMove(2000, -2000);
}

void motorTask(void *param) {

  while (true) {

    if (selectedPath == PATH_YELLOW) {
      runYellowPath();
    } else {
      runBluePath();
    }

    Serial.println("🏁 Trajectoire terminée");
    vTaskSuspend(NULL);   // arrêt définitif
  }
}



void setup() {

  Serial.begin(115200);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP);


  engine.init();

  motorLeft = engine.stepperConnectToPin(STEP1_PIN);
  motorRight = engine.stepperConnectToPin(STEP2_PIN);

  motorLeft->setDirectionPin(DIR1_PIN);
  motorLeft->setEnablePin(EN1_PIN);
  motorLeft->setAutoEnable(true);
  motorLeft->setSpeedInHz(SPEED_HZ);
  motorLeft->setAcceleration(ACCELERATION);

  motorRight->setDirectionPin(DIR2_PIN);
  motorRight->setEnablePin(EN2_PIN);
  motorRight->setAutoEnable(true);
  motorRight->setSpeedInHz(SPEED_HZ);
  motorRight->setAcceleration(ACCELERATION);

  headServo.attach(SERVO_PIN, 500, 2400);

if (digitalRead(MODE_SWITCH_PIN) == HIGH) {
  selectedPath = PATH_YELLOW;
  Serial.println("🟡 Parcours JAUNE sélectionné");
} else {
  selectedPath = PATH_BLUE;
  Serial.println("🔵 Parcours BLEU sélectionné");
}


  // Création des tâches FreeRTOS
  xTaskCreatePinnedToCore(
    motorTask,
    "Motor Task",
    4096,
    NULL,
    2,
    &motorTaskHandle,
    1   // Core 1
  );

  xTaskCreatePinnedToCore(
    sensorTask,
    "Sensor Task",
    2048,
    NULL,
    1,
    &sensorTaskHandle,
    0   // Core 0
  );

}

void loop() {
  updateServo();  // servo actif même quand moteurs suspendus
}