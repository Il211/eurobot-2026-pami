#include <Arduino.h>
#include <FastAccelStepper.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>



// =====================
// PINS
// =====================
#define STEP1_PIN D5
#define DIR1_PIN  D2
#define EN1_PIN   D4

#define STEP2_PIN D6
#define DIR2_PIN  D3
#define EN2_PIN   D9

#define US_TRIG_PIN 12
#define US_ECHO_PIN 11
#define SERVO_PIN D10

#define PAMI 4

// =====================
// PARAMS
// =====================
#define SPEED_HZ     50000
#define ACCELERATION 10000
#define OBSTACLE_CM  16

#define SERVO_MIN 30
#define SERVO_MAX 150
#define SERVO_DELAY 15

#define CHANNEL 1

// =====================
// ESP-NOW
// =====================
volatile bool colorReceived = false;
volatile uint8_t lastColor = 255;
bool pathChosen = false;
bool trajectoryFinished = false;
volatile bool forceStop = false;

unsigned long startTime = 0;
bool startReceived = false;
bool trajectoryStarted = false;

#define MAX_RUN_TIME 100000   // 100 secondes en ms

// =====================
// ENUM
// =====================
enum PathMode {
  PATH_YELLOW,
  PATH_BLUE
};

PathMode selectedPath;

// =====================
// MOTEURS
// =====================
FastAccelStepperEngine engine;
FastAccelStepper *motorLeft;
FastAccelStepper *motorRight;

TaskHandle_t motorTaskHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;

volatile bool pauseMotors = false;

long targetLeft = 0;
long targetRight = 0;

// =====================
// SERVO
// =====================
Servo headServo;

// =====================
// ESP-NOW CALLBACK
// =====================
void onReceive(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if(len == 1){
    lastColor = incomingData[0];
    colorReceived = true;
  }
}

void timerTask(void *param){

  // attendre réception START
  while(!startReceived){
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  Serial.println("⏱️ Chrono lancé (100s)");

  unsigned long t0 = millis();

  while(true){

    if(millis() - t0 >= MAX_RUN_TIME){

      Serial.println("⏰ FIN TEMPS → STOP TOTAL + SERVO");

      forceStop = true;
      trajectoryFinished = true;
      pauseMotors = false;

      // stop moteurs IMMEDIAT
      motorLeft->forceStop();
      motorRight->forceStop();

      vTaskDelete(NULL); // kill timer
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void initEspNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);

  uint8_t newMac[6] = {0xE8, 0x06, 0x90, 0xA0, 0x48, 0x38};
  if (esp_wifi_set_mac(WIFI_IF_STA, newMac) == ESP_OK) {
    Serial.println("MAC modifiée avec succès !");
  } else {
    Serial.println("Erreur lors de la modification du MAC");
  }


  if(esp_now_init() != ESP_OK){
    Serial.println("ESP NOW FAIL");
    return;
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println("ESP NOW OK");
}



// =====================
// ULTRASON
// =====================
float readUltrasonic(){
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  long duration = pulseIn(US_ECHO_PIN,HIGH,30000);
  if(duration==0) return 999.0f;
  return duration*0.0343f/2.0f;
}

// =====================
// SENSOR TASK
// =====================
void sensorTask(void *param){
  bool obstacle = false;

  while(true){
    
    if(trajectoryFinished){
      pauseMotors = false;
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;    
    }

    float d = readUltrasonic();

    if(d < OBSTACLE_CM && !obstacle){
      obstacle=true;
      pauseMotors=true;
      motorLeft->stopMove();
      motorRight->stopMove();
      Serial.println("⛔ Obstacle");
    }

    if(d >= OBSTACLE_CM && obstacle){
      obstacle=false;
      pauseMotors=false;
      Serial.println("✅ Reprise");
    }

    vTaskDelay(pdMS_TO_TICKS(80));
  }
}

// =====================
// SAFE MOVE
// =====================
void safeMove(long leftDelta, long rightDelta){

  targetLeft = motorLeft->getCurrentPosition() + leftDelta;
  targetRight = motorRight->getCurrentPosition() + rightDelta;

  motorLeft->moveTo(targetLeft);
  motorRight->moveTo(targetRight);

while (motorLeft->isRunning() || motorRight->isRunning()) {

  // 🔴 STOP GLOBAL PRIORITAIRE
  if(forceStop){
    motorLeft->forceStop();
    motorRight->forceStop();
    return;
  }

  // ⛔ obstacle
  if (pauseMotors) {
    motorLeft->stopMove();
    motorRight->stopMove();

    while (pauseMotors && !forceStop){
      vTaskDelay(pdMS_TO_TICKS(20));
    }

    if(forceStop) return;

    motorLeft->moveTo(targetLeft);
    motorRight->moveTo(targetRight);
  }

  vTaskDelay(pdMS_TO_TICKS(2));
}
}

// =====================
// TRAJECTOIRES
// =====================
void runYellowPath() {
  if (PAMI == 1) {
    delay(5000);
    safeMove(10000, -10000);   // avancer
  }

  if (PAMI == 2) {
    safeMove(9500, -9500);   // avancer
    safeMove(700, 700);  // tourner G
    safeMove(1000, -1000);
    safeMove(-700, -700);
    safeMove(7000, -7000);
    safeMove(700, 700);
    safeMove(4000, -4000);
    safeMove(-700, -700);
    safeMove(1000, -1000);  
  }

  if (PAMI == 3) {
    delay(5000);
    safeMove(9500, -9500);   // avancer
    safeMove(700, 700);  // tourner G
    safeMove(5000, -5000);
    safeMove(-700, -700);
    safeMove(900, -900);
    

  }
  if (PAMI == 4) {
    delay(3000);
    safeMove(9500, -9500);   // avancer
    safeMove(700, 700);  // tourner G
    safeMove(11000, -11000);
    safeMove(-700, -700);
    safeMove(1500, -1500);   

  }
  if (PAMI == 5) {
    safeMove(16000, -16000);
    safeMove(700, 700);
    safeMove(9000, -9000);
    safeMove(-700, -700);
    safeMove(2000, -2000); 
  } 
}

void runBluePath() {
  if (PAMI == 1) {
    delay(5000);
    safeMove(10000, -10000);   // avancer
  }

  if (PAMI == 2) {
    safeMove(9500, -9500);   // avancer
    safeMove(-700, -700);  // tourner G
    safeMove(1000, -1000);
    safeMove(700, 700);
    safeMove(7000, -7000);
    safeMove(-700, -700);
    safeMove(4000, -4000);
    safeMove(700, 700);
    safeMove(1000, -1000);   
  }
  
  if (PAMI == 3) {
    delay(5000);
    safeMove(9500, -9500);   // avancer
    safeMove(-700, -700);  // tourner D
    safeMove(5000, -5000);
    safeMove(700, 700);
    safeMove(900, -900);  
  }
  if (PAMI == 4) {
    delay(3000);
    safeMove(9500, -9500);   // avancer
    safeMove(-700, -700);  // tourner D
    safeMove(11000, -11000);
    safeMove(700, 700);
    safeMove(1000, -1000);
  }

  if (PAMI == 5) {
    safeMove(16000, -16000);
    safeMove(700, 700);
    safeMove(9000, -9000);
    safeMove(-700, -700);
    safeMove(2000, -2000);
  }
}

// =====================
// MOTOR TASK
// =====================
void motorTask(void *param){

  Serial.println("⏳ Attente START...");

  // 🔴 1. Attente réception START
  while(!startReceived){
    if(colorReceived){
      colorReceived = false;

      startTime = millis();
      startReceived = true;

      if(lastColor == 0){
        selectedPath = PATH_YELLOW;
        Serial.println("🟡 JAUNE reçu");
      } else {
        selectedPath = PATH_BLUE;
        Serial.println("🔵 BLEU reçu");
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // ⏳ 2. Attente 85 secondes AVANT départ
  Serial.println("⏳ Attente 85s avant départ...");

  while(!trajectoryStarted){

    if(millis() - startTime >= 85000){
      trajectoryStarted = true;
      Serial.println("🚀 Départ !");
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }

  // 🚀 3. Lancement trajectoire
  if(selectedPath == PATH_YELLOW){
    runYellowPath();
  } else {
    runBluePath();
  }

  Serial.println("🏁 Trajectoire terminée");

  trajectoryFinished = true;

  vTaskSuspend(NULL);
}

void updateServo() {

  if (!trajectoryFinished) return;  // ❌ rien avant la fin

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

// =====================
// SETUP
// =====================
void setup(){
  Serial.begin(115200);

  Serial.println("PAMI ESCLAVE 🚀");

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);

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

  headServo.attach(SERVO_PIN,500,2400);

  initEspNow();

  xTaskCreatePinnedToCore(motorTask,"Motor",4096,NULL,2,&motorTaskHandle,1);
  xTaskCreatePinnedToCore(sensorTask,"Sensor",2048,NULL,1,&sensorTaskHandle,0);
  xTaskCreatePinnedToCore(
  timerTask,
  "Timer",
  2048,
  NULL,
  3,   // priorité haute
  NULL,
  0
);
}

// =====================
// LOOP
// =====================
void loop(){
  updateServo();
  vTaskDelay(pdMS_TO_TICKS(2));
}