// Ball Maze Controller with ESP-NOW
// Improved state machine and debouncing logic

// files on SD for DFPlayer
// 1 - ready to start
// 2 - in progress
// 3 - end
// 4 - razz when hitting an obstacle. Should be short. maybe 1 second

// TO DO: Add code to run addressable LEDs on the game platform

#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "DFRobotDFPlayerMini.h"

HardwareSerial playerSerial(1);   // Use UART1 on ESP32
HardwareSerial displaySerial(2);  // Use UART2 on ESP32

DFRobotDFPlayerMini myDFPlayer;

typedef struct struct_message {
  int x;
  int y;
  int z;
} struct_message;

struct_message accelData;

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 290  // Minimum value
#define SERVOMAX 390  // Maximum value

#define SERVO_CENTER0 342  // adjust to center board 0 x
#define SERVO_CENTER1 327  // adjust to center board 1 y

#define SER0 0  // Servo Motor 0 on connector 0
#define SER1 1  // Servo Motor 1 on connector 1
int pwm0;       // PWM to drive servos
int pwm1;

#define MAXDIFF 15  // to limit really rapid changes
const int SENSOR_RANGE = 50;

#define STARTPIN 18
#define ENDPIN 19
#define RAZZPIN 25

// Game state machine
enum GameState {
  WAITING_FOR_START,
  IN_PROGRESS,
  ENDED
};

GameState gameState = WAITING_FOR_START;

// State flags
bool isProgressing = false;
bool razz = false;
bool isRazzing = false;

// Timing variables for interrupts
volatile unsigned long onEnd_time = 0;
volatile unsigned long previous_onEnd_time = 0;
volatile unsigned long razz_time = 0;
volatile unsigned long previous_razz_time = 0;

unsigned long intStartMillis = 0;

// Start detection with polling
bool wasBallOnStart = false;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&accelData, incomingData, sizeof(accelData));
}

// Interrupt for end position (with debouncing)
void IRAM_ATTR isr1() {
  onEnd_time = millis();
  if (onEnd_time - previous_onEnd_time > 250) {
    if (gameState == IN_PROGRESS) {
      gameState = ENDED;
    }
    previous_onEnd_time = onEnd_time;
  }
}

// Interrupt for razz detection (with debouncing)
void IRAM_ATTR isr2() {
  razz_time = millis();
  if (razz_time - previous_razz_time > 100) {
    razz = true;
    previous_razz_time = razz_time;
  }
}

String getTime() {
  unsigned long currentMillis = millis();
  float seconds = (currentMillis - intStartMillis) / 1000.0;
  return String(seconds, 1);  // Returns time with 1 decimal place
}

void updateDisplay(String displayValue) {
  displaySerial.println(displayValue);
}

bool isPlayingTrack() {
  int status = myDFPlayer.readState();
  return (status == 513);
}

void controlServos() {
  Serial.print("X: ");
  Serial.print(accelData.x);
  Serial.print(" -> PWM: ");
  Serial.print(map(accelData.x, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX));
  Serial.print(", Y: ");
  Serial.print(accelData.y);
  Serial.print(" -> PWM: ");
  Serial.println(map(accelData.y, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX));
  
  pwm0 = map(accelData.x, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  pca9685.setPWM(SER0, 0, pwm0);
  
  pwm1 = map(accelData.y, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  pca9685.setPWM(SER1, 0, pwm1);
}

void centerServos() {
  // Smoothly move servos to center position
  delay(40);
  
  // Center X axis
  if (pwm0 > SERVO_CENTER0) {
    for (int x = pwm0; x >= SERVO_CENTER0; x -= 5) {
      pca9685.setPWM(SER0, 0, x);
      delay(40);
    }
  } else if (pwm0 < SERVO_CENTER0) {
    for (int x = pwm0; x <= SERVO_CENTER0; x += 5) {
      pca9685.setPWM(SER0, 0, x);
      delay(40);
    }
  }

  // Center Y axis
  if (pwm1 > SERVO_CENTER1) {
    for (int y = pwm1; y >= SERVO_CENTER1; y -= 5) {
      pca9685.setPWM(SER1, 0, y);
      delay(40);
    }
  } else if (pwm1 < SERVO_CENTER1) {
    for (int y = pwm1; y <= SERVO_CENTER1; y += 5) {
      pca9685.setPWM(SER1, 0, y);
      delay(40);
    }
  }

  pca9685.setPWM(SER0, 0, SERVO_CENTER0);
  pca9685.setPWM(SER1, 0, SERVO_CENTER1);
  
  // Update PWM tracking variables
  pwm0 = SERVO_CENTER0;
  pwm1 = SERVO_CENTER1;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Turning on");
  
  playerSerial.begin(9600, SERIAL_8N1, 16, 17);    // RX=16, TX=17
  displaySerial.begin(38400, SERIAL_8N1, 26, 27);  // TX=27
  
  delay(500);
  Serial.println("Starting DFPlayer");
  if (!myDFPlayer.begin(playerSerial)) {
    Serial.println("Unable to begin DFPlayer Mini:");
    Serial.println("1. Check wiring!");
    Serial.println("2. Insert SD card!");
    while (true) {
      delay(1000);
    }
  }
  myDFPlayer.volume(30);  // Volume 0–30

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  pca9685.begin();
  pca9685.setPWMFreq(50);

  pinMode(STARTPIN, INPUT_PULLUP);
  pinMode(ENDPIN, INPUT_PULLUP);
  pinMode(RAZZPIN, INPUT_PULLUP);
  
  // Only attach interrupts for END and RAZZ (START uses polling)
  attachInterrupt(ENDPIN, isr1, RISING);
  attachInterrupt(RAZZPIN, isr2, RISING);

  // Initialize accelData to center
  accelData.x = 0;
  accelData.y = 0;
  accelData.z = 0;
  
  // Center servos on startup
  pwm0 = SERVO_CENTER0;
  pwm1 = SERVO_CENTER1;
  pca9685.setPWM(SER0, 0, SERVO_CENTER0);
  pca9685.setPWM(SER1, 0, SERVO_CENTER1);
  
  updateDisplay("Starting");
  delay(500);
}

void loop() {
  // Poll for ball on start position
  bool ballOnStart = (digitalRead(STARTPIN) == LOW);
  bool ballOnEnd = (digitalRead(ENDPIN) == HIGH);
  
  switch(gameState) {
    case WAITING_FOR_START:
      if (ballOnStart) {
        Serial.println("Ball on start position");
        centerServos();
        updateDisplay("Ready!");
        
        // Ensure track 1 is playing - switch if different track, restart if finished
        int currentTrack = myDFPlayer.readCurrentFileNumber();
        if (!isPlayingTrack()) {
          // Track finished or not playing - start track 1
          myDFPlayer.play(1);
          Serial.println("Starting track 1");
        } else if (currentTrack != 1) {
          // Wrong track playing - switch to track 1
          myDFPlayer.play(1);
          Serial.println("Switching to track 1");
        }
        // If track 1 is already playing, do nothing
      }
      
      // Detect when ball leaves start (game begins)
      if (!ballOnStart && wasBallOnStart) {
        gameState = IN_PROGRESS;
        isProgressing = false;
        intStartMillis = millis();
        Serial.println("Game started!");
      }
      
      wasBallOnStart = ballOnStart;
      break;
      
    case IN_PROGRESS:
      Serial.println("In progress");
      controlServos();
      updateDisplay(getTime());
      
      // Start progress music on first loop
      if (!isProgressing) {
        myDFPlayer.play(2);
        isProgressing = true;
      } else if (!isPlayingTrack() && !razz) {
        // Resume progress music if stopped (and not razzing)
        myDFPlayer.play(2);
      }
      
      // Handle razz (obstacle hit)
      if (razz) {
        if (!isRazzing) {
          myDFPlayer.play(4);
          isRazzing = true;
        } else {
          // Wait for razz sound to finish
          if (!isPlayingTrack()) {
            razz = false;
            isRazzing = false;
          }
        }
      }
      break;
      
    case ENDED:
      Serial.println("Game ended!");
      String finalTime = getTime();
      updateDisplay(finalTime);
      myDFPlayer.play(3);
      centerServos();
      
      // Wait for servos to center before checking for restart
      delay(1000);
      
      // Stay in ENDED state until ball is placed back on start
      while (gameState == ENDED) {
        ballOnStart = (digitalRead(STARTPIN) == LOW);
        if (ballOnStart) {
          gameState = WAITING_FOR_START;
          isProgressing = false;
          razz = false;
          isRazzing = false;
          Serial.println("Reset to start - ready for new game");
          updateDisplay("Ready!");
          break;
        }
        delay(100);
      }
      break;
  }
  
  delay(40);
}
