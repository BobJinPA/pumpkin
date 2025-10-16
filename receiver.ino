// Ball Maze Controller with ESP-NOW
// Improved state machine and debouncing logic

// files on SD for DFPlayer
// 1 - ready to start
// 2 - in progress
// 3 - end
// 4 - razz when hitting an obstacle. Should be short. maybe 1 second

// TO DO: Add code to run addressable LEDs on the game platform

// Debug mode - set to false for production to improve performance
#define DEBUG_MODE false

#if DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

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
int targetPwm0; // Target PWM values for smooth movement
int targetPwm1;

// Smoothing and limiting parameters
#define MAX_CHANGE_PER_LOOP 3   // Maximum PWM change per 40ms loop (smoother movement)
#define DEADZONE 2              // Ignore small accel changes (reduces jitter)
#define SMOOTHING_FACTOR 0.3    // Low-pass filter (0.0-1.0, lower = smoother)
const int SENSOR_RANGE = 50;

// Smoothed accelerometer values
float smoothedX = 0;
float smoothedY = 0;

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
unsigned long lastDisplayUpdate = 0;
unsigned long lastMusicCheck = 0;

#define DISPLAY_UPDATE_INTERVAL 100  // Update display every 100ms
#define MUSIC_CHECK_INTERVAL 200     // Check music status every 200ms

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
  // Apply low-pass filter for smoother values (exponential moving average)
  smoothedX = (SMOOTHING_FACTOR * accelData.x) + ((1.0 - SMOOTHING_FACTOR) * smoothedX);
  smoothedY = (SMOOTHING_FACTOR * accelData.y) + ((1.0 - SMOOTHING_FACTOR) * smoothedY);
  
  // Apply deadzone to ignore tiny movements
  float effectiveX = (abs(smoothedX) < DEADZONE) ? 0 : smoothedX;
  float effectiveY = (abs(smoothedY) < DEADZONE) ? 0 : smoothedY;
  
  // Calculate target PWM values
  targetPwm0 = map(effectiveX, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  targetPwm1 = map(effectiveY, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  
  // Constrain targets to valid range
  targetPwm0 = constrain(targetPwm0, SERVOMIN, SERVOMAX);
  targetPwm1 = constrain(targetPwm1, SERVOMIN, SERVOMAX);
  
  // Gradually move toward target (rate limiting)
  int diff0 = targetPwm0 - pwm0;
  if (diff0 > MAX_CHANGE_PER_LOOP) {
    pwm0 += MAX_CHANGE_PER_LOOP;
  } else if (diff0 < -MAX_CHANGE_PER_LOOP) {
    pwm0 -= MAX_CHANGE_PER_LOOP;
  } else {
    pwm0 = targetPwm0;  // Close enough, snap to target
  }
  
  int diff1 = targetPwm1 - pwm1;
  if (diff1 > MAX_CHANGE_PER_LOOP) {
    pwm1 += MAX_CHANGE_PER_LOOP;
  } else if (diff1 < -MAX_CHANGE_PER_LOOP) {
    pwm1 -= MAX_CHANGE_PER_LOOP;
  } else {
    pwm1 = targetPwm1;  // Close enough, snap to target
  }
  
  // Apply to servos
  pca9685.setPWM(SER0, 0, pwm0);
  pca9685.setPWM(SER1, 0, pwm1);
  
  // Debug output (only if DEBUG_MODE enabled)
  #if DEBUG_MODE
  DEBUG_PRINT("X: ");
  DEBUG_PRINT(accelData.x);
  DEBUG_PRINT(" (smooth: ");
  DEBUG_PRINT(smoothedX, 1);
  DEBUG_PRINT(") -> PWM: ");
  DEBUG_PRINT(pwm0);
  DEBUG_PRINT(" | Y: ");
  DEBUG_PRINT(accelData.y);
  DEBUG_PRINT(" (smooth: ");
  DEBUG_PRINT(smoothedY, 1);
  DEBUG_PRINT(") -> PWM: ");
  DEBUG_PRINTLN(pwm1);
  #endif
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
  DEBUG_PRINTLN("Turning on");
  
  playerSerial.begin(9600, SERIAL_8N1, 16, 17);    // RX=16, TX=17
  displaySerial.begin(38400, SERIAL_8N1, 26, 27);  // TX=27
  
  delay(500);
  DEBUG_PRINTLN("Starting DFPlayer");
  if (!myDFPlayer.begin(playerSerial)) {
    DEBUG_PRINTLN("Unable to begin DFPlayer Mini:");
    DEBUG_PRINTLN("1. Check wiring!");
    DEBUG_PRINTLN("2. Insert SD card!");
    while (true) {
      delay(1000);
    }
  }
  myDFPlayer.volume(30);  // Volume 0–30

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    DEBUG_PRINTLN("Error initializing ESP-NOW");
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

  // Initialize accelData and smoothing to center
  accelData.x = 0;
  accelData.y = 0;
  accelData.z = 0;
  smoothedX = 0;
  smoothedY = 0;
  
  // Center servos on startup
  pwm0 = SERVO_CENTER0;
  pwm1 = SERVO_CENTER1;
  targetPwm0 = SERVO_CENTER0;
  targetPwm1 = SERVO_CENTER1;
  pca9685.setPWM(SER0, 0, SERVO_CENTER0);
  pca9685.setPWM(SER1, 0, SERVO_CENTER1);
  
  updateDisplay("START");  // Shortened for better display
  delay(500);
}

void loop() {
  // Poll for ball on start position
  bool ballOnStart = (digitalRead(STARTPIN) == LOW);
  bool ballOnEnd = (digitalRead(ENDPIN) == HIGH);
  
  switch(gameState) {
    case WAITING_FOR_START:
      // Continuously reset smoothing to prevent pre-game tilt accumulation
      smoothedX = 0;
      smoothedY = 0;
      
      if (ballOnStart) {
        DEBUG_PRINTLN("Ball on start position");
        centerServos();
        updateDisplay("Ready!");
        
        // Check music status periodically (not every loop)
        if (millis() - lastMusicCheck >= MUSIC_CHECK_INTERVAL) {
          int currentTrack = myDFPlayer.readCurrentFileNumber();
          if (!isPlayingTrack()) {
            myDFPlayer.play(1);
            DEBUG_PRINTLN("Starting track 1");
          } else if (currentTrack != 1) {
            myDFPlayer.play(1);
            DEBUG_PRINTLN("Switching to track 1");
          }
          lastMusicCheck = millis();
        }
      }
      
      // Detect when ball leaves start (game begins)
      if (!ballOnStart && wasBallOnStart) {
        gameState = IN_PROGRESS;
        isProgressing = false;
        intStartMillis = millis();
        lastDisplayUpdate = millis();  // Initialize display timer
        DEBUG_PRINTLN("Game started!");
      }
      
      wasBallOnStart = ballOnStart;
      break;
      
    case IN_PROGRESS:
      // Control servos based on accelerometer data
      controlServos();
      
      // Update display periodically (not every loop)
      if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
        updateDisplay(getTime());
        lastDisplayUpdate = millis();
      }
      
      // Music management: Track 2 loops, interrupted by Track 4 for razz
      if (razz) {
        // Razz interrupt: play track 4
        if (!isRazzing) {
          DEBUG_PRINTLN("Razz detected - playing track 4");
          myDFPlayer.play(4);
          isRazzing = true;
        } else {
          // Wait for razz sound to finish before resuming track 2
          if (!isPlayingTrack()) {
            DEBUG_PRINTLN("Razz complete - resuming track 2");
            myDFPlayer.play(2);  // Resume progress music
            razz = false;
            isRazzing = false;
          }
        }
      } else {
        // Normal progress state: ensure track 2 is playing
        if (!isProgressing) {
          // First entry into IN_PROGRESS - start track 2
          DEBUG_PRINTLN("Starting progress music (track 2)");
          myDFPlayer.play(2);
          isProgressing = true;
        } else if (!isPlayingTrack()) {
          // Track 2 finished - restart it (loop behavior)
          DEBUG_PRINTLN("Track 2 completed - restarting");
          myDFPlayer.play(2);
        }
      }
      break;
      
    case ENDED:
      DEBUG_PRINTLN("Game ended!");
      String finalTime = getTime();
      updateDisplay(finalTime);
      
      // Center servos and hold position
      centerServos();
      
      // Play end track once
      myDFPlayer.play(3);
      DEBUG_PRINT("Final time: ");
      DEBUG_PRINTLN(finalTime);
      
      // Wait for servos to center and track to start
      delay(1000);
      
      // Stay in ENDED state until ball is removed from end
      while (gameState == ENDED) {
        ballOnEnd = (digitalRead(ENDPIN) == HIGH);
        
        // Reset smoothing filters to prevent accumulated tilt during ball transfer
        smoothedX = 0;
        smoothedY = 0;
        
        if (ballOnEnd) {
          // Ball still on end - keep track 3 playing/looping
          if (!isPlayingTrack()) {
            myDFPlayer.play(3);
            DEBUG_PRINTLN("Track 3 completed - restarting");
          }
          
          // Check for ball on start to reset
          ballOnStart = (digitalRead(STARTPIN) == LOW);
          if (ballOnStart) {
            gameState = WAITING_FOR_START;
            isProgressing = false;
            razz = false;
            isRazzing = false;
            // Reset PWM tracking to center position
            pwm0 = SERVO_CENTER0;
            pwm1 = SERVO_CENTER1;
            targetPwm0 = SERVO_CENTER0;
            targetPwm1 = SERVO_CENTER1;
            DEBUG_PRINTLN("Reset to start - ready for new game");
            updateDisplay("Ready!");
            break;
          }
        } else {
          // Ball removed from end - wait for it to be placed on start
          ballOnStart = (digitalRead(STARTPIN) == LOW);
          if (ballOnStart) {
            gameState = WAITING_FOR_START;
            isProgressing = false;
            razz = false;
            isRazzing = false;
            // Reset PWM tracking to center position
            pwm0 = SERVO_CENTER0;
            pwm1 = SERVO_CENTER1;
            targetPwm0 = SERVO_CENTER0;
            targetPwm1 = SERVO_CENTER1;
            DEBUG_PRINTLN("Reset to start - ready for new game");
            updateDisplay("Ready!");
            break;
          }
        }
        
        delay(100);
      }
      break;
  }
  
  delay(40);
}
