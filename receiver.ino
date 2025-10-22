// Ball Maze Controller with ESP-NOW
// Non-blocking audio - servos work even if DFPlayer fails
// FIXED: Track 2 now loops properly during gameplay with improved debouncing

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

// DFPlayer BUSY pin - LOW when playing, HIGH when idle
#define DFPLAYER_BUSY_PIN 34

// Audio system control
bool audioAvailable = false;  // Set to true if DFPlayer initializes successfully

// Forward declarations
void setup();
void loop();
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);
void controlServos();
void centerServos();
String getTime();
void updateDisplay(String displayValue);
bool isTrackFinished();
void playTrack(int trackNumber);

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

// Servo 0 (X-axis) limits
// #define SERVO0MIN 281  // Minimum value (331 - 50)
// #define SERVO0MAX 381  // Maximum value (331 + 50)
#define SERVO_CENTER0 331  // adjust to center board 0 x

// Servo 1 (Y-axis) limits
// #define SERVO1MIN 287  // Minimum value (337 - 50)
// #define SERVO1MAX 387  // Maximum value (337 + 50)
#define SERVO_CENTER1 375  // adjust to center board 1 y
#define SERVO_RANGE 50

#define SERVO0MIN (SERVO_CENTER0 - SERVO_RANGE)  // X
#define SERVO0MAX (SERVO_CENTER0 + SERVO_RANGE)  // X
#define SERVO1MIN (SERVO_CENTER1 - SERVO_RANGE)  // Y
#define SERVO1MAX (SERVO_CENTER1 + SERVO_RANGE)  // Y

#define SER0 0  // Servo Motor 0 on connector 0
#define SER1 1  // Servo Motor 1 on connector 1
int pwm0;       // PWM to drive servos
int pwm1;
int targetPwm0; // Target PWM values for smooth movement
int targetPwm1;

// Smoothing and limiting parameters
#define MAX_CHANGE_PER_LOOP 8   // Maximum PWM change per 40ms loop (increased from 3 for faster response)
#define DEADZONE 2              // Ignore small accel changes (reduces jitter)
#define SMOOTHING_FACTOR 0.5    // Low-pass filter (0.0-1.0, higher = more responsive, was 0.3)
const int SENSOR_RANGE = 50;

// Smoothed accelerometer values
float smoothedX = 0;
float smoothedY = 0;

#define STARTPIN 18
#define ENDPIN 19
#define RAZZPIN 25

unsigned long lastTrackStartTime = 0;
#define TRACK_START_GRACE_PERIOD 1000  // INCREASED: Wait 1000ms after starting track before checking if finished

// Track 2 debouncing to prevent premature restarts
unsigned long track2FinishedTime = 0;
bool track2WasFinished = false;
#define TRACK2_RESTART_DEBOUNCE 800  // Wait 800ms of continuous "finished" state before restarting

// Game state machine
enum GameState {
  WAITING_FOR_START,
  IN_PROGRESS,
  ENDED
};

GameState gameState = WAITING_FOR_START;

// State flags
bool isProgressing = false;
volatile bool razz = false;
bool isRazzing = false;

// Timing variables for interrupts
volatile unsigned long onEnd_time = 0;
volatile unsigned long previous_onEnd_time = 0;
volatile unsigned long razz_time = 0;
volatile unsigned long previous_razz_time = 0;

unsigned long intStartMillis = 0;
unsigned long lastDisplayUpdate = 0;
unsigned long lastMusicCheck = 0;
int currentlyPlayingTrack = 0;

#define DISPLAY_UPDATE_INTERVAL 100  // Update display every 100ms
#define MUSIC_CHECK_INTERVAL 500     // Check music status every 500ms

// Start detection with polling
bool wasBallOnStart = false;

// Debouncing for ball sensors
unsigned long lastStartDebounceTime = 0;
unsigned long lastEndDebounceTime = 0;
bool lastStartReading = HIGH;
bool lastEndReading = HIGH;
bool debouncedStartState = HIGH;
bool debouncedEndState = HIGH;
#define DEBOUNCE_DELAY 50  // 50ms debounce time

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&accelData, incomingData, sizeof(accelData));
  // Serial.print("Data received - X: ");
  // Serial.print(accelData.x);
  // Serial.print(" Y: ");
  // Serial.println(accelData.y);
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
  //Serial.println(displayValue);
  displaySerial.println(displayValue);
}

bool isTrackFinished() {
  if (!audioAvailable) return true;  // If no audio, always report finished
  
  // Don't check BUSY pin for a grace period after starting track
  if (millis() - lastTrackStartTime < TRACK_START_GRACE_PERIOD) {
    return false;  // Track just started, definitely not finished yet
  }
  
  // Read BUSY pin: LOW = playing, HIGH = idle/finished
  return (digitalRead(DFPLAYER_BUSY_PIN) == HIGH);
}

void playTrack(int trackNumber) {
  if (!audioAvailable) {
    Serial.println("Audio not available - skipping playback");
    return;  // Skip if audio not working
  }
  
  myDFPlayer.play(trackNumber);
  currentlyPlayingTrack = trackNumber;
  lastTrackStartTime = millis();  // Record when track started
  
  // Reset Track 2 debouncing when starting any track
  track2WasFinished = false;
  track2FinishedTime = 0;
  
  Serial.print("Playing track ");
  Serial.println(trackNumber);
}

void controlServos() {
  //Serial.println("inside control servos");
  // Apply low-pass filter for smoother values (exponential moving average)
  smoothedX = (SMOOTHING_FACTOR * accelData.x) + ((1.0 - SMOOTHING_FACTOR) * smoothedX);
  smoothedY = (SMOOTHING_FACTOR * accelData.y) + ((1.0 - SMOOTHING_FACTOR) * smoothedY);
  
  // Apply deadzone to ignore tiny movements
  float effectiveX = (abs(smoothedX) < DEADZONE) ? 0 : smoothedX;
  float effectiveY = (abs(smoothedY) < DEADZONE) ? 0 : smoothedY;
  
  // Calculate target PWM values
  targetPwm0 = map(effectiveX, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVO0MIN, SERVO0MAX);
  // REVERSED Y AXIS MAPPING - swap min/max parameters
  targetPwm1 = map(effectiveY, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVO1MAX, SERVO1MIN);
  
  // Constrain targets to valid range
  targetPwm0 = constrain(targetPwm0, SERVO0MIN, SERVO0MAX);
  targetPwm1 = constrain(targetPwm1, SERVO1MIN, SERVO1MAX);
  
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
  
  // Minimal debug output - only show when values change significantly
  static int lastPrintedPwm0 = 0;
  static int lastPrintedPwm1 = 0;
  if (abs(pwm0 - lastPrintedPwm0) > 10 || abs(pwm1 - lastPrintedPwm1) > 10) {
    // Serial.print("Servo - X: ");
    // Serial.print(accelData.x);
    // Serial.print(" -> PWM: ");
    // Serial.print(pwm0);
    // Serial.print(" | Y: ");
    // Serial.print(accelData.y);
    // Serial.print(" -> PWM: ");
    // Serial.println(pwm1);
    lastPrintedPwm0 = pwm0;
    lastPrintedPwm1 = pwm1;
  }
}

void centerServos() {
  Serial.println("Centering servos...");
  
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
  delay(500);
  Serial.println("\n\n=================================");
  Serial.println("Ball Maze Controller Starting...");
  Serial.println("=================================");
  
  // Initialize display serial first
  displaySerial.begin(38400, SERIAL_8N1, 26, 27);  // TX=27
  Serial.println("Display serial initialized");
  
  // Try to initialize DFPlayer (non-blocking if it fails)
  Serial.println("Attempting DFPlayer initialization...");
  playerSerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(500);
  
  if (myDFPlayer.begin(playerSerial)) {
    Serial.println("DFPlayer initialized successfully!");
    myDFPlayer.volume(30);
    audioAvailable = true;
  } else {
    Serial.println("WARNING: DFPlayer not available - continuing without audio");
    audioAvailable = false;
    // Don't halt - continue with servos
  }

  // Initialize WiFi and ESP-NOW
  Serial.println("Initializing ESP-NOW...");
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW initialization failed!");
  } else {
    Serial.println("ESP-NOW initialized successfully");
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  }
  
  // Initialize PCA9685 servo driver
  Serial.println("Initializing PCA9685...");
  pca9685.begin();
  pca9685.setPWMFreq(50);
  Serial.println("PCA9685 initialized");

  // Setup pins
  pinMode(STARTPIN, INPUT_PULLUP);
  pinMode(ENDPIN, INPUT_PULLUP);
  pinMode(RAZZPIN, INPUT_PULLUP);
  pinMode(DFPLAYER_BUSY_PIN, INPUT);  // BUSY pin from DFPlayer
  
  // Only attach interrupts for END and RAZZ (START uses polling)
  attachInterrupt(ENDPIN, isr1, RISING);
  attachInterrupt(RAZZPIN, isr2, RISING);
  //Serial.println("Interrupts configured");

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
  
  Serial.println("Setting servos to center position...");
  pca9685.setPWM(SER0, 0, SERVO_CENTER0);
  pca9685.setPWM(SER1, 0, SERVO_CENTER1);
  // Serial.print("Servo 0 PWM: ");
  // Serial.println(SERVO_CENTER0);
  // Serial.print("Servo 1 PWM: ");
  // Serial.println(SERVO_CENTER1);
  
  updateDisplay("START");
  Serial.println("=== Setup Complete - Waiting for ball ===\n");
  delay(500);
}

void loop() {
  // Read raw sensor values
  bool startReading = digitalRead(STARTPIN);
  bool endReading = digitalRead(ENDPIN);
  
  // Debounce START sensor
  if (startReading != lastStartReading) {
    lastStartDebounceTime = millis();
  }
  if ((millis() - lastStartDebounceTime) > DEBOUNCE_DELAY) {
    if (startReading != debouncedStartState) {
      debouncedStartState = startReading;
    }
  }
  lastStartReading = startReading;
  
  // Debounce END sensor
  if (endReading != lastEndReading) {
    lastEndDebounceTime = millis();
  }
  if ((millis() - lastEndDebounceTime) > DEBOUNCE_DELAY) {
    if (endReading != debouncedEndState) {
      debouncedEndState = endReading;
    }
  }
  lastEndReading = endReading;
  
  // Use debounced values (START is active LOW, END is active HIGH)
  bool ballOnStart = (debouncedStartState == LOW);
  bool ballOnEnd = (debouncedEndState == HIGH);
  
  switch(gameState) {
    case WAITING_FOR_START:
      // Reset smoothing only when ball first arrives on start
      if (ballOnStart && !wasBallOnStart) {
        // Ball just arrived on start
        Serial.println("Ball placed on start position");
        smoothedX = 0;
        smoothedY = 0;
        centerServos();
        updateDisplay("Ready!");
        playTrack(4);  // Play ready sound
        wasBallOnStart = true;
      }
      
      if (ballOnStart) {
        // Ball is on start position - SERVOS ACTIVE for practice
        controlServos();  // Allow user to control servos while waiting
        
        // While ball is on start, keep ready track playing
        if (millis() - lastMusicCheck >= MUSIC_CHECK_INTERVAL) {
          if (isTrackFinished() && currentlyPlayingTrack != 1) {
            playTrack(4);
          }
          lastMusicCheck = millis();
        }
      } else {
        // Ball is NOT on start position
        if (wasBallOnStart) {
          // Ball just left start - BEGIN GAME!
          gameState = IN_PROGRESS;
          isProgressing = false;
          intStartMillis = millis();
          lastDisplayUpdate = millis();
          Serial.println("=== GAME STARTED ===");
        }
        
        wasBallOnStart = false;  // Update state: ball is not on start
      }
      delay(40);  // Changed from 50 to match IN_PROGRESS timing
      break;
      
    case IN_PROGRESS:
      // Control servos based on accelerometer data (RUNS EVERY LOOP)
      controlServos();
      
      // Update display periodically (not every loop)
      if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
        updateDisplay(getTime());
        lastDisplayUpdate = millis();
      }
      
      // Music management: Track 2 loops, interrupted by Track 4 for razz
      if (razz) {
        if (!isRazzing) {
          Serial.println("Razz detected - playing track 4");
          playTrack(1);
          isRazzing = true;
        } else {
          // Wait for razz sound to finish before resuming track 2
          if (isTrackFinished()) {
            Serial.println("Razz complete - resuming track 2");
            playTrack(2);
            razz = false;
            isRazzing = false;
          }
        }
      } else {
        // Normal progress state: ensure track 2 is playing and looping
        if (!isProgressing) {
          Serial.println("Starting progress music (track 2)");
          playTrack(2);
          isProgressing = true;
          lastMusicCheck = millis();  // Initialize music check timer
        } else {
          // IMPROVED: Check Track 2 status with debouncing to prevent premature restarts
          if (millis() - lastMusicCheck >= MUSIC_CHECK_INTERVAL) {
            if (currentlyPlayingTrack == 2) {
              bool trackFinished = isTrackFinished();
              
              if (trackFinished) {
                // Track appears finished
                if (!track2WasFinished) {
                  // First detection of finished state - start debounce timer
                  track2FinishedTime = millis();
                  track2WasFinished = true;
                  Serial.println("Track 2 appears finished - starting debounce timer");
                } else if (millis() - track2FinishedTime >= TRACK2_RESTART_DEBOUNCE) {
                  // Track has been finished continuously for debounce period - restart it
                  Serial.println("Track 2 confirmed finished - restarting");
                  playTrack(2);
                }
              } else {
                // Track is playing - reset debounce state
                if (track2WasFinished) {
                  Serial.println("Track 2 still playing - false alarm");
                }
                track2WasFinished = false;
              }
            }
            lastMusicCheck = millis();
          }
        }
      }
      
      delay(40);
      break;
      
    case ENDED:
      Serial.println("=== GAME ENDED ===");
      String finalTime = getTime();
      updateDisplay(finalTime);
      
      // Center servos and hold position
      centerServos();
      
      // Play end track once - only if not already playing
      if (currentlyPlayingTrack != 3) {
        playTrack(3);
        Serial.print("Final time: ");
        Serial.println(finalTime);
      }
      
      // Wait for servos to center and track to start
      delay(1000);
      
      // Stay in ENDED state until ball is removed from end
      while (gameState == ENDED) {
        ballOnEnd = (digitalRead(ENDPIN) == HIGH);
        
        // Reset smoothing filters to prevent accumulated tilt during ball transfer
        smoothedX = 0;
        smoothedY = 0;
        
        if (ballOnEnd) {
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
            Serial.println("Reset to start - ready for new game");
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
            Serial.println("Reset to start - ready for new game");
            updateDisplay("Ready!");
            break;
          }
        }
        
        delay(100);
      }
      break;
  }
  
  //delay(40);
}
