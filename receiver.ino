// to consider
// when ball is taken off end to be placed on start. What do the motors do. they may go crazy and start movin based on the position of the accel.
// either handle in code, hardware, or behavior BJ 10/14


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
HardwareSerial displaySerial(2);  // Use UART2 on ESP32 - make sure correct BAUD for ard

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

#define SER0 0  //Servo Motor 0 on connector 0
#define SER1 1  //Servo Motor 1 on connector 1
int pwm0;       // PWM to drive servos - gets fed into the motor after mapping he xval changes
int pwm1;

#define MAXDIFF 15  // to limit really rapid changes
int xVal;
int yVal;
int previousXVal = 0;
int previousYVal = 0;

const int SENSOR_RANGE = 50;

#define STARTPIN 18
#define ENDPIN 19
#define RAZZPIN 25

int status;
bool razz = false;
bool isRazzing = false;
unsigned long razzStartTime;
bool isProgressing = false;

static bool hasEnded = false;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&accelData, incomingData, sizeof(accelData));
}

//variables to keep track of the timing of recent interrupts
unsigned long onEnd_time = 0;
unsigned long previous_onEnd_time = 0;

unsigned long intStartMillis;
unsigned long currentMillis;
bool inProgress;
bool onStart = false;
bool onEnd = false;

void IRAM_ATTR isr0() {  // fires when onEnd is detected. checks twice to make sure the ball does not skip past the end
  intStartMillis = millis();
  onStart = false;
}

// interrupt functions for end and razz
void IRAM_ATTR isr1() {  // fires when onEnd is detected. checks twice to make sure the ball does not skip past the end
  onEnd_time = millis();
  if (onEnd_time - previous_onEnd_time > 250) {
    onEnd = true;
    previous_onEnd_time = onEnd_time;
  }
}

void IRAM_ATTR isr2() {  // fires when razz is detected. Does not check twice; want it to fire if touched even for a second
  razz = true;
  razzStartTime = millis();
}

String getTime() {
  currentMillis = millis();
  return String((currentMillis - intStartMillis) / 100);
}

void updateDisplay(String displayValue) {
  displaySerial.println(displayValue);  // send serial to displaySerial
}

bool playingTrack() {
  int status = myDFPlayer.readState();
  if (status == 513) {
    return true;
  } else {
    return false;
  }
}

void controlServos() {  // revisit this hot garbage
  Serial.print("X: ");
  Serial.print(map(accelData.x, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX));
  Serial.print(", Y: ");
  Serial.print(map(accelData.y, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX));

  // xVal = accelData.x;
  // if (previousXVal == 0) { previousXVal = xVal; }
  // int xDiff = previousXVal - xVal;

  // if (abs(xDiff) > MAXDIFF && xDiff > 0) {  // review this part. > or <
  //   previousXVal = xVal + MAXDIFF;
  //   pwm0 = map(previousXVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  // } else if (abs(xDiff) > MAXDIFF && xDiff > 0) {
  //   previousXVal = xVal - MAXDIFF;
  //   pwm0 = map(previousXVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  // } else {
  //   previousXVal = xVal;
  //   pwm0 = map(xVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  // }
  // pca9685.setPWM(SER0, 0, pwm0);

  // yVal = accelData.y;
  // int yDiff = previousYVal - yVal;
  // if (previousYVal == 0) { previousYVal = yVal; }

  // if (abs(yDiff) > MAXDIFF && yDiff > 0) {
  //   previousYVal = yVal + MAXDIFF;
  //   pwm1 = map(previousYVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  // } else if (abs(yDiff) > MAXDIFF && xDiff > 0) {
  //   previousYVal = yVal - MAXDIFF;
  //   pwm1 = map(previousYVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  // } else {
  //   previousYVal = yVal;
  //   pwm1 = map(yVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  // }
  // pca9685.setPWM(SER1, 0, pwm1);

  pwm0 = map(accelData.x, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  pca9685.setPWM(SER0, 0, pwm0);
  pwm1 = map(accelData.y, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  pca9685.setPWM(SER1, 0, pwm1);
}

void centerServos() {  // goal here is to keep the board from going crazy when the maze is complete
  delay(40);
  bool done = false;
  // Use pwm0/pwm1 instead of xVal/yVal
  if (pwm0 > SERVO_CENTER0) {
    for (int x = pwm0; x >= SERVO_CENTER0; x = x - 5) {
      pca9685.setPWM(SER0, 0, x);
      delay(40);
    }
  } else if (pwm0 < SERVO_CENTER0) {
    for (int x = pwm0; x <= SERVO_CENTER0; x = x + 5) {
      pca9685.setPWM(SER0, 0, x);
      delay(40);
    }
  }

  if (pwm1 > SERVO_CENTER1) {
    for (int y = pwm1; y >= SERVO_CENTER1; y = y - 5) {
      pca9685.setPWM(SER1, 0, y);
      delay(40);
    }
  } else if (pwm1 < SERVO_CENTER1) {
    for (int y = pwm1; y <= SERVO_CENTER1; y = y + 5) {
      pca9685.setPWM(SER1, 0, y);
      delay(40);
    }
  }

  pca9685.setPWM(SER0, 0, SERVO_CENTER0);
  pca9685.setPWM(SER1, 0, SERVO_CENTER1);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Turning on");                    // for debugging
  playerSerial.begin(9600, SERIAL_8N1, 16, 17);    // RX=16, TX=17
  displaySerial.begin(38400, SERIAL_8N1, 26, 27);  //only using 27 TX. No incoming messages but needed to define both
  delay(1000);
  Serial.println("Starting DFPlayer");
  if (!myDFPlayer.begin(playerSerial)) {
    Serial.println("Unable to begin DFPlayer Mini:");
    Serial.println("1. Check wiring!");
    Serial.println("2. Insert SD card!");
    while (true)
      ;
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
  attachInterrupt(STARTPIN, isr1, FALLING);
  attachInterrupt(ENDPIN, isr1, RISING);
  attachInterrupt(RAZZPIN, isr2, RISING);

  updateDisplay("Starting");
  delay(1000);
}

void loop() {

  onStart = !(digitalRead(STARTPIN));

  if (onStart) {  // ready to start
    onEnd = false;
    hasEnded = false;
    isProgressing = false;
    Serial.println("OnStart");
    // check this
    centerServos();
    updateDisplay("Ready!");

    if (playingTrack()) {
      if (myDFPlayer.readCurrentFileNumber() != 1) {
        myDFPlayer.play(1);
      }       // if currently playing track 1, do nothing. let it go
    } else {  // not currently playing
      myDFPlayer.play(1);
    }
  }

  if (!onStart && !onEnd) {  // In progress
                             // first time, start track 2
    Serial.println("InProgress");
    controlServos();
    updateDisplay(getTime());

    if (isProgressing == false) {  // start song on first in progress
      myDFPlayer.play(2);
      isProgressing = true;
    } else {  // normal non razzed in progress state - Razz is false
      if (!playingTrack()) {
        myDFPlayer.play(2);
      }
    }

    if (razz) {
      if (isRazzing == false) {  // check this
        myDFPlayer.play(4);
        isRazzing = true;
      } else {
        if (!playingTrack()) {
          razz = false;  // need to capture state so the razz sound can finish. maybe if (isRazzing == true)
          isRazzing = false;
        }
      }
    }
  }

  if (!onStart && onEnd && !hasEnded) {  // End (only once)
    Serial.println("OnEnd");
    updateDisplay(getTime());
    myDFPlayer.play(3);
    centerServos();
    hasEnded = true;
  }

  // on loops after hasEnded is true, nothing happens

  delay(40);  // need?
}
