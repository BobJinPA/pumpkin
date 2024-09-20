#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---  DISPLAY SET UP ----
//change from SSD1306 when changing display
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

typedef struct struct_message {
  int x;
  int y;
  int z;
} struct_message;

struct_message accelData;

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 290  // Minimum value
#define SERVOMAX 390  // Maximum value
#define SER0 0        //Servo Motor 0 on connector 0
#define SER1 1        //Servo Motor 1 on connector 1
int pwm0;
int pwm1;

//if smoothing works
#define MAXDIFF 15
int xVal;
int yVal;
int previousXVal = 0;
int previousYVal = 0;

const int SENSOR_RANGE = 50;

#define STARTPIN 18
#define ENDPIN 19


// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&accelData, incomingData, sizeof(accelData));
}

//variables to keep track of the timing of recent interrupts
unsigned long button1_time = 0;
unsigned long last_button1_time = 0;
unsigned long button2_time = 0;
unsigned long last_button2_time = 0;
int intStartMillis;
int currentMillis;
bool inProgress;
bool onStart = false;
bool onEnd = false;

void IRAM_ATTR isr() {
  button2_time = millis();
  if (button2_time - last_button2_time > 250) {
    onEnd = true;
    last_button2_time = button2_time;
  }
}

float getTime() {
  currentMillis = millis();
  return (currentMillis - intStartMillis) / 100;
}

void updateDisplay(String displayValue) {
  display.clearDisplay();
  display.setCursor(0, 10);
  display.println(displayValue);
  display.display();
}

void displaySetup() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {  // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
}

// // this is current
// void controlServos() {
//   Serial.print("X: ");
//   Serial.print(map(accelData.x, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX));
//   Serial.print(", Y: ");
//   Serial.println(map(accelData.y, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX));

//   //Could add a limit to change of accelData.x value to reduce shakiness
//   pwm0 = map(accelData.x, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
//   pca9685.setPWM(SER0, 0, pwm0);

//   pwm1 = map(accelData.y, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
//   pca9685.setPWM(SER1, 0, pwm1);
// }


// experiment with smoothing
void controlServos() {
  Serial.print("X: ");
  Serial.print(map(accelData.x, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX));
  Serial.print(", Y: ");
  Serial.print(map(accelData.y, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX));

  xVal = accelData.x;
  if (previousXVal == 0) { previousXVal = xVal; }
  int xDiff = previousXVal - xVal;

  if (abs(xDiff) > MAXDIFF && xDiff > 0) {
    previousXVal = xVal + MAXDIFF;
    pwm0 = map(previousXVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  } else if (abs(xDiff) > MAXDIFF && xDiff > 0) {
    previousXVal = xVal - MAXDIFF;
    pwm0 = map(previousXVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  } else {
    previousXVal = xVal;
    pwm0 = map(xVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  }
  pca9685.setPWM(SER0, 0, pwm0);

  yVal = accelData.y;
  int yDiff = previousYVal - yVal;
  if (previousYVal == 0) { previousYVal = yVal; }

  if (abs(yDiff) > MAXDIFF && yDiff > 0) {
    previousYVal = yVal + MAXDIFF;
    pwm1 = map(previousYVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  } else if (abs(yDiff) > MAXDIFF && xDiff > 0) {
    previousYVal = yVal - MAXDIFF;
    pwm1 = map(previousXVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  } else {
    previousYVal = yVal;
    pwm1 = map(yVal, (SENSOR_RANGE * -1), SENSOR_RANGE, SERVOMIN, SERVOMAX);
  }
  pca9685.setPWM(SER1, 0, pwm1);
}

void setup() {
  Serial.begin(115200);
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
  attachInterrupt(ENDPIN, isr, RISING);

  displaySetup();  //to update when changing display type
}

void loop() {

  onStart = digitalRead(STARTPIN);  //flipped because of pullup resister
  Serial.print("OnStart val: ");
  Serial.println(onStart);
  if (!onStart) {
    onEnd = false;
    Serial.println("OnStart");
    intStartMillis = millis();
    updateDisplay("Ready to Start!");
    controlServos();
  }

  if (onStart && !onEnd) {  // in progress
    Serial.println("InProgress");
    controlServos();
    updateDisplay(String(getTime()));
  }

  if (onStart && onEnd) {  // end
    Serial.println("OnEnd");
    updateDisplay(String((currentMillis - intStartMillis) / 100));  // stall motors by not calling controlServos()
  }

  delay(40);
}
