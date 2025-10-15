
// Uno driving one 32x16 P10 display

// Data pins wired as:
// A = 6
// B = 7
// CLK = 8
// SCK = 9
// OE = 10

// GND shared between boards

// Connect ESP32 TX (pin 27) to Uno RX (pin 0)
// You won't be able to use Serial Monitor while connected

#include <DMD2.h>
#include <fonts/SystemFont5x7.h>
#include <fonts/Arial14.h>
// #define DISPLAYS_ACROSS 1
// #define DISPLAYS_DOWN 1

// SPIDMD dmd(DISPLAYS_ACROSS, DISPLAYS_DOWN);
SoftDMD dmd(1,1);  // DMD controls the entire display
DMD_TextBox box(dmd, 0, 2);  //

char incomingMsg[50];
byte msgIndex = 0;

void setup() {
  Serial.begin(38400);  // Hardware serial - can handle 38400 reliably
  
  dmd.setBrightness(255);
  dmd.selectFont(Arial14);
  dmd.begin();
  dmd.clearScreen();
  box.print("Waiting");
  delay(1000);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (msgIndex > 0) {
        incomingMsg[msgIndex] = '\0';
        displayMessage(incomingMsg);
        msgIndex = 0;
      }
    } else if (msgIndex < sizeof(incomingMsg) - 1) {
      incomingMsg[msgIndex++] = c;
    }
  }

  //dmd.loop();
}

void displayMessage(const char *msg) {
  dmd.clearScreen();
  box.print(msg);
}
