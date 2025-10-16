// Arduino Uno driving one 32x16 P10 display
// Receives game status updates from ESP32 via Serial
//
// Hardware connections:
// P10 Display pins:
//   A = 6, B = 7, CLK = 8, SCK = 9, OE = 10
//   GND shared between Uno and ESP32
// Serial connection:
//   ESP32 TX (pin 27) → Uno RX (pin 0)
//
// Note: Cannot use Serial Monitor while ESP32 is connected

#include <DMD2.h>
#include <fonts/SystemFont5x7.h>
#include <fonts/Arial14.h>

SoftDMD dmd(1, 1);  // 1 display across, 1 down (32x16 pixels)
DMD_TextBox box(dmd);  // Text box - position set dynamically

// Display constraints
#define DISPLAY_WIDTH 32
#define DISPLAY_HEIGHT 16

// Message buffer
char incomingMsg[50];
byte msgIndex = 0;

// Timeout handling
unsigned long lastMessageTime = 0;
#define MESSAGE_TIMEOUT 5000  // 5 seconds
bool connectionLost = false;

// Display refresh
unsigned long lastRefresh = 0;
#define REFRESH_INTERVAL 20  // Refresh every 20ms for smooth display

void setup() {
  Serial.begin(38400);  // Must match ESP32 displaySerial baud rate
  
  // Initialize display
  dmd.setBrightness(255);  // Max brightness (0-255)
  dmd.selectFont(Arial14);
  dmd.begin();
  dmd.clearScreen();
  
  // Show startup message
  box.print("Starting");
  delay(1000);
  dmd.clearScreen();
  box.print("Waiting");
  
  lastMessageTime = millis();
}

void loop() {
  // Process incoming serial data
  while (Serial.available()) {
    char c = Serial.read();
    
    // Reset timeout on any received data
    lastMessageTime = millis();
    connectionLost = false;
    
    // Process message terminator
    if (c == '\n' || c == '\r') {
      if (msgIndex > 0) {
        incomingMsg[msgIndex] = '\0';  // Null terminate
        displayMessage(incomingMsg);
        msgIndex = 0;  // Reset for next message
      }
    } 
    // Add character to buffer (with overflow protection)
    else if (msgIndex < sizeof(incomingMsg) - 1) {
      incomingMsg[msgIndex++] = c;
    } 
    // Buffer overflow - discard message and reset
    else {
      msgIndex = 0;  
    }
  }
  
  // Check for connection timeout
  if (!connectionLost && (millis() - lastMessageTime > MESSAGE_TIMEOUT)) {
    connectionLost = true;
    displayConnectionLost();
  }
  
  // Refresh display periodically for smooth rendering
  if (millis() - lastRefresh >= REFRESH_INTERVAL) {
    // dmd.loop();  // Uncomment if using SPIDMD instead of SoftDMD
    lastRefresh = millis();
  }
}

void displayMessage(const char *msg) {
  dmd.clearScreen();
  
  // Center text vertically better for different message types
  // Short messages (like "Ready!") use standard position
  // Time values might benefit from centering
  if (isTimeValue(msg)) {
    // Time values - could add formatting or icons
    box.setPosY(2);
  } else {
    // Text messages
    box.setPosY(2);
  }
  
  box.print(msg);
}

void displayConnectionLost() {
  dmd.clearScreen();
  dmd.selectFont(SystemFont5x7);  // Smaller font for error message
  box.setPosY(0);
  box.print("Connection");
  box.setPosY(8);
  box.print("Lost!");
  dmd.selectFont(Arial14);  // Reset to normal font
}

// Helper function to detect if message is a time value
bool isTimeValue(const char *msg) {
  // Check if string contains a decimal point (time values like "12.3")
  for (int i = 0; msg[i] != '\0'; i++) {
    if (msg[i] == '.') return true;
  }
  return false;
}

// Optional: Add visual effects for different game states
void displayWithEffect(const char *msg) {
  // Could add blinking, scrolling, or other effects here
  // For now, just use standard display
  displayMessage(msg);
}
