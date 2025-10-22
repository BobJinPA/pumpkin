// Arduino Uno driving one 32x16 P10 display
// Receives game status updates from ESP32 via Serial

#include <DMD2.h>
#include <fonts/SystemFont5x7.h>
#include <fonts/Arial14.h>

SoftDMD dmd(1, 1);  // 1 display across, 1 down (32x16 pixels)

// Display constraints
#define DISPLAY_WIDTH 32
#define DISPLAY_HEIGHT 16

// Message buffer with generous size
char incomingMsg[64];
byte msgIndex = 0;

// Timeout handling
unsigned long lastMessageTime = 0;
#define MESSAGE_TIMEOUT 5000  // 5 seconds
bool connectionLost = false;

// Current display state tracking
char currentDisplay[64] = "";

// Display refresh
unsigned long lastRefresh = 0;
#define REFRESH_INTERVAL 50  // 50ms for stable refresh

// Forward declarations
bool isTimeValue(const char *msg);
int16_t getTextWidth(const char *text, const uint8_t *font);
void displayMessage(const char *msg);
void displayConnectionLost();

void setup() {
  Serial.begin(38400);  // Must match ESP32 displaySerial baud rate
  
  // Initialize display
  dmd.setBrightness(255);  // Max brightness (0-255)
  dmd.selectFont(Arial14);
  dmd.begin();
  dmd.clearScreen();
  
  // Show startup message centered
  displayMessage("START");
  
  lastMessageTime = millis();
}

void loop() {
  // Process incoming serial data
  while (Serial.available()) {
    char c = Serial.read();
    
    // Reset timeout on any received data
    lastMessageTime = millis();
    if (connectionLost) {
      connectionLost = false;
      // Force refresh to clear error message
      displayMessage(currentDisplay);
    }
    
    // Process message terminator
    if (c == '\n' || c == '\r') {
      if (msgIndex > 0) {
        incomingMsg[msgIndex] = '\0';  // Null terminate
        
        // Only update display if message changed (reduces flicker)
        if (strcmp(incomingMsg, currentDisplay) != 0) {
          displayMessage(incomingMsg);
          strncpy(currentDisplay, incomingMsg, sizeof(currentDisplay) - 1);
          currentDisplay[sizeof(currentDisplay) - 1] = '\0';
        }
        
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
    currentDisplay[0] = '\0';  // Clear cached display state
  }
  
  // Refresh display periodically
  if (millis() - lastRefresh >= REFRESH_INTERVAL) {
    // SoftDMD handles refresh internally, but calling scanDisplay() 
    // ensures smooth rendering
    dmd.scanDisplay();
    lastRefresh = millis();
  }
}

void displayMessage(const char *msg) {
  dmd.clearScreen();
  
  // Choose font based on message length
  const uint8_t *selectedFont;
  int16_t textHeight;
  int16_t estimatedWidth = strlen(msg) * 8;  // Quick estimate for Arial14
  
  // Use smaller font if text is too long for Arial14
  if (estimatedWidth > DISPLAY_WIDTH) {
    selectedFont = SystemFont5x7;
    textHeight = 7;
  } else {
    selectedFont = Arial14;
    textHeight = 14;
  }
  
  dmd.selectFont(selectedFont);
  
  // Calculate actual text width
  int16_t textWidth = getTextWidth(msg, selectedFont);
  
  // If still too wide, truncate or use even smaller positioning
  if (textWidth > DISPLAY_WIDTH) {
    // For time values, just use left-aligned to ensure visibility
    if (isTimeValue(msg)) {
      int16_t y = (DISPLAY_HEIGHT - textHeight) / 2;
      if (y < 0) y = 0;
      dmd.drawString(0, y, msg);  // Left-aligned
      return;
    }
    
    // For text, try to center what fits
    textWidth = DISPLAY_WIDTH;  // Cap at display width
  }
  
  // Center horizontally and vertically
  int16_t x = (DISPLAY_WIDTH - textWidth) / 2;
  int16_t y = (DISPLAY_HEIGHT - textHeight) / 2;
  
  // Ensure non-negative positions
  if (x < 0) x = 0;
  if (y < 0) y = 0;
  
  // Draw the text
  dmd.drawString(x, y, msg);
}

void displayConnectionLost() {
  // dmd.clearScreen();
  // dmd.selectFont(SystemFont5x7);  // Smaller font for multi-line error
  
  // // Line 1: "NO" (shorter message)
  // dmd.drawString(8, 1, "NO");
  
  // // Line 2: "LINK"
  // dmd.drawString(4, 9, "LINK");
}

// Helper function to detect if message is a time value
bool isTimeValue(const char *msg) {
  // Check if string contains a decimal point (time values like "12.3")
  // and only digits otherwise
  bool hasDecimal = false;
  bool hasDigit = false;
  
  for (int i = 0; msg[i] != '\0'; i++) {
    if (msg[i] == '.') {
      hasDecimal = true;
    } else if (isdigit(msg[i])) {
      hasDigit = true;
    } else if (msg[i] != ' ') {
      // Non-digit, non-decimal, non-space = not a time value
      return false;
    }
  }
  
  return hasDecimal && hasDigit;
}

// Helper to calculate text width for centering
// Improved accuracy for 32px display
int16_t getTextWidth(const char *text, const uint8_t *font) {
  int16_t width = 0;
  
  // For Arial14: More conservative measurements
  if (font == Arial14) {
    for (int i = 0; text[i] != '\0'; i++) {
      char c = text[i];
      if (c >= '0' && c <= '9') {
        width += 7;  // Digits
      } else if (c == '.') {
        width += 3;  // Decimal point
      } else if (c == ' ') {
        width += 4;  // Space
      } else if (c == '!') {
        width += 3;  // Exclamation mark
      } else if (c >= 'A' && c <= 'Z') {
        width += 8;  // Uppercase letters
      } else if (c >= 'a' && c <= 'z') {
        width += 7;  // Lowercase letters
      } else {
        width += 7;  // Other characters
      }
    }
  }
  // For SystemFont5x7: Fixed width font
  else if (font == SystemFont5x7) {
    width = strlen(text) * 6;  // 5px + 1px spacing
  }
  
  return width;
}
