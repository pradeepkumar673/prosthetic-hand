#include <Servo.h>

// Create servo objects
Servo thumbServo;
Servo thumb2Servo;  // Second thumb servo
Servo indexServo;
Servo middleServo;
Servo ringServo;
Servo pinkyServo;

// Pin definitions - MOVED THUMB_PIN from 3 to 4 to avoid Serial conflicts
const int THUMB_PIN = 4;     // Changed from 3 to avoid Serial conflicts
const int THUMB2_PIN = 11;   // Pin for second thumb servo
const int INDEX_PIN = 5;
const int MIDDLE_PIN = 6;
const int RING_PIN = 9;
const int PINKY_PIN = 10;
const int BUTTON_PIN = A0;

// Servo positions - Thumb now uses 30° range with inverted direction
const int SERVO_ON = 180;    // Closed position for fingers
const int SERVO_OFF = 0;     // Open position for fingers
const int THUMB_ON = 0;      // Closed position for thumb (inverted)
const int THUMB_OFF = 40;    // Open position for thumb (30° range)

const int PACKET_LENGTH = 6; // $ + 5 digits
const int DEBOUNCE_DELAY = 50;
const bool DEBUG = true;

// Servo controller structure
struct ServoController {
  int thumbPos = THUMB_OFF;  // Start with thumb open
  int thumb2Pos = 0;         // Will be calculated based on thumb position
  int indexPos = SERVO_OFF;
  int middlePos = SERVO_OFF;
  int ringPos = SERVO_OFF;
  int pinkyPos = SERVO_OFF;
  
  int prevThumbPos = -1;
  int prevThumb2Pos = -1;    // Track previous thumb2 position
  int prevIndexPos = -1;
  int prevMiddlePos = -1;
  int prevRingPos = -1;
  int prevPinkyPos = -1;
};

ServoController hand;
bool portableMode = false;
unsigned long lastButtonRead = 0;

void setup() {
  Serial.begin(9600);
  
  // Attach servos to pins
  thumbServo.attach(THUMB_PIN);
  thumb2Servo.attach(THUMB2_PIN);  // Attach second thumb servo
  indexServo.attach(INDEX_PIN);
  middleServo.attach(MIDDLE_PIN);
  ringServo.attach(RING_PIN);
  pinkyServo.attach(PINKY_PIN);
  
  // Set all servos to off initially
  allServosOff();
  
  // Determine operation mode based on button press at startup
  portableMode = (debouncedButtonRead() == 0);
  
  if (DEBUG) {
    Serial.print("Starting in ");
    Serial.print(portableMode ? "PORTABLE" : "SERIAL");
    Serial.println(" mode");
    Serial.println("Send data as: $10101 (thumb,index,middle,ring,pinky)");
    Serial.println("Servos Initialized");
    Serial.println("Thumb: inverted direction, 30° range");
    Serial.print("Thumb servo on pin: "); Serial.println(THUMB_PIN);
    Serial.print("Thumb2 servo on pin: "); Serial.println(THUMB2_PIN);
  }
}

void loop() {
  if (portableMode) {
    // Portable mode: button controls all servos together
    if (debouncedButtonRead() == 0) {
      allServosOff();
    } else {
      allServosOn();
    }
  } else {
    // Serial mode: receive and parse commands
    if (receiveAndParseData()) {
      updateServos(); // Only update if positions changed
    }
  }
}

/**
 * Reads button with debouncing to prevent noise
 * Returns: 0 if pressed, 1 if not pressed
 */
int debouncedButtonRead() {
  unsigned long currentTime = millis();
  if (currentTime - lastButtonRead < DEBOUNCE_DELAY) {
    return analogRead(BUTTON_PIN) == 0 ? 0 : 1;
  }
  
  lastButtonRead = currentTime;
  int buttonState = analogRead(BUTTON_PIN);
  return (buttonState == 0) ? 0 : 1;
}

/**
 * Parses serial data in format: $10101
 * Where each digit represents servo state (1=on, 0=off)
 * Returns: true if valid data was parsed and servos need update
 */
bool receiveAndParseData() {
  static char buffer[PACKET_LENGTH + 1];
  static int bufferIndex = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    // Start of packet
    if (c == '$') {
      bufferIndex = 0;
      buffer[bufferIndex++] = c;
      continue;
    }
    
    // Continue filling buffer if we're in a packet
    if (bufferIndex > 0 && bufferIndex < PACKET_LENGTH) {
      // Validate digit is 0 or 1
      if (c == '0' || c == '1') {
        buffer[bufferIndex++] = c;
      } else {
        // Invalid character, discard packet
        if (DEBUG) Serial.println("Error: Invalid character in packet");
        bufferIndex = 0;
        continue;
      }
    }
    
    // Complete packet received
    if (bufferIndex == PACKET_LENGTH) {
      buffer[bufferIndex] = '\0'; // Null terminate
      
      // Parse servo positions - thumb uses special inverted positions
      hand.thumbPos = (buffer[1] == '1') ? THUMB_ON : THUMB_OFF;
      hand.indexPos = (buffer[2] == '1') ? SERVO_ON : SERVO_OFF;
      hand.middlePos = (buffer[3] == '1') ? SERVO_ON : SERVO_OFF;
      hand.ringPos = (buffer[4] == '1') ? SERVO_ON : SERVO_OFF;
      hand.pinkyPos = (buffer[5] == '1') ? SERVO_ON : SERVO_OFF;
      
      // Calculate thumb2 position based on thumb position
      hand.thumb2Pos = calculateThumb2Pos(hand.thumbPos);
      
      if (DEBUG) {
        Serial.print("Parsed: ");
        Serial.print(buffer);
        Serial.print(" -> Thumb:");
        Serial.print(hand.thumbPos);
        Serial.print(" Thumb2:");
        Serial.print(hand.thumb2Pos);
        Serial.print(" Index:");
        Serial.print(hand.indexPos);
        Serial.print(" Middle:");
        Serial.print(hand.middlePos);
        Serial.print(" Ring:");
        Serial.print(hand.ringPos);
        Serial.print(" Pinky:");
        Serial.println(hand.pinkyPos);
      }
      
      bufferIndex = 0;
      return true;
    }
  }
  return false;
}

/**
 * Calculate thumb2 position based on thumb position
 * Maps thumb's 0-40° range to thumb2's 0-180° range
 */
int calculateThumb2Pos(int thumbPos) {
  return map(thumbPos, 0, 40, 0, 180);
}

/**
 * Updates servo positions only if they have changed
 * Improves efficiency and reduces servo jitter
 */
void updateServos() {
  // Always calculate thumb2 position based on current thumb position
  hand.thumb2Pos = calculateThumb2Pos(hand.thumbPos);
  
  if (hand.thumbPos != hand.prevThumbPos) {
    thumbServo.write(hand.thumbPos);
    hand.prevThumbPos = hand.thumbPos;
    if (DEBUG) {
      Serial.print("Thumb moved to: "); 
      Serial.println(hand.thumbPos);
    }
  }
  
  // Only update thumb2 if its calculated position has changed
  if (hand.thumb2Pos != hand.prevThumb2Pos) {
    thumb2Servo.write(hand.thumb2Pos);
    hand.prevThumb2Pos = hand.thumb2Pos;
    if (DEBUG) {
      Serial.print("Thumb2 moved to: "); 
      Serial.println(hand.thumb2Pos);
    }
  }
  
  if (hand.indexPos != hand.prevIndexPos) {
    indexServo.write(hand.indexPos);
    hand.prevIndexPos = hand.indexPos;
  }
  if (hand.middlePos != hand.prevMiddlePos) {
    middleServo.write(hand.middlePos);
    hand.prevMiddlePos = hand.middlePos;
  }
  if (hand.ringPos != hand.prevRingPos) {
    ringServo.write(hand.ringPos);
    hand.prevRingPos = hand.ringPos;
  }
  if (hand.pinkyPos != hand.prevPinkyPos) {
    pinkyServo.write(hand.pinkyPos);
    hand.prevPinkyPos = hand.pinkyPos;
  }
}

/**
 * Sets all servos to maximum position (closed grip)
 */
void allServosOn() {
  hand.thumbPos = THUMB_ON;    // Thumb closed (inverted)
  hand.indexPos = SERVO_ON;
  hand.middlePos = SERVO_ON;
  hand.ringPos = SERVO_ON;
  hand.pinkyPos = SERVO_ON;
  updateServos();
}

/**
 * Sets all servos to minimum position (open grip)
 */
void allServosOff() {
  hand.thumbPos = THUMB_OFF;   // Thumb open (inverted)
  hand.indexPos = SERVO_OFF;
  hand.middlePos = SERVO_OFF;
  hand.ringPos = SERVO_OFF;
  hand.pinkyPos = SERVO_OFF;
  updateServos();
}