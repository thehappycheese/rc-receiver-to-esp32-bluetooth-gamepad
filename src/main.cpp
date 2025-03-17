#include <Arduino.h>

// Pin definitions
#define RC_PIN_A 7  // RC servo input pin - Horizontal
#define RC_PIN_B 15  // RC servo input pin - Vertical

// Variables to store RC servo readings for channel A (Horizontal)
volatile unsigned long pulseStartTimeA = 0;
volatile unsigned long pulseWidthA = 0;
volatile boolean newPulseAvailableA = false;

// Variables to store RC servo readings for channel B (Vertical)
volatile unsigned long pulseStartTimeB = 0;
volatile unsigned long pulseWidthB = 0;
volatile boolean newPulseAvailableB = false;

// Interrupt service routine for RC pulse measurement - Channel A
void IRAM_ATTR rcInterruptA() {
  // If the pin is high, it's the start of the pulse
  if (digitalRead(RC_PIN_A) == HIGH) {
    pulseStartTimeA = micros();
  } else {
    // If the pin is low, it's the end of the pulse
    if (pulseStartTimeA != 0) {
      pulseWidthA = micros() - pulseStartTimeA;
      newPulseAvailableA = true;
    }
  }
}

// Interrupt service routine for RC pulse measurement - Channel B
void IRAM_ATTR rcInterruptB() {
  // If the pin is high, it's the start of the pulse
  if (digitalRead(RC_PIN_B) == HIGH) {
    pulseStartTimeB = micros();
  } else {
    // If the pin is low, it's the end of the pulse
    if (pulseStartTimeB != 0) {
      pulseWidthB = micros() - pulseStartTimeB;
      newPulseAvailableB = true;
    }
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  delay(1000); // Give the serial monitor time to connect
  
  Serial.println("ESP32-S3 Dual RC Channel Reader");
  Serial.println("Reading horizontal on pin 7, vertical on pin 8");
  
  // Configure the RC pins as input
  pinMode(RC_PIN_A, INPUT);
  pinMode(RC_PIN_B, INPUT);
  
  // Attach interrupts to the RC pins for both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(RC_PIN_A), rcInterruptA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RC_PIN_B), rcInterruptB, CHANGE);
}

void loop() {
  // Local variables to store calculated percentages
  int percentA = 0;
  int percentB = 0;
  
  // Process new pulse from channel A if available
  if (newPulseAvailableA) {
    // Calculate percentage (assuming standard 1000-2000μs range)
    percentA = map(pulseWidthA, 1000, 2000, 0, 100);
    // Constrain to valid range
    percentA = constrain(percentA, 0, 100);
    // Reset the flag
    newPulseAvailableA = false;
  }
  
  // Process new pulse from channel B if available
  if (newPulseAvailableB) {
    // Calculate percentage (assuming standard 1000-2000μs range)
    percentB = map(pulseWidthB, 1000, 2000, 0, 100);
    // Constrain to valid range
    percentB = constrain(percentB, 0, 100);
    // Reset the flag
    newPulseAvailableB = false;
  }
  
  // Print the values in the requested format
  Serial.print("A: ");
  Serial.print(percentA);
  Serial.print(" %    ");
  
  Serial.print("B: ");
  Serial.print(percentB);
  Serial.println(" %");
  
  // Short delay to prevent flooding the serial monitor
  delay(100);
}