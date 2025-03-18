/*
 * ESP32 BLE Controller with PWM Input
 * 
 * This code programs an ESP32 as a BLE flight controller that reads
 * 4 servo PWM signals from pins 4, 5, 6, and 7 and maps them to
 * X, Y, Rudder, and Throttle axes.
 */

 #include <Arduino.h>
 #include <BleGamepad.h>
 
 // BLE Controller Configuration
 #define numOfButtons 2        // Reserved for future use (pins 1 & 2)
 #define numOfHatSwitches 0    // No hat switches in this configuration
 
 // Axis Configuration - which axes to enable
 #define enableX true          // First servo PWM input (pin 4)
 #define enableY true          // Second servo PWM input (pin 5)
 #define enableZ false
 #define enableRX false
 #define enableRY false
 #define enableRZ false
 #define enableSlider1 false
 #define enableSlider2 false
 #define enableRudder true     // Third servo PWM input (pin 6)
 #define enableThrottle true   // Fourth servo PWM input (pin 7)
 #define enableAccelerator false
 #define enableBrake false
 #define enableSteering false
 
 // PWM Input Configuration
 #define numOfPWMInputs 4
 #define PWM_MIN 1000          // Minimum PWM pulse width (1000μs = 1ms)
 #define PWM_MAX 2000          // Maximum PWM pulse width (2000μs = 2ms)
 #define PWM_CENTER 1500       // Center PWM pulse width (1500μs = 1.5ms)
 
 // PWM Input Pins
 byte pwmPins[numOfPWMInputs] = {4, 5, 6, 7}; // Pins to read PWM signals from
 int pwmValues[numOfPWMInputs] = {0, 0, 0, 0}; // Current PWM values
 int previousPwmValues[numOfPWMInputs] = {0, 0, 0, 0}; // Previous PWM values
 
 // Standard axes range
 #define AXIS_MIN 0x8001      // -32767 in hex
 #define AXIS_MAX 0x7FFF      // 32767 in hex
 
 // Simulation control range (rudder, throttle, etc.)
 #define SIM_MIN -255
 #define SIM_MAX 255
 
 // Button configuration (reserved for future use)
 byte buttonPins[numOfButtons] = {1, 2};
 byte previousButtonStates[numOfButtons] = {HIGH, HIGH};
 byte currentButtonStates[numOfButtons] = {HIGH, HIGH};
 
 BleGamepad bleGamepad("BLE Flight Controller", "lemmingDev", 100);
 
 // Function to read PWM pulse width from a pin
 unsigned long readPWM(int pin) {
   // Wait for the signal to go HIGH (with timeout)
   unsigned long startWait = micros();
   while (digitalRead(pin) == LOW) {
     if (micros() - startWait > 25000) {  // 25ms timeout
       return PWM_CENTER; // Return center value if timeout
     }
   }
   
   // Start timing when signal goes HIGH
   unsigned long start = micros();
   
   // Wait for the signal to go LOW (with timeout)
   startWait = micros();
   while (digitalRead(pin) == HIGH) {
     if (micros() - startWait > 25000) {  // 25ms timeout
       return PWM_CENTER; // Return center value if timeout
     }
   }
   
   // Return the pulse width in microseconds
   return micros() - start;
 }
 
 // Map PWM (1000-2000μs) to axes value (-32767 to 32767)
 int mapPWMToAxis(unsigned long pwmValue) {
   pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
   return map(pwmValue, PWM_MIN, PWM_MAX, AXIS_MIN, AXIS_MAX);
 }
 
 // Map PWM (1000-2000μs) to simulation control value (-255 to 255)
 int mapPWMToSim(unsigned long pwmValue) {
   pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
   return map(pwmValue, PWM_MIN, PWM_MAX, SIM_MIN, SIM_MAX);
 }
 
 void setup() {
   Serial.begin(115200);
   Serial.println("Starting ESP32 BLE Flight Controller!");
 
   // Setup PWM Input Pins
   for (byte i = 0; i < numOfPWMInputs; i++) {
     pinMode(pwmPins[i], INPUT);
   }
   
   // Setup Button Pins (for future use)
   for (byte i = 0; i < numOfButtons; i++) {
     pinMode(buttonPins[i], INPUT_PULLUP);
   }
   
   // Configure BleGamepad
   BleGamepadConfiguration bleGamepadConfig;
   bleGamepadConfig.setAutoReport(false);
   bleGamepadConfig.setControllerType(CONTROLLER_TYPE_MULTI_AXIS);
   bleGamepadConfig.setButtonCount(numOfButtons);
   bleGamepadConfig.setIncludeStart(true);
   bleGamepadConfig.setIncludeSelect(true);
   
   // Configure which axes to use
   bleGamepadConfig.setWhichAxes(
     enableX, enableY, enableZ, 
     enableRX, enableRY, enableRZ, 
     enableSlider1, enableSlider2
   );
   
   // Configure which simulation controls to use
   bleGamepadConfig.setWhichSimulationControls(
     enableRudder, enableThrottle, 
     enableAccelerator, enableBrake, 
     enableSteering
   );
   
   // Configure hat switches
   bleGamepadConfig.setHatSwitchCount(numOfHatSwitches);
   
   // Configure axes range
   bleGamepadConfig.setAxesMin(AXIS_MIN);
   bleGamepadConfig.setAxesMax(AXIS_MAX);
   
   // Configure simulation controls range
   bleGamepadConfig.setSimulationMin(SIM_MIN);
   bleGamepadConfig.setSimulationMax(SIM_MAX);
   
   // Start BleGamepad
   bleGamepad.begin(&bleGamepadConfig);
   
   // Initialize all axes to center/min values
   bleGamepad.setX(0);
   bleGamepad.setY(0);
   bleGamepad.setThrottle(SIM_MIN);
   bleGamepad.setRudder(0);
   
   Serial.println("ESP32 BLE Flight Controller initialized!");
 }
 
 void loop() {
   if (bleGamepad.isConnected()) {
     bool valueChanged = false;
     
     // Read PWM values and update axes
     for (byte i = 0; i < numOfPWMInputs; i++) {
       // Read PWM value from pin
       unsigned long pwmValue = readPWM(pwmPins[i]);
       pwmValues[i] = pwmValue;
       
       // Check if value has changed significantly (to reduce unnecessary updates)
       if (abs(pwmValues[i] - previousPwmValues[i]) > 5) {
         valueChanged = true;
         previousPwmValues[i] = pwmValues[i];
       }
     }
     
     // Map PWM values to appropriate axes
     if (valueChanged) {
       // X-axis (first PWM input)
       bleGamepad.setX(mapPWMToAxis(pwmValues[0]));
       
       // Y-axis (second PWM input)
       bleGamepad.setY(mapPWMToAxis(pwmValues[1]));
       
       // Rudder (third PWM input)
       bleGamepad.setRudder(mapPWMToSim(pwmValues[2]));
       
       // Throttle (fourth PWM input)
       bleGamepad.setThrottle(mapPWMToSim(pwmValues[3]));
       
       // Send report
       bleGamepad.sendReport();
       
       // Debug output
       Serial.print("PWM Values: ");
       for (byte i = 0; i < numOfPWMInputs; i++) {
         Serial.print(pwmValues[i]);
         Serial.print(" ");
       }
       Serial.println();
     }
     
     // Read button states (commented out but preserved for future use)
     /*
     for (byte i = 0; i < numOfButtons; i++) {
       currentButtonStates[i] = digitalRead(buttonPins[i]);
       
       if (currentButtonStates[i] != previousButtonStates[i]) {
         if (currentButtonStates[i] == LOW) {
           bleGamepad.press(i + 1);
         } else {
           bleGamepad.release(i + 1);
         }
         previousButtonStates[i] = currentButtonStates[i];
         bleGamepad.sendReport();
       }
     }
     */
     
     // Small delay to reduce CPU usage
     delay(10);
   } else {
     // Not connected - wait a bit longer
     Serial.println("Waiting for BLE connection...");
     delay(500);
   }
 }
 
 /*
  * WIRING INSTRUCTIONS:
  * 
  * 1. Connect your servo PWM signals to pins 4, 5, 6, and 7:
  *    - Pin 4: X-axis (corresponds to setX in the code)
  *    - Pin 5: Y-axis (corresponds to setY in the code)
  *    - Pin 6: Rudder (corresponds to setRudder in the code)
  *    - Pin 7: Throttle (corresponds to setThrottle in the code)
  * 
  * 2. For future hardware buttons (when needed):
  *    - Connect one terminal of each button to pins 1 and 2
  *    - Connect the other terminal of each button to GND
  *    - The internal pullup resistors are enabled, so no external resistors are needed
  *    - To enable buttons, uncomment the button handling code in the loop function
  * 
  * 3. Make sure to provide appropriate power to the ESP32
  *    - USB power is usually sufficient for testing
  *    - For portable use, a 3.7V LiPo battery with appropriate regulation is recommended
  */