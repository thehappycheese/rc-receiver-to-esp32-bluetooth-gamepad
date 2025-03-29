/*
 * ESP32 BLE Controller with Xbox Controller Spoofing and Improved PWM Reading
 * 
 * This code programs an ESP32 as a BLE gamepad that mimics an Xbox controller
 * while reading 4 servo PWM signals from pins 4, 5, 6, and 7,
 * mapping them to Left Stick X/Y and Right Stick X/Y.
 * Also supports 2 physical buttons on pins 1 and 2.
 */

 #include <Arduino.h>
 #include <BleGamepad.h>
 
 // Xbox controller has 14 standard buttons excluding special buttons
 #define XBOX_BUTTON_COUNT 14
 
 // How many buttons we'll physically implement
 #define PHYSICAL_BUTTON_COUNT 2
 
 // Standard Xbox controller button mapping
 #define BUTTON_A 1
 #define BUTTON_B 2
 #define BUTTON_X 3
 #define BUTTON_Y 4
 #define BUTTON_LB 5       // Left bumper
 #define BUTTON_RB 6       // Right bumper
 #define BUTTON_BACK 7     // Back/View button
 #define BUTTON_START 8    // Start/Menu button
 #define BUTTON_HOME 9     // Xbox button
 #define BUTTON_L3 10      // Left stick press
 #define BUTTON_R3 11      // Right stick press
 #define DPAD_UP 12        // D-pad up
 #define DPAD_DOWN 13      // D-pad down
 #define DPAD_LEFT 14      // D-pad left
 #define DPAD_RIGHT 15     // D-pad right
 
 // PWM Input Configuration
 #define numOfPWMInputs 4
 #define PWM_MIN 1000      // Minimum PWM pulse width (1000μs = 1ms)
 #define PWM_MAX 2000      // Maximum PWM pulse width (2000μs = 2ms)
 #define PWM_CENTER 1500   // Center PWM pulse width (1500μs = 1.5ms)
 #define PWM_DEADZONE 50   // Deadzone to prevent jitter (in μs)
 
 // PWM Input Pins
 byte pwmPins[numOfPWMInputs] = {4, 5, 6, 7}; // Pins to read PWM signals from
 unsigned long pwmValues[numOfPWMInputs] = {PWM_CENTER, PWM_CENTER, PWM_CENTER, PWM_CENTER}; // Current PWM values
 unsigned long previousPwmValues[numOfPWMInputs] = {PWM_CENTER, PWM_CENTER, PWM_CENTER, PWM_CENTER}; // Previous PWM values
 
 // Physical buttons
 byte buttonPins[PHYSICAL_BUTTON_COUNT] = {1, 2};
 byte previousButtonStates[PHYSICAL_BUTTON_COUNT] = {HIGH, HIGH};
 byte currentButtonStates[PHYSICAL_BUTTON_COUNT] = {HIGH, HIGH};
 // Map physical buttons to Xbox controller buttons (modify as needed)
 byte physicalToXboxButtonMap[PHYSICAL_BUTTON_COUNT] = {BUTTON_A, BUTTON_B};
 
 // Standard axes range for Xbox controller
 #define AXIS_MIN -32767      // Min axis value (defined in decimal for clarity)
 #define AXIS_MAX 32767       // Max axis value (defined in decimal for clarity)
 
 // Trigger range for Xbox controller (0 to 255)
 #define TRIGGER_MIN 0
 #define TRIGGER_MAX 255
 
 // Variables for interrupt-based PWM reading
 volatile unsigned long pwmStartTime[numOfPWMInputs] = {0, 0, 0, 0};
 volatile unsigned long pwmDuration[numOfPWMInputs] = {0, 0, 0, 0};
 volatile bool pwmUpdated[numOfPWMInputs] = {false, false, false, false};
 
 BleGamepad bleGamepad("Xbox Controller", "ESP32", 100);
 
 // PWM pin change ISRs (Interrupt Service Routines)
 void IRAM_ATTR pwmISR0() {
   if (digitalRead(pwmPins[0]) == HIGH) {
     pwmStartTime[0] = micros();
   } else {
     pwmDuration[0] = micros() - pwmStartTime[0];
     pwmUpdated[0] = true;
   }
 }
 
 void IRAM_ATTR pwmISR1() {
   if (digitalRead(pwmPins[1]) == HIGH) {
     pwmStartTime[1] = micros();
   } else {
     pwmDuration[1] = micros() - pwmStartTime[1];
     pwmUpdated[1] = true;
   }
 }
 
 void IRAM_ATTR pwmISR2() {
   if (digitalRead(pwmPins[2]) == HIGH) {
     pwmStartTime[2] = micros();
   } else {
     pwmDuration[2] = micros() - pwmStartTime[2];
     pwmUpdated[2] = true;
   }
 }
 
 void IRAM_ATTR pwmISR3() {
   if (digitalRead(pwmPins[3]) == HIGH) {
     pwmStartTime[3] = micros();
   } else {
     pwmDuration[3] = micros() - pwmStartTime[3];
     pwmUpdated[3] = true;
   }
 }
 
 // Map PWM (1000-2000μs) to axes value (-32767 to 32767)
 int mapPWMToAxis(unsigned long pwmValue) {
   // Apply deadzone around center
   if (pwmValue > PWM_CENTER - PWM_DEADZONE && pwmValue < PWM_CENTER + PWM_DEADZONE) {
     return 0;
   }
   
   // Constrain PWM value to valid range
   pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
   
   // Manual mapping calculations with long integers to avoid overflow
   long result;
   
   // For values less than center
   if (pwmValue < PWM_CENTER - PWM_DEADZONE) {
     // Map from [PWM_MIN, PWM_CENTER-DEADZONE] to [AXIS_MIN, 0]
     result = (long)(pwmValue - PWM_MIN);
     result *= (long)(0 - AXIS_MIN);
     result /= (long)((PWM_CENTER - PWM_DEADZONE) - PWM_MIN);
     result += AXIS_MIN;
     return (int)result;
   } else if (pwmValue > PWM_CENTER + PWM_DEADZONE) {
     // Map from [PWM_CENTER+DEADZONE, PWM_MAX] to [0, AXIS_MAX]
     result = (long)(pwmValue - (PWM_CENTER + PWM_DEADZONE));
     result *= (long)AXIS_MAX;
     result /= (long)(PWM_MAX - (PWM_CENTER + PWM_DEADZONE));
     return (int)result;
   } else {
     // Within deadzone (should have been caught by the first check, but just in case)
     return 0;
   }
 }
 
 // Map PWM (1000-2000μs) to trigger value (0 to 255)
 int mapPWMToTrigger(unsigned long pwmValue) {
   pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
   return map(pwmValue, PWM_MIN, PWM_MAX, TRIGGER_MIN, TRIGGER_MAX);
 }
 
 void setup() {
   Serial.begin(115200);
   Serial.println("Starting ESP32 BLE Xbox Controller!");
 
   // Setup PWM Input Pins and attach interrupts
   for (byte i = 0; i < numOfPWMInputs; i++) {
     pinMode(pwmPins[i], INPUT);
   }
   
   // Attach interrupts to PWM pins
   attachInterrupt(digitalPinToInterrupt(pwmPins[0]), pwmISR0, CHANGE);
   attachInterrupt(digitalPinToInterrupt(pwmPins[1]), pwmISR1, CHANGE);
   attachInterrupt(digitalPinToInterrupt(pwmPins[2]), pwmISR2, CHANGE);
   attachInterrupt(digitalPinToInterrupt(pwmPins[3]), pwmISR3, CHANGE);
   
   // Setup Button Pins
   for (byte i = 0; i < PHYSICAL_BUTTON_COUNT; i++) {
     pinMode(buttonPins[i], INPUT_PULLUP);
   }
   
   // Configure BleGamepad for Xbox-like controller
   BleGamepadConfiguration bleGamepadConfig;
   bleGamepadConfig.setAutoReport(false);
   bleGamepadConfig.setControllerType(CONTROLLER_TYPE_GAMEPAD);  // Xbox uses standard gamepad type
   bleGamepadConfig.setButtonCount(XBOX_BUTTON_COUNT);
   bleGamepadConfig.setIncludeStart(true);
   bleGamepadConfig.setIncludeSelect(true);  // "Select" is "Back" on Xbox
   bleGamepadConfig.setIncludeHome(true);    // Xbox button
   
   // Xbox controller has both joysticks (X, Y, Z, RZ) and triggers (RX, RY)
   bleGamepadConfig.setWhichAxes(true, true, true, true, true, true, false, false);
   
   // Configure which simulation controls to use (not needed for Xbox)
   bleGamepadConfig.setWhichSimulationControls(false, false, false, false, false);
   
   // Xbox has a D-pad which is typically implemented as a hat switch
   bleGamepadConfig.setHatSwitchCount(1);
   
   // Configure axes range (standard for Xbox)
   bleGamepadConfig.setAxesMin(AXIS_MIN);
   bleGamepadConfig.setAxesMax(AXIS_MAX);
   
   // Start BleGamepad
   bleGamepad.begin(&bleGamepadConfig);
   
   // Initialize all axes to center values
   bleGamepad.setX(0);        // Left stick X
   bleGamepad.setY(0);        // Left stick Y
   bleGamepad.setZ(0);        // Right stick X
   bleGamepad.setRX(0);       // Left trigger
   bleGamepad.setRY(0);       // Right trigger
   bleGamepad.setRZ(0);       // Right stick Y
   bleGamepad.setHat1(0);     // D-pad centered
   
   Serial.println("ESP32 BLE Xbox Controller initialized!");
   Serial.println("PWM pins configured for interrupt-based reading");
 }
 
 void loop() {
   if (bleGamepad.isConnected()) {
     bool valueChanged = false;
     
     // Process PWM readings from interrupts
     for (byte i = 0; i < numOfPWMInputs; i++) {
       // Check if PWM value was updated by interrupt
       if (pwmUpdated[i]) {
         // Get the PWM value that was measured by the interrupt
         pwmValues[i] = pwmDuration[i];
         pwmUpdated[i] = false;
         
         // Validate PWM value (reject extremes that might be errors)
         if (pwmValues[i] < 500 || pwmValues[i] > 2500) {
           // Invalid PWM value, use previous valid value
           pwmValues[i] = previousPwmValues[i];
         }
         
         // Check if value has changed significantly
         if (abs((long)pwmValues[i] - (long)previousPwmValues[i]) > 10) {
           valueChanged = true;
           previousPwmValues[i] = pwmValues[i];
           
           // Debug output 
           Serial.print("PWM ");
           Serial.print(i);
           Serial.print(": ");
           Serial.print(pwmValues[i]);
           Serial.print(" μs, Mapped: ");
           int mappedValue = mapPWMToAxis(pwmValues[i]);
           Serial.println(mappedValue);
         }
       }
     }
     
     // Map PWM values to Xbox controller axes
     if (valueChanged) {
       // Map first 4 PWM inputs to controller axes
       int L_x = mapPWMToAxis(pwmValues[2]);
       int L_y = -mapPWMToAxis(pwmValues[3]);
       int R_x = mapPWMToAxis(pwmValues[0]);
       int R_y = -mapPWMToAxis(pwmValues[1]);
       
       // NOTE: mapping to xbox axes doesn't make much sense reading the axis names below:
       bleGamepad.setX(0);
       bleGamepad.setY(0);
       bleGamepad.setZ(R_x);
       bleGamepad.setRX(L_x);
       bleGamepad.setRY(L_y);
       bleGamepad.setRZ(R_y);

       
       // Send report
       bleGamepad.sendReport();
     }
     
     // Read physical button states
     bool buttonChanged = false;
     for (byte i = 0; i < PHYSICAL_BUTTON_COUNT; i++) {
       currentButtonStates[i] = digitalRead(buttonPins[i]);
       
       if (currentButtonStates[i] != previousButtonStates[i]) {
         buttonChanged = true;
         if (currentButtonStates[i] == LOW) {
           // Button pressed - map to configured Xbox button
           bleGamepad.press(physicalToXboxButtonMap[i]);
           Serial.print("Button ");
           Serial.print(i + 1);
           Serial.print(" pressed, mapped to Xbox button ");
           Serial.println(physicalToXboxButtonMap[i]);
         } else {
           // Button released
           bleGamepad.release(physicalToXboxButtonMap[i]);
           Serial.print("Button ");
           Serial.print(i + 1);
           Serial.println(" released");
         }
         previousButtonStates[i] = currentButtonStates[i];
       }
     }
     
     // Send report if button state changed
     if (buttonChanged) {
       bleGamepad.sendReport();
     }
     
     // Small delay to reduce CPU usage
     delay(5);
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
  *    - Pin 4: Left Stick X-axis (corresponds to setX in the code)
  *    - Pin 5: Left Stick Y-axis (corresponds to setY in the code, inverted)
  *    - Pin 6: Right Stick X-axis (corresponds to setZ in the code)
  *    - Pin 7: Right Stick Y-axis (corresponds to setRZ in the code, inverted)
  * 
  * 2. For hardware buttons:
  *    - Connect one terminal of button 1 to pin 1 (maps to Xbox A button)
  *    - Connect one terminal of button 2 to pin 2 (maps to Xbox B button)
  *    - Connect the other terminal of each button to GND
  *    - The internal pullup resistors are enabled, so no external resistors are needed
  * 
  * 3. Make sure to provide appropriate power to the ESP32
  *    - USB power is usually sufficient for testing
  *    - For portable use, a 3.7V LiPo battery with appropriate regulation is recommended
  *
  * NOTE: Make sure the pins you are using (4, 5, 6, 7) support interrupts on the ESP32.
  * If you have issues, check which pins on your specific ESP32 board support interrupts
  * and modify the pwmPins array accordingly.
  */