/*
 * ESP32 BLE Controller with Xbox Controller Spoofing
 * 
 * This code programs an ESP32 as a BLE gamepad that mimics an Xbox controller
 * while actually reading 4 servo PWM signals from pins 4, 5, 6, and 7,
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
 
 // PWM Input Pins
 byte pwmPins[numOfPWMInputs] = {4, 5, 6, 7}; // Pins to read PWM signals from
 int pwmValues[numOfPWMInputs] = {0, 0, 0, 0}; // Current PWM values
 int previousPwmValues[numOfPWMInputs] = {0, 0, 0, 0}; // Previous PWM values
 
 // Physical buttons
 byte buttonPins[PHYSICAL_BUTTON_COUNT] = {1, 2};
 byte previousButtonStates[PHYSICAL_BUTTON_COUNT] = {HIGH, HIGH};
 byte currentButtonStates[PHYSICAL_BUTTON_COUNT] = {HIGH, HIGH};
 // Map physical buttons to Xbox controller buttons (modify as needed)
 byte physicalToXboxButtonMap[PHYSICAL_BUTTON_COUNT] = {BUTTON_A, BUTTON_B};
 
 // Standard axes range for Xbox controller
 #define AXIS_MIN 0x8001      // -32767 in hex
 #define AXIS_MAX 0x7FFF      // 32767 in hex
 
 // Trigger range for Xbox controller (0 to 255)
 #define TRIGGER_MIN 0
 #define TRIGGER_MAX 255
 
 BleGamepad bleGamepad("Xbox Controller", "ESP32", 100);
 
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
 
 // Map PWM (1000-2000μs) to trigger value (0 to 255)
 int mapPWMToTrigger(unsigned long pwmValue) {
   pwmValue = constrain(pwmValue, PWM_MIN, PWM_MAX);
   return map(pwmValue, PWM_MIN, PWM_MAX, TRIGGER_MIN, TRIGGER_MAX);
 }
 
 void setup() {
   Serial.begin(115200);
   Serial.println("Starting ESP32 BLE Xbox Controller!");
 
   // Setup PWM Input Pins
   for (byte i = 0; i < numOfPWMInputs; i++) {
     pinMode(pwmPins[i], INPUT);
   }
   
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
   bleGamepad.setRZ(0);       // Right stick Y
   bleGamepad.setRX(0);       // Left trigger
   bleGamepad.setRY(0);       // Right trigger
   bleGamepad.setHat1(0);     // D-pad centered
   
   Serial.println("ESP32 BLE Xbox Controller initialized!");
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
     
     // Map PWM values to Xbox controller axes
     if (valueChanged) {
       // Map first 4 PWM inputs to controller axes
       // You can adjust this mapping based on your needs
       
       // Left Stick X (first PWM input)
       bleGamepad.setX(mapPWMToAxis(pwmValues[0]));
       
       // Left Stick Y (second PWM input)
       bleGamepad.setY(mapPWMToAxis(pwmValues[1]));
       
       // Right Stick X (third PWM input)
       bleGamepad.setZ(mapPWMToAxis(pwmValues[2]));
       
       // Right Stick Y (fourth PWM input)
       bleGamepad.setRZ(mapPWMToAxis(pwmValues[3]));
       
       // NOTE: Not physically reading triggers, setting them to 0
       bleGamepad.setRX(0); // Left trigger
       bleGamepad.setRY(0); // Right trigger
       
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
  *    - Pin 4: Left Stick X-axis (corresponds to setX in the code)
  *    - Pin 5: Left Stick Y-axis (corresponds to setY in the code)
  *    - Pin 6: Right Stick X-axis (corresponds to setZ in the code)
  *    - Pin 7: Right Stick Y-axis (corresponds to setRZ in the code)
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
  */