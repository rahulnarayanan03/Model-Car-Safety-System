// All Definitions
// Hashtags
//For Servo
#include <Servo.h>
//For LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
// Pin and Time Definitions
const int left_button = 13;
const int left_LED = 12;
const int left_bs_R = 11;
const int left_bs_G = 10;
const int trigPin_left_bsm = 28;
const int echoPin_left_bsm = 29;

const int P_button = 8;
const int R_button = 7;
const int D_button = 6;

const int right_button = 5;
const int right_LED = 4;
const int right_bs_R = 26;
const int right_bs_G = 27;
const int trigPin_right_bsm = 24;
const int echoPin_right_bsm = 25;

const int trigPin_back = 32;
const int echoPin_back = 33;

const int trigPin_front_FCA = 34;
const int echoPin_front_FCA = 35;

const int trigPin_front_AF_Right = 36;
const int echoPin_front_AF_Right = 37;
const int trigPin_front_AF_Left = 39;
const int echoPin_front_AF_Left = 38;

const int trigPin_front_AUF_Right = 40;
const int echoPin_front_AUF_Right = 41;
const int trigPin_front_AUF_Left = 42;
const int echoPin_front_AUF_Left = 43;

const int analogInPin = A0;

const int buttonSelectPin = 22;
const int buttonSubmitPin = 23;

const int Buzzer_1 = 30;
const int Buzzer_2 = 31;

const unsigned long Second = 1000;

// States
int state = 1;

// Indicator Bools
bool rightLEDFlashing = false;      // Tracks if the right LED is flashing
bool leftLEDFlashing = false;       // Tracks if the left LED is flashing
bool rightButtonState = false;      // Tracks the current state of the right button
bool leftButtonState = false;       // Tracks the current state of the left button
bool lastRightButtonState = false;  // Tracks the previous state of the right button
bool lastLeftButtonState = false;   // Tracks the previous state of the left button

// Password
const String password = "62442";       // Define the password
bool systemUnlocked = false;           // Track if the system is unlocked
String enteredPin = "";                // Store the 4-digit entered PIN
int digitIndex = 0;                    // Track the number of entered digits
unsigned long pot_PreviousMillis = 0;  // Last time the potentiometer was read
const long pot_Interval = 300;         // 300ms interval for potentiometer updates
unsigned long debounceMillis = 0;      // Track last debounce time
const long debounceDelay = 50;         // 50ms debounce delay

unsigned long messageMillis = 0;    // Time when message display started
const long messageDuration = 1000;  // 3 second to show the message

// Variables for Double Press Detection
bool firstPressDetected = false;      // Track if the first press was detected
bool secondPressDetected = false;     // Track if the second press was detected
bool buttonCurrentlyPressed = false;  // Track if the button is currently pressed
bool isButtonHeld = false;

unsigned long firstPressMillis = 0;    // Time of the first press
unsigned long secondPressMillis = 0;   // Time of the second press
const long doublePressInterval = 500;  // Max interval between two presses for double press

unsigned long debounceMillis_lock = 0;  // Debounce timer for the lock button
const long debounceDelay_lock = 50;     // Debounce delay for the lock button

// Flags for button state tracking
bool selectButtonPressed = false;      // Track if Select button was pressed
bool submitButtonPressed = false;      // Track if Submit button was pressed
bool showingMessage = false;           // Track if a message is being displayed
bool isDisplayingLockMessage = false;  // Flag to indicate if the lock message is being displayed

// Servo Side View Mirrors
// Right Side
Servo myservo_1;
int servoPos_1 = 0;                                          // stores the servo position
unsigned long servoMoveMillis_prev_1 = 0;                    // Store last movement time
unsigned long servoMoveMillis_interval_speedcontrol_1 = 25;  // 25 ms between movements

// Left Side
Servo myservo_2;
int servoPos_2 = 0;                                          // stores the servo position
unsigned long servoMoveMillis_prev_2 = 0;                    // Store last movement time
unsigned long servoMoveMillis_interval_speedcontrol_2 = 25;  // 25 ms between movements

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Potmeter
int sensorValue = 0;
int mappedValue = 0;

// U/S sensor variables
const float SOUND_SPEED = 0.0343;                                        // cm per microsecond
const unsigned long MAX_DISTANCE_CM = 400;                               // Maximum range of HC-SR04 in cm
const unsigned long MAX_DURATION = (MAX_DISTANCE_CM * 2) / SOUND_SPEED;  // Max pulse duration

// Buzzer
unsigned long previousMillis_beep = 0;
unsigned long previousMillis_beep_both = 0;
const int beepDuration = 1000;  // Duration for the beep in milliseconds
const int beepInterval = 300;   // Total interval for beep on/off in milliseconds

// Mirror Fold/Unfold
bool isAutoFoldActive = false;  // Flag to track if auto-folding is active
bool holdRightClosed = false;   // Flag to hold right mirror closed
bool holdLeftClosed = false;    // Flag to hold left mirror closed

void setup() {
  pinMode(P_button, INPUT);
  pinMode(R_button, INPUT);
  pinMode(D_button, INPUT);

  pinMode(right_LED, OUTPUT);
  pinMode(right_button, INPUT);
  pinMode(left_LED, OUTPUT);
  pinMode(left_button, INPUT);

  pinMode(buttonSelectPin, INPUT);
  pinMode(buttonSubmitPin, INPUT);

  pinMode(trigPin_right_bsm, OUTPUT);
  pinMode(echoPin_right_bsm, INPUT);

  pinMode(trigPin_left_bsm, OUTPUT);
  pinMode(echoPin_left_bsm, INPUT);

  pinMode(trigPin_back, OUTPUT);
  pinMode(echoPin_back, INPUT);

  pinMode(trigPin_front_FCA, OUTPUT);
  pinMode(echoPin_front_FCA, INPUT);

  pinMode(trigPin_front_AF_Right, OUTPUT);
  pinMode(echoPin_front_AF_Right, INPUT);
  pinMode(trigPin_front_AF_Left, OUTPUT);
  pinMode(echoPin_front_AF_Left, INPUT);

  pinMode(trigPin_front_AUF_Right, OUTPUT);
  pinMode(echoPin_front_AUF_Right, INPUT);
  pinMode(trigPin_front_AUF_Left, OUTPUT);
  pinMode(echoPin_front_AUF_Left, INPUT);

  pinMode(right_bs_R, OUTPUT);
  pinMode(right_bs_G, OUTPUT);
  digitalWrite(right_bs_R, LOW);
  digitalWrite(right_bs_G, LOW);
  pinMode(left_bs_R, OUTPUT);
  pinMode(left_bs_G, OUTPUT);
  digitalWrite(left_bs_R, LOW);
  digitalWrite(left_bs_G, LOW);

  pinMode(Buzzer_1, OUTPUT);
  pinMode(Buzzer_2, OUTPUT);

  myservo_1.attach(3);
  myservo_1.write(0);
  myservo_2.attach(9);
  myservo_2.write(180);

  lcd.begin(16, 2);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Enter PIN:");
}
// Password functions
void askForPassword() {
  if (showingMessage && millis() - messageMillis >= messageDuration) {
    resetScreen();
  }
  if (!showingMessage) {
    updatePotentiometer();
  }
  handleSelectButton();
  handleSubmitButton();
}

void handleSelectButton() {
  if (digitalRead(buttonSelectPin) == LOW && !selectButtonPressed && millis() - debounceMillis >= debounceDelay) {
    debounceMillis = millis();
    selectButtonPressed = true;
    addDigitsToPin(mappedValue);
  }
  if (digitalRead(buttonSelectPin) == HIGH) {
    selectButtonPressed = false;
  }
}

void addDigitsToPin(int value) {
  String valueStr = String(value);
  for (int i = 0; i < valueStr.length(); i++) {
    if (digitIndex < 5) {
      enteredPin += valueStr[i];
      digitIndex++;
    }
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("PIN: ");
  lcd.print(enteredPin);
  if (digitIndex == 5) {
    lcd.setCursor(0, 1);
    lcd.print("    Submit PIN");
  }
}

void handleDoublePressForLock() {
  int buttonState = digitalRead(buttonSubmitPin);

  if (buttonState == LOW && millis() - debounceMillis_lock >= debounceDelay_lock) {
    debounceMillis_lock = millis();

    if (!isButtonHeld) {
      isButtonHeld = true;

      if (!firstPressDetected) {
        firstPressDetected = true;
        firstPressMillis = millis();
      } else if (millis() - firstPressMillis <= doublePressInterval) {
        secondPressDetected = true;

        if (state == 1) {
          lockSystem();

        } else {
          lcd.clear();
          lcd.setCursor(0, 1);
          lcd.print("Put car to Park");
          messageMillis = millis();
          showingMessage = true;
          isDisplayingLockMessage = true;
        }
      }
    }
  }

  if (buttonState == HIGH) {
    isButtonHeld = false;

    if (firstPressDetected && millis() - firstPressMillis > doublePressInterval) {
      resetPressDetection();
    }
  }

  if (showingMessage && millis() - messageMillis >= messageDuration) {
    resetScreen();
    isDisplayingLockMessage = false;
  }
}

void handleSubmitButton() {
  if (digitalRead(buttonSubmitPin) == LOW && !submitButtonPressed && millis() - debounceMillis >= debounceDelay) {
    debounceMillis = millis();
    submitButtonPressed = true;

    if (digitIndex > 0) {
      if (digitIndex == 5) {
        if (enteredPin == password) {
          displayMessage("Access Granted");
          systemUnlocked = true;
        } else {
          displayMessage("Access Denied");
        }
      } else {
        displayMessage("");
      }
    }
    resetPinEntry();
  }
  if (digitalRead(buttonSubmitPin) == HIGH) {
    submitButtonPressed = false;
  }
}

void lockSystem() {
  systemUnlocked = false;
  displayMessage("Car Locked");
  resetPressDetection();
  myservo_1.write(0);
  myservo_2.write(180);
  digitalWrite(right_bs_R, LOW);
  digitalWrite(right_bs_G, LOW);
  digitalWrite(left_bs_R, LOW);
  digitalWrite(left_bs_G, LOW);
  digitalWrite(left_LED, LOW);
  digitalWrite(right_LED, LOW);
  rightLEDFlashing = false;
  leftLEDFlashing = false;
}

void resetPressDetection() {
  firstPressDetected = false;
  secondPressDetected = false;
  firstPressMillis = 0;
  secondPressMillis = 0;
}

void resetPinEntry() {
  enteredPin = "";
  digitIndex = 0;
}

void updatePotentiometer() {
  unsigned long currentMillis = millis();
  if (currentMillis - pot_PreviousMillis >= pot_Interval) {
    pot_PreviousMillis = currentMillis;
    sensorValue = analogRead(analogInPin);
    mappedValue = sensorValue / 100;

    mappedValue = min(mappedValue, 9);

    lcd.setCursor(0, 1);
    lcd.print(mappedValue);
  }
}

void displayMessage(const char* message) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(message);
  showingMessage = true;
  messageMillis = millis();
}

void resetScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Enter PIN:");
  showingMessage = false;
}

// Gearbox
void gearbox() {
  switch (state) {
    case 1:
      Park();
      break;

    case 2:
      Reverse();
      break;

    case 3:
      Drive();
      break;
  }
}

void Park() {
  myservo_1.write(90);
  myservo_2.write(90);
  isAutoFoldActive = false;
  holdRightClosed = false;
  holdLeftClosed = false;
  if (digitalRead(R_button) == HIGH) {
    state = 2;
  } else if (digitalRead(D_button) == HIGH) {
    state = 3;
  } else if (digitalRead(P_button) == HIGH) {
    state = 1;
  }
}

void Reverse() {
  myservo_1.write(90);
  myservo_2.write(90);
  isAutoFoldActive = false;
  holdRightClosed = false;
  holdLeftClosed = false;
  if (digitalRead(R_button) == HIGH) {
    state = 2;
  } else if (digitalRead(D_button) == HIGH) {
    state = 3;
  } else if (digitalRead(P_button) == HIGH) {
    state = 1;
  }
}

void Drive() {
  if (digitalRead(R_button) == HIGH) {
    state = 2;
  } else if (digitalRead(D_button) == HIGH) {
    state = 3;
  } else if (digitalRead(P_button) == HIGH) {
    state = 1;
  }
}

// Indicators and flashes
void indicators() {
  rightButtonState = digitalRead(right_button);
  leftButtonState = digitalRead(left_button);

  if (rightButtonState != lastRightButtonState && rightButtonState == HIGH) {
    if (rightLEDFlashing) {
      rightLEDFlashing = false;
      stop_buzz(Buzzer_1);
    } else {
      rightLEDFlashing = true;
      leftLEDFlashing = false;
      stop_buzz(Buzzer_2);
    }
  }

  if (leftButtonState != lastLeftButtonState && leftButtonState == HIGH) {
    if (leftLEDFlashing) {
      leftLEDFlashing = false;
      stop_buzz(Buzzer_2);
    } else {
      leftLEDFlashing = true;
      rightLEDFlashing = false;
      stop_buzz(Buzzer_1);
    }
  }

  lastRightButtonState = rightButtonState;
  lastLeftButtonState = leftButtonState;

  if (rightLEDFlashing) {
    flashLED(right_LED, 1.5);
    noTone(Buzzer_2);
  } else {
    digitalWrite(right_LED, LOW);
  }

  if (leftLEDFlashing) {
    flashLED(left_LED, 1.5);
    noTone(Buzzer_1);
  } else {
    digitalWrite(left_LED, LOW);
  }
}

void flashLED(int ledPin, float frequency) {
  unsigned long interval = (Second / frequency) / 2;
  if (millis() % (unsigned long)(interval * 2) < interval) {
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }
}

// Servo
void Mirrors() {
  if (!systemUnlocked && !isAutoFoldActive) {
    myservo_1.write(0);
    myservo_2.write(180);
  } else if (systemUnlocked && !isAutoFoldActive && !holdRightClosed && !holdLeftClosed) {
    myservo_1.write(90);
    myservo_2.write(90);
  }
}

// Variables for BSM, RCTA, and FCA
long measureDistance(int trigPin, int echoPin) {  // Function to measure distance for any ultrasonic sensor
  unsigned long startMicros = micros();

  digitalWrite(trigPin, LOW);
  while (micros() - startMicros < 2)
    ;

  digitalWrite(trigPin, HIGH);
  startMicros = micros();
  while (micros() - startMicros < 10)
    ;

  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, MAX_DURATION);

  return (duration == 0) ? -1 : duration;
}

float durationToCentimeters(long duration) {  // Function to convert duration to distance in centimeters
  if (duration == -1) return -1;

  return (duration * SOUND_SPEED) / 2.0;
}

// Buzzer for BSM and RCTA, and FCA
void shortBeep(int buzzerPin, float buzz_freq) {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis_beep >= beepInterval) {
    previousMillis_beep = currentMillis;

    if (digitalRead(buzzerPin) == LOW) {
      tone(buzzerPin, buzz_freq);
    } else {
      noTone(buzzerPin);
    }
  }
  if (currentMillis - previousMillis_beep >= beepDuration) {
    noTone(buzzerPin);
  }
}

void stop_buzz(int buzzerPin) {
  noTone(buzzerPin);
}

// BSM
void BSM() {
  if (state == 1 || state == 3) {
    BSM_check(trigPin_right_bsm, echoPin_right_bsm, right_bs_R, right_bs_G, Buzzer_1, rightLEDFlashing, state == 3);
    BSM_check(trigPin_left_bsm, echoPin_left_bsm, left_bs_R, left_bs_G, Buzzer_2, leftLEDFlashing, state == 3);
  }
}

void BSM_check(int trigPin, int echoPin, int redLED, int greenLED, int buzzer, bool ledFlashing, bool beep) {
  long duration = measureDistance(trigPin, echoPin);
  float cm = durationToCentimeters(duration);

  if (cm > 0 && cm < 20) {
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);

    if (ledFlashing) {
      digitalWrite(greenLED, LOW);
      flashLED(redLED, 4);

      if (beep) {
        shortBeep(buzzer, 4000);
      }
    }
  } else {
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
    noTone(buzzer);
  }
}

// RCTA
void RCTA() {
  if (state == 2) {
    RCTA_check(trigPin_right_bsm, echoPin_right_bsm);
    RCTA_check(trigPin_left_bsm, echoPin_left_bsm);
    RCTA_check(trigPin_back, echoPin_back);
  }
}

void RCTA_check(int trigPin, int echoPin) {
  long duration = measureDistance(trigPin, echoPin);
  float cm = durationToCentimeters(duration);

  if (cm > 0 && cm < 20) {
    if (state == 2) {
      flashLED(right_bs_R, 4);
      flashLED(left_bs_R, 4);
      shortBeep(Buzzer_1, 4000);
    }
  } else {
    digitalWrite(right_bs_R, LOW);
    digitalWrite(right_bs_G, LOW);
    digitalWrite(left_bs_R, LOW);
    digitalWrite(left_bs_G, LOW);
    noTone(Buzzer_1);
  }
}

// FCA
void FCA() {
  if (!isDisplayingLockMessage) {
    long duration_front_FCA = measureDistance(trigPin_front_FCA, echoPin_front_FCA);
    float cm = durationToCentimeters(duration_front_FCA);

    if (cm > 0 && cm <= 20 && state == 3) {
      if (cm <= 20) {
        shortBeep(Buzzer_2, 4000);
      } else {
        noTone(Buzzer_2);
      }
      lcd.setCursor(0, 1);
      lcd.print(cm);
      lcd.print("cm ^");

      if (cm <= 5) {
        lcd.print(" STOP!!!");
        tone(Buzzer_2, 4000);
      }
    } else {
      lcd.setCursor(0, 1);
      lcd.print("                ");
      noTone(Buzzer_2);
    }
  }
}

//Autofold Mirrors
void Autofold_unfold() {
  if (state == 3) {
    if (checkDistance(trigPin_front_AF_Right, echoPin_front_AF_Right, 3)) {
      myservo_1.write(0);
      holdRightClosed = true;
    }
    if (checkDistance(trigPin_front_AUF_Right, echoPin_front_AUF_Right, 3)) {
      myservo_1.write(90);
      holdRightClosed = false;
    }
    if (checkDistance(trigPin_front_AF_Left, echoPin_front_AF_Left, 3)) {
      myservo_2.write(180);
      holdLeftClosed = true;
    }
    if (checkDistance(trigPin_front_AUF_Left, echoPin_front_AUF_Left, 3)) {
      myservo_2.write(90);
      holdLeftClosed = false;
    }

    isAutoFoldActive = holdRightClosed || holdLeftClosed;
  } else {
    isAutoFoldActive = false;
  }
}

bool checkDistance(int trigPin, int echoPin, int threshold) {
  long duration = measureDistance(trigPin, echoPin);
  float cm = durationToCentimeters(duration);
  return (cm > 0 && cm <= threshold);
}

void LCD() {  // Display when system car runs
  if (state == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Gear : Park   ");
  } else if (state == 2) {
    lcd.setCursor(0, 0);
    lcd.print("Gear : Reverse");
  } else if (state == 3) {
    lcd.setCursor(0, 0);
    lcd.print("Gear : Drive  ");
  }
}

void car() {
  gearbox();
  indicators();
  Mirrors();
  LCD();
  BSM();
  RCTA();
  FCA();
  Autofold_unfold();
}

void loop() {
  if (!systemUnlocked) {
    askForPassword();
  } else {
    car();
    handleDoublePressForLock();
  }
}
