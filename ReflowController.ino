/*******************************************************************************
  Title: Tiny Reflow Controller
  Version: 2.00
  Date: 03-03-2019
  Company: Rocket Scream Electronics
  Author: Lim Phang Moh
  Website: www.rocketscream.com

  https://github.com/rocketscream/TinyReflowController

  Mod. 2.0.1 by Radioelf for use MAX31865

  Brief
  =====
  This is an example firmware for our Arduino compatible Tiny Reflow Controller.
  A big portion of the code is copied over from our Reflow Oven Controller
  Shield. We added both lead-free and leaded reflow profile support in this
  firmware which can be selected by pressing switch #2 (labelled as LF|PB on PCB)
  during system idle. The unit will remember the last selected reflow profile.
  You'll need to use the MAX31856 library for Arduino.

  Lead-Free Reflow Curve
  ======================

  Temperature (Degree Celcius)                 Magic Happens Here!
  245-|                                               x  x
      |                                            x        x
      |                                         x              x
      |                                      x                    x
  200-|                                   x                          x
      |                              x    |                          |   x
      |                         x         |                          |       x
      |                    x              |                          |
  150-|               x                   |                          |
      |             x |                   |                          |
      |           x   |                   |                          |
      |         x     |                   |                          |
      |       x       |                   |                          |
      |     x         |                   |                          |
      |   x           |                   |                          |
  30 -| x             |                   |                          |
      |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
      | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ |_ _ _ _ _
                                                                 Time (Seconds)

  Leaded Reflow Curve (Kester EP256)
  ==================================

  Temperature (Degree Celcius)         Magic Happens Here!
  219-|                                       x  x
      |                                    x        x
      |                                 x              x
  180-|                              x                    x
      |                         x    |                    |   x
      |                    x         |                    |       x
  150-|               x              |                    |           x
      |             x |              |                    |
      |           x   |              |                    |
      |         x     |              |                    |
      |       x       |              |                    |
      |     x         |              |                    |
      |   x           |              |                    |
  30 -| x             |              |                    |
      |<  60 - 90 s  >|<  60 - 90 s >|<   60 - 90 s      >|
      | Preheat Stage | Soaking Stage|   Reflow Stage     | Cool
   0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ |_ _ _ _ _ _ _ _ _ _ _
                                                                 Time (Seconds)

  This firmware owed very much on the works of other talented individuals as
  follows:
  ==========================================
  Brett Beauregard (www.brettbeauregard.com)
  ==========================================
  Author of Arduino PID library. On top of providing industry standard PID
  implementation, he gave a lot of help in making this reflow oven controller
  possible using his awesome library.

  ==========================================
  Limor Fried of Adafruit (www.adafruit.com)
  ==========================================
  Author of Arduino MAX31856 and SSD1306 libraries. Adafruit has been the source 
  of tonnes of tutorials, examples, and libraries for everyone to learn.

  ==========================================
  Spence Konde (www.drazzy.com/e/)
  ==========================================
  Maintainer of the ATtiny core for Arduino:
  https://github.com/SpenceKonde/ATTinyCore

  Disclaimer
  ==========
  Dealing with high voltage is a very dangerous act! Please make sure you know
  what you are dealing with and have proper knowledge before hand. Your use of
  any information or materials on this Tiny Reflow Controller is entirely at
  your own risk, for which we shall not be liable.

  Licences
  ========
  This Tiny Reflow Controller hardware and firmware are released under the
  Creative Commons Share Alike v3.0 license
  http://creativecommons.org/licenses/by-sa/3.0/
  You are free to take this piece of code, use it and modify it.
  All we ask is attribution including the supporting libraries used in this
  firmware.

  Required Libraries
  ==================
  - Arduino PID Library:
    >> https://github.com/br3ttb/Arduino-PID-Library
  - Adafruit MAX31856 Library:
    >> https://github.com/adafruit/Adafruit_MAX31856
  - Adafruit MAX31865 Library:
    >> https://github.com/adafruit/Adafruit_MAX31865
  - Adafruit SSD1306 Library:
    >> https://github.com/adafruit/Adafruit_SSD1306
  - Adafruit GFX Library:
    >> https://github.com/adafruit/Adafruit-GFX-Library

  Revision  Description
  ========  ===========
  2.00      Support V2 of the Tiny Reflow Controller:
            - Based on ATMega328P 3.3V @ 8MHz
            - Uses SSD1306 128x64 OLED
  2.0.1    Mod. for use MAX31865 and minor changes.. (Radioelf)

*******************************************************************************/

// ***** INCLUDES *****
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MAX31865.h>
#include <PID_v1.h>

//#define serial

// ***** TYPE DEFINITIONS *****
// Switch press status
typedef enum SWITCH {
  SWITCH_NONE,
  SWITCH_1,
  SWITCH_2
} switch_t;

// Reflow oven controller state machine state constants and variable
#define REFLOW_STATE_IDLE 0
#define REFLOW_STATE_PREHEAT 1
#define REFLOW_STATE_SOAK 2
#define REFLOW_STATE_REFLOW 3
#define REFLOW_STATE_COOL 4
#define REFLOW_STATE_COMPLETE 5
#define REFLOW_STATE_TOO_HOT 6
#define REFLOW_STATE_ERROR 7
unsigned char reflowState;
// Reflow oven controller status constants and variable
#define REFLOW_STATUS_OFF 0
#define REFLOW_STATUS_ON 1
bool reflowStatus;
// Switch debounce state machine state constants and variable
#define DEBOUNCE_STATE_IDLE 0
#define DEBOUNCE_STATE_CHECK 1
#define DEBOUNCE_STATE_RELEASE 2
unsigned char debounceState;
// Reflow profile type constants and variable
#define REFLOW_PROFILE_LEADFREE 0
#define REFLOW_PROFILE_LEADED 1
bool reflowProfile;

// ***** CONSTANTS *****
// ***** GENERAL *****
// ***** GENERAL PROFILE CONSTANTS *****
#define PROFILE_TYPE_ADDRESS 0
#define TEMPERATURE_ROOM 50
#define TEMPERATURE_SOAK_MIN 150
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5

// ***** LEAD FREE PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_LF 200
#define TEMPERATURE_REFLOW_MAX_LF 250
#define SOAK_MICRO_PERIOD_LF 9000

// ***** LEADED PROFILE CONSTANTS *****
#define TEMPERATURE_SOAK_MAX_PB 180
#define TEMPERATURE_REFLOW_MAX_PB 224
#define SOAK_MICRO_PERIOD_PB 10000

// **** DESOLDERING
#define TEMPERATURE_DESOLDERING 260

// ***** SWITCH SPECIFIC CONSTANTS *****
#define DEBOUNCE_PERIOD_MIN 100

// ***** DISPLAY SPECIFIC CONSTANTS *****
#define UPDATE_RATE 100

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define X_AXIS_START 18   // X-axis starting position
#define OPERATING_WIDTH (SCREEN_WIDTH - X_AXIS_START)
#define OLED_RESET -1  // NO pin Reset pin
#define SCREEN_ADDRESS 0x3C

// ***** PIN ASSIGNMENT *****
#define ssrPin A0  // SSR (Zero Voltage Turn-On)
#define fanPin  A1
#define thermoCSPin 10
#define ledPin 4
#define buzzerPin 5
#define switchStartStopPin 3
#define switchLfPbPin 2

// ***** PID CONTROL VARIABLES *****
#define windowSize 2000
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;

unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long updateLcd;
unsigned long timerSoak;
unsigned long buzzerPeriod;
unsigned char soakTemperatureMax;
unsigned char reflowTemperatureMax;
unsigned long soakMicroPeriod;

// Switch debounce timer
long lastDebounceTime;
// Switch press status
switch_t switchStatus;
switch_t switchValue;
switch_t switchMask;
// Seconds timer
unsigned int timerSeconds;
// thermo fault status
unsigned char fault;

unsigned int timerUpdate;
unsigned char temperature[OPERATING_WIDTH];
unsigned char x;
unsigned char countError = 0;
bool desoldering = false;

// The value of the Rref resistor. Use 430.0 for PT100 and 4300.0 for PT1000
#define RREF 430.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL 100.0

// PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

//Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // Pin A4->SDA and A5->SCL
// Use software SPI: CS, DI, DO, CLK
// Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
Adafruit_MAX31865 thermo = Adafruit_MAX31865(thermoCSPin);

// Reset software
void (*resetFunc)(void) = 0;

void setup() {
  // Check current selected reflow profile
  unsigned char value = EEPROM.read(PROFILE_TYPE_ADDRESS);
  if ((value == 0) || (value == 1)) {
    // Valid reflow profile value
    reflowProfile = value ? REFLOW_PROFILE_LEADED : REFLOW_PROFILE_LEADFREE;
  } else {
    // Default to lead-free profile
    EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
    reflowProfile = REFLOW_PROFILE_LEADFREE;
  }
  // SSR pin initialization to ensure reflow oven is off (SSR Zero Voltage Turn-On)
  digitalWrite(ssrPin, LOW);
  pinMode(ssrPin, OUTPUT);

  // Fan pin initialization
  digitalWrite(fanPin, LOW);
  pinMode(fanPin, OUTPUT);

  // Buzzer pin initialization to ensure annoying buzzer is off
  digitalWrite(buzzerPin, LOW);
  pinMode(buzzerPin, OUTPUT);

  // LED pins initialization and turn on upon start-up (active high)
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

#ifdef serial
  // Serial communication at 115200 bps
  Serial.begin(115200);
#endif

  // Initialize thermo interface
  thermo.begin(MAX31865_4WIRE);  // PT-100, 4 wires

  // Start-up splash
  digitalWrite(buzzerPin, HIGH);

  // Pullup switchs
  pinMode(switchStartStopPin, INPUT_PULLUP);
  pinMode(switchLfPbPin, INPUT_PULLUP);
  // SSD1306_SWITCHCAPVCC = internally generate display voltage from 3.3V, I2C address 0x3C for 128x64
  if (oled.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    oled.display();
  } else {
#ifdef serial
    Serial.println(F("Oled Error"));
#endif
    delay(5000);
    resetFunc();
  }
  delay(250);
  digitalWrite(buzzerPin, LOW);
  delay(500);

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(WHITE);
  oled.setCursor(0, 0);
  oled.println(F("Mod. Reflow"));
  oled.println(F("    Control"));
  oled.println();
  oled.println(F("    v2.01"));
  oled.println();
  oled.println(F("  17-09-23"));
  oled.display();
  delay(3000);
  oled.clearDisplay();

  // Turn off LED (active high)
  digitalWrite(ledPin, LOW);
  // Initialize time keeping variable
  nextCheck = millis();
  // Initialize thermo reading variable
  nextRead = millis();
  // Initialize LCD update timer
  updateLcd = millis();
  if (thermo.readFault()) {
    oled.setTextSize(1);
    oled.setTextColor(WHITE);
    oled.setCursor(0, 0);
    oled.println(F("PT100 Error"));
#ifdef serial
    Serial.println(F("PT100 Error"));
#endif
    digitalWrite(ssrPin, LOW);
    do {
      digitalWrite(ledPin, !(digitalRead(ledPin)));
      delay(150);
    } while (thermo.readFault());
    oled.clearDisplay();
  }
}

void loop() {

  // Current time
  unsigned long now;

  // Time to read thermo?
  if (millis() > nextRead) {
    // Read thermo next sampling period
    nextRead += SENSOR_SAMPLING_TIME;
    // Read current temperature
    input = thermo.temperature(RNOMINAL, RREF);
    if (fault != 10) {
      // Check for thermo fault
      fault = thermo.readFault();
    }

    if (fault || input > 325) {
      // Illegal operation
      digitalWrite(ssrPin, LOW);
      digitalWrite(buzzerPin, HIGH);
      reflowState = REFLOW_STATE_ERROR;
      reflowStatus = REFLOW_STATUS_OFF;
      switchStatus = SWITCH_NONE;
#ifdef serial
      Serial.println(F("Fallo"));
#endif
      delay(500);
      digitalWrite(buzzerPin, LOW);
    }
  }

  if (millis() > nextCheck) {
    // Check input in the next seconds
    nextCheck += SENSOR_SAMPLING_TIME;
    // If reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON) {
      // Toggle LED as system heart beat
      digitalWrite(ledPin, !(digitalRead(ledPin)));
      // Increase seconds timer for reflow curve plot/led
      timerSeconds++;
#ifdef serial
      // Send temperature and time stamp to serial
      Serial.print(timerSeconds);
      Serial.print(F(";"));
      Serial.print(setpoint);
      Serial.print(F(";"));
      Serial.print(input);
      Serial.print(F(";"));
      Serial.println(output);
#endif
    } else {
      // Turn off LED
      digitalWrite(ledPin, LOW);
    }
  }

  if (millis() > updateLcd) {
    // Update LCD in the next 100 ms
    updateLcd += UPDATE_RATE;

    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setCursor(0, 0);
    switch (reflowState) {
      case REFLOW_STATE_IDLE:
        oled.print(F("Espera"));
        break;
      case REFLOW_STATE_PREHEAT:
        oled.setTextSize(1);
        oled.print(F("Calentando"));
        break;
      case REFLOW_STATE_SOAK:
        oled.print(F("Fluido"));
        break;
      case REFLOW_STATE_REFLOW:
        if (desoldering) {
          oled.setTextSize(1);
          oled.print(F("Desoldar"));
        } else
          oled.print(F("Fundir"));
        break;
      case REFLOW_STATE_COOL:
        oled.print(F("Enfriar"));
        break;
      case REFLOW_STATE_COMPLETE:
        oled.print(F("Fin"));
        break;
      case REFLOW_STATE_TOO_HOT:
        oled.setTextSize(1);
        oled.print(F("Caliente"));
        break;
      case REFLOW_STATE_ERROR:
        oled.print(F("!ERROR!"));
        break;
    }
    oled.setTextSize(1);
    if (reflowProfile == REFLOW_PROFILE_LEADFREE) {
      oled.setCursor(80, 0);
      oled.print(F("NO Plomo"));
    } else {
      oled.setCursor(95, 0);
      oled.print(F("Plomo"));
    }

    // Temperature marker
    oled.setCursor(0, 18);
    oled.print(F("250"));
    oled.setCursor(0, 36);
    oled.print(F("150"));
    oled.setCursor(0, 54);
    oled.print(F("50"));
    // Draw temperature and time axis
    oled.drawLine(18, 18, 18, 63, WHITE);
    oled.drawLine(18, 63, 127, 63, WHITE);
    oled.setCursor(115, 0);

    // If currently in error state
    if (reflowState == REFLOW_STATE_ERROR) {
      oled.clearDisplay();
      oled.setTextSize(3);
      oled.setCursor(0, 20);
      oled.print(F("Error"));
      oled.print(fault);
#ifdef serial
      Serial.println(F("Error"));
#endif
      digitalWrite(ssrPin, LOW);
      oled.display();
      while (thermo.readFault()) {
        digitalWrite(ledPin, !digitalRead(ledPin));
        delay(200);
        if (++countError == 200)
          resetFunc();
      }
    } else {
      countError = 0;
      // Right align temperature reading
      if (input < 10)
        oled.setCursor(91, 9);
      else if (input < 100)
        oled.setCursor(85, 9);
      else
        oled.setCursor(80, 9);
      // Display current temperature
      oled.print(input);
      oled.print((char)247);
      oled.print(F("C"));
      if (reflowStatus == REFLOW_STATUS_ON) {
        oled.setCursor(90, 55);
        oled.print(int(setpoint));
        oled.print((char)247);
        oled.print(F("C"));
      }
    }

    if (reflowStatus == REFLOW_STATUS_ON) {
      // We are updating the display faster than sensor reading
      if (timerSeconds > timerUpdate) {
        // Store temperature reading every 3 s
        if ((timerSeconds % 3) == 0) {
          timerUpdate = timerSeconds;
          unsigned char averageReading = map(input, 0, TEMPERATURE_REFLOW_MAX_LF, 63, 19);
          if (x < (OPERATING_WIDTH)) {
            temperature[x++] = averageReading;
          } else if (desoldering) {
            x = 1;
            temperature[x] = averageReading;
          }
        }
      }
    }
    for (unsigned char timeAxis = 0; timeAxis < x; timeAxis++) {
      oled.drawPixel(timeAxis + X_AXIS_START, temperature[timeAxis], WHITE);
    }

    // Update screen
    oled.display();
  }

  // Reflow oven controller state machine
  switch (reflowState) {
    case REFLOW_STATE_IDLE:
      // If oven temperature is still above room temperature
      if (input >= TEMPERATURE_ROOM && desoldering == false) {
        reflowState = REFLOW_STATE_TOO_HOT;
      } else {
        // If switch is pressed to start reflow process
        if (switchStatus == SWITCH_1) {
#ifdef serial
          // Send header for CSV file
          Serial.println(F("Time,Setpoint,Input,Output"));
#endif
          // Intialize seconds timer for serial debug information
          timerSeconds = 0;

          // Initialize reflow plot update timer
          timerUpdate = 0;

          for (x = 0; x < OPERATING_WIDTH; x++) {
            temperature[x] = 0;
          }
          // Initialize index for average temperature array used for reflow plot
          x = 0;

          // Initialize PID control window starting time
          windowStartTime = millis();
          // Ramp up to minimum soaking temperature
          setpoint = TEMPERATURE_SOAK_MIN;
          // Load profile specific constant
          if (reflowProfile == REFLOW_PROFILE_LEADFREE) {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_LF;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
            soakMicroPeriod = SOAK_MICRO_PERIOD_LF;
          } else {
            soakTemperatureMax = TEMPERATURE_SOAK_MAX_PB;
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_PB;
            soakMicroPeriod = SOAK_MICRO_PERIOD_PB;
          }
          // Tell the PID to range between 0 and the full window size
          reflowOvenPID.SetOutputLimits(0, windowSize);
          reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
          // Turn the PID on
          reflowOvenPID.SetMode(AUTOMATIC);
          if (desoldering) {
            setpoint = (TEMPERATURE_DESOLDERING);
            reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
            reflowTemperatureMax = TEMPERATURE_REFLOW_MAX_LF;
            reflowState = REFLOW_STATE_REFLOW;
            reflowStatus = REFLOW_STATUS_ON;
          } else
            // Proceed to preheat stage
            reflowState = REFLOW_STATE_PREHEAT;
        }
      }
      break;

    case REFLOW_STATE_PREHEAT:
      reflowStatus = REFLOW_STATUS_ON;
      // If minimum soak temperature is achieve
      if (input >= TEMPERATURE_SOAK_MIN) {
        // Chop soaking period into smaller sub-period
        timerSoak = millis() + soakMicroPeriod;
        // Set less agressive PID parameters for soaking ramp
        reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
        // Ramp up to first section of soaking temperature
        setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
        // Proceed to soaking state
        reflowState = REFLOW_STATE_SOAK;
      }
      break;

    case REFLOW_STATE_SOAK:
      // If micro soak temperature is achieved
      if (millis() > timerSoak) {
        timerSoak = millis() + soakMicroPeriod;
        // Increment micro setpoint
        setpoint += SOAK_TEMPERATURE_STEP;
        if (setpoint > soakTemperatureMax) {
          // Set agressive PID parameters for reflow ramp
          reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
          // Ramp up to first section of soaking temperature
          setpoint = reflowTemperatureMax;
          // Proceed to reflowing state
          reflowState = REFLOW_STATE_REFLOW;
        }
      }
      break;

    case REFLOW_STATE_REFLOW:
      // We need to avoid hovering at peak temperature for too long
      // Crude method that works like a charm and safe for the components
      if (input >= (reflowTemperatureMax - 5) && desoldering == false) {
        // Set PID parameters for cooling ramp
        reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
        // Ramp down to minimum cooling temperature
        setpoint = TEMPERATURE_COOL_MIN;
        // Proceed to cooling state
        reflowState = REFLOW_STATE_COOL;
      }
      break;

    case REFLOW_STATE_COOL:
      // If minimum cool temperature is achieve
      if (input <= TEMPERATURE_COOL_MIN) {
        // Retrieve current time for buzzer usage
        buzzerPeriod = millis() + 1000;
        // Turn on buzzer to indicate completion
        digitalWrite(buzzerPin, HIGH);
        // Turn off reflow process
        reflowStatus = REFLOW_STATUS_OFF;
        // Proceed to reflow Completion state
        reflowState = REFLOW_STATE_COMPLETE;
        // Turn off fan
        digitalWrite(fanPin, LOW);
      }
      else {
        // Turn on fan
        digitalWrite(fanPin, HIGH);
      }
      break;

    case REFLOW_STATE_COMPLETE:
      if (millis() > buzzerPeriod) {
        // Turn off buzzer
        digitalWrite(buzzerPin, LOW);
        // Reflow process ended
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_TOO_HOT:
      // If oven temperature drops below room temperature
      if (input < TEMPERATURE_ROOM) {
        // Ready to reflow
        reflowState = REFLOW_STATE_IDLE;
      }
      break;

    case REFLOW_STATE_ERROR:
      // Check for thermo fault
      // 1: Ground short detected.
      // 2: VCC short detected.
      // 4: RTD sensor failure. Resistance out of range
      // 8: Open circuit detected. No RTD sensor connected.
      // 16: ADC conversion failure. RTD sensor cannot be read.
      fault = thermo.readFault();

      // If thermo problem is still present
      if (fault) {
        // Wait until thermo wire is connected
        reflowState = REFLOW_STATE_ERROR;
      } else {
        // Clear to perform reflow process
        reflowState = REFLOW_STATE_IDLE;
      }
      break;
  }

  // If switch 1 is pressed
  if (switchStatus == SWITCH_1) {
    // If currently reflow process is on going
    if (reflowStatus == REFLOW_STATUS_ON && desoldering == false) {
      // Button press is for cancelling
      // Turn off reflow process
      reflowStatus = REFLOW_STATUS_OFF;
      // Reinitialize state machine
      reflowState = REFLOW_STATE_IDLE;
    }
  }
  // Switch 2 is pressed
  else if (switchStatus == SWITCH_2) {
    // Only can switch reflow profile during idle
    if (reflowState == REFLOW_STATE_IDLE) {
      // Currently using lead-free reflow profile
      if (reflowProfile == REFLOW_PROFILE_LEADFREE) {
        // Switch to leaded reflow profile
        reflowProfile = REFLOW_PROFILE_LEADED;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 1);
      }
      // Currently using leaded reflow profile
      else {
        // Switch to lead-free profile
        reflowProfile = REFLOW_PROFILE_LEADFREE;
        EEPROM.write(PROFILE_TYPE_ADDRESS, 0);
      }
    }
  }
  // Switch status has been read
  switchStatus = SWITCH_NONE;

  // Simple switch debounce state machine (analog switch)
  switch (debounceState) {
    case DEBOUNCE_STATE_IDLE:
      // No valid switch press
      switchStatus = SWITCH_NONE;

      switchValue = readSwitch();

      // If either switch is pressed
      if (switchValue != SWITCH_NONE) {
        // Keep track of the pressed switch
        switchMask = switchValue;
        // Intialize debounce counter
        lastDebounceTime = millis();
        // Proceed to check validity of button press
        debounceState = DEBOUNCE_STATE_CHECK;
      }
      break;

    case DEBOUNCE_STATE_CHECK:
      switchValue = readSwitch();
      if (switchValue == switchMask) {
        // If minimum debounce period is completed
        if ((millis() - lastDebounceTime) > DEBOUNCE_PERIOD_MIN) {
          // Valid switch press
          switchStatus = switchMask;
          // Proceed to wait for button release
          debounceState = DEBOUNCE_STATE_RELEASE;
        }
      }
      // False trigger
      else {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;

    case DEBOUNCE_STATE_RELEASE:
      switchValue = readSwitch();
      if (switchValue == SWITCH_NONE) {
        // Reinitialize button debounce state machine
        debounceState = DEBOUNCE_STATE_IDLE;
      }
      break;
  }

  // PID computation and SSR control
  if (reflowStatus == REFLOW_STATUS_ON) {
    now = millis();

    reflowOvenPID.Compute();

    if (int(now - windowStartTime) > windowSize) {
      // Time to shift the Relay Window
      windowStartTime += windowSize;
    }
    if (output > (now - windowStartTime))
      digitalWrite(ssrPin, HIGH);
    else
      digitalWrite(ssrPin, LOW);
  }
  // Reflow oven process is off, ensure oven is off
  else {
    digitalWrite(ssrPin, LOW);
  }
}

// read readSwitch
switch_t readSwitch(void) {
  if (digitalRead(switchStartStopPin) == LOW && digitalRead(switchLfPbPin) == LOW) {
    if (desoldering == true) {
      delay(500);
      if (digitalRead(switchStartStopPin) == LOW && digitalRead(switchLfPbPin) == LOW)
        digitalWrite(buzzerPin, HIGH);
      while (digitalRead(switchStartStopPin) == LOW && digitalRead(switchLfPbPin) == LOW) {
        delay(100);
        digitalWrite(ssrPin, LOW);
        digitalWrite(buzzerPin, LOW);
        desoldering = false;
        setpoint = 0;
        reflowState = REFLOW_STATE_TOO_HOT;
        reflowStatus = REFLOW_STATUS_OFF;
      }
      return SWITCH_NONE;
    }
    unsigned char count = 0;
    do {
      delay(10);
      if (++count == 250) {
        desoldering = true;
        digitalWrite(buzzerPin, LOW);
        switchStatus = SWITCH_1;
        return SWITCH_NONE;
      }
      if (count % 25 == 0)
        digitalWrite(buzzerPin, !(digitalRead(buzzerPin)));
    } while (digitalRead(switchStartStopPin) == LOW && digitalRead(switchLfPbPin) == LOW);
    digitalWrite(buzzerPin, LOW);
    desoldering = false;
    return SWITCH_NONE;
  }
  // Switch connected directly to individual separate pins
  if (digitalRead(switchStartStopPin) == LOW)
    return SWITCH_1;

  if (digitalRead(switchLfPbPin) == LOW)
    return SWITCH_2;

  return SWITCH_NONE;
}
