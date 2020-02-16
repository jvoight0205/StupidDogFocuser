#include <DHT.h>
#include <Encoder.h>
#include <AccelStepper.h>

/*
    This is the firmware for an Arduino UNO based electronic focuser.
    Stupid Dog Observator
    Written and copyright by Jeff Voight, Feb 2020,

    TODO: Put a normal license here.

    Meanwhile:
    Consider this to be a terribly restrictive license.
    This software contains no warranty of any kind and makes no claims as to it's performance. If it
    erases your favorite photos and murders your friends, it's all on you.
*/

#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN 4
#define ENC_A 5
#define ENC_B 6
#define TURBO_PIN 7
#define HALF_STEP_PIN 8
#define DHT_PIN 9
#define VERSION 1.0
#define INITIALIZATION_CHAR ':'
#define TERMINATOR_CHAR '#'
#define NEWLINE_CHAR '\n'
#define TURBO_MULTIPLIER 200

#define DHTTYPE DHT11
//#define DHTTYPE DHT22

int16_t encoderPosition = 0; // Set to be incorrect on purpose so that it gets updated in setup.
uint16_t motorPosition = 0x7FFF; // midpoint of 0xFFFF
uint16_t newMotorPosition = motorPosition; // for two-stage moves

uint16_t maxSpeed = 1000;
uint8_t accelRate = 150;
bool isHalfStep = false;

int8_t temperatureCoefficient = 0;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN) ; // Skipping enable pin on this
Encoder encoder(ENC_A, ENC_B);
DHT dht(DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(9600);
  while (!Serial); // Wait for Serial initialization. Should be very little time.
  respond(formatVersion(VERSION));
  initializeDHT();
  initializeStepper();
  initializeEncoder();
}

/*
   Main loop. We need to call stepper.run() as frequently as possible so the motor doesn't stutter.
   As such, we call stepper.run() between each functional block of code. We don't call it at the very
   end because the loop is just about to be called again and stepper.run() is the first line.

   In this loop, we determine if any serial commands have come in. If so, interpret them. Then we determine
   if any changes have been made to the focus knob. If so, handle them. And, lastly, update the motor
   position variable so that it remains up-to-date at all times.
*/
void loop() {
  stepper.run();
  if (Serial.available() > 0) {
    interpretSerial(Serial.readStringUntil(NEWLINE_CHAR));
  }
  stepper.run();
  interpretEncoder();
  stepper.run();
  motorPosition = stepper.currentPosition();
}

/*
   Respond moonlite style (e.g. with a # and a carriage return at the end)
*/
void respond(String response) {
  Serial.print(response); Serial.println("#");
}

/**
   Determine if a valid command has been received and act on it.
   This parser is built as a bit of a decision tree for performance reasons.
*/
void interpretSerial(String command) {
  // Bail out early if the command isn't terminated.
  if (command.charAt(0) != INITIALIZATION_CHAR || command.charAt(command.length() - 1) != TERMINATOR_CHAR) {
    Serial.print("ERROR: "); Serial.print(command); Serial.println(" should start with : and end with #");
  } else {
    // Parse the command out from between the chars
    String commandString = command.substring(1, command.length() - 1);

    switch (commandString.charAt(0)) {
      case 'C': // Temperature Conversion. This probably isn't necessary
        dht.read();
        break;

      case 'F': // Feed Commands
        switch (commandString.charAt(1)) {
          case 'G': // Go-to (move) to new set position
            stepper.moveTo(newMotorPosition);
            break;

          case 'Q': // HALT movement
            stepper.stop();
            while (stepper.isRunning()) { // Call run() as fast as possible to allow motor to halt as quickly as possible
              for (int i = 0; i < 20; i++) { // Can go even faster to halt if we take a bunch of steps between isRunning callse
                stepper.run();
              }
            }
            motorPosition = stepper.currentPosition();
            newMotorPosition = motorPosition; // Make sure we don't take off again.
            break;

          default: // Not implemented
            respond("??");
        }
        break;

      case 'G': // Getter Commands
        switch (commandString.charAt(1)) {
          case 'C': // Temperature coefficient
            respond(format2UHex(temperatureCoefficient));
            break;

          case 'D': // Stepping delay
            respond(format2UHex(32));
            break;

          case 'H': // Half-step
            respond(format2UHex(isHalfStep ? 255 : 0));
            break;

          case 'I': // Moving
            respond(format2UHex(stepper.isRunning() ? 1 : 0));
            break;

          case 'N': // New Position
            respond(formatUHex(newMotorPosition));
            break;

          case 'P': // Position
            respond(formatUHex(stepper.currentPosition()));
            break;

          case 'T': // Temperature
            respond(formatFHex(dht.readTemperature()));
            break;

          case 'V': // Version
            respond(formatVersion(VERSION));
            break;

          default: // Not implemented
            respond("??");

        }
        break;

      case 'S': // Setter commands
        switch (commandString.charAt(1)) {
          case 'C': // Temperature coefficient
            temperatureCoefficient = parse2UHex(commandString.substring(2));
            break;

          case 'D': // Stepping delay
            // Ignored
            break;

          case 'F': // Full-step mode
            isHalfStep = false;
            digitalWrite(HALF_STEP_PIN, HIGH);
            break;

          case 'H': // Half-step mode
            isHalfStep = true;
            digitalWrite(HALF_STEP_PIN, HIGH);
            break;

          case 'N': // New position
            newMotorPosition = parseUHex(commandString.substring(2));
            break;

          case 'P': // Current position
            newMotorPosition = parseUHex(commandString.substring(2));
            stepper.setCurrentPosition(newMotorPosition);
            motorPosition = newMotorPosition; // Make sure everything points to the same number
            break;

          default: // Not implemented
            respond("??");
        }
        break;

      case '+': // Activate Temperature Compensation Focusing
        break; // Not implemented

      case '-': // Deactivate Temperature Compensation Focusing
        break; // Not implemented

      case 'P':
        break;

      default: // Not implemented
        respond("??");
    }
  }
}

uint8_t parse2UHex(String theHexString) {
  uint8_t i;
  int strLen = theHexString.length() + 1;
  char charBuffer[strLen];
  theHexString.toCharArray(charBuffer, strLen);
  sscanf(charBuffer, "%x", &i);
  return i;
}

uint16_t parseUHex(String theHexString) {
  uint16_t i;
  int strLen = theHexString.length() + 1;
  char charBuffer[strLen];
  theHexString.toCharArray(charBuffer, strLen);
  sscanf(charBuffer, "%x", &i);
  return i;
}

/*
   Format version number for moonlite
*/
String formatVersion(float theNumber) {
  char buffer[3];
  sprintf(buffer, "%1u%1u", int(theNumber), int((theNumber - int(theNumber)) * 10));
  return String(buffer);
}

/*
   Convert an unsigned 8 bit int to a hexidecimal string representation
*/
String format2UHex(uint8_t theNumber) {
  char buffer[3];
  sprintf(buffer, "%02X", theNumber);
  return String(buffer);
}

/*
   Convert an unsigned 16 bit int to a hexidecimal string representation
*/
String formatUHex(uint16_t theNumber) {
  char buffer[5];
  sprintf(buffer, "%04X", theNumber);
  return String(buffer);
}

/*
   Convert a float to a hexidecimal representation.
*/
String formatFHex(float theNumber) {
  char buffer[5];
  sprintf(buffer, "%04X", theNumber);
  return String(buffer);
}

/*
   Determine if the encoder has moved this cycle. If so, make the adjustments
   to the stepper and update variables.
*/
void interpretEncoder() {
  long newEncoderPosition = encoder.read();
  if (newEncoderPosition != encoderPosition) {                  // If there's any change from the previous read,
    long encoderChange = newEncoderPosition - encoderPosition;  // determine the difference,
    newMotorPosition += encoderChange * getTurboMultiplier();   // calculate new stepper target
    stepper.moveTo(newMotorPosition);                           // set the new target
  }
  encoderPosition = newEncoderPosition;                       // reset the encoder status
}

/*
   Read the turbo button and apply a multiplier if necessary
*/
uint16_t getTurboMultiplier() {
  return digitalRead(TURBO_PIN) == HIGH ? TURBO_MULTIPLIER : 1;
}

/*
   Initialize the stepper motor.
*/
void initializeStepper() {
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(accelRate);
  stepper.setCurrentPosition(motorPosition);
  pinMode(HALF_STEP_PIN, OUTPUT);
  digitalWrite(HALF_STEP_PIN, isHalfStep ? HIGH : LOW);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
}

/*
   Initialize the encoder. For now, just the turbo button
*/
void initializeEncoder() {
  pinMode(TURBO_PIN, INPUT_PULLUP);
}

/*
   Initialize the temperature sensor
*/
void initializeDHT() {
  pinMode(DHT_PIN, INPUT_PULLUP);
  dht.begin();
}
