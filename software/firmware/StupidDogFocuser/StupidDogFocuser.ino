#define ENCODER_OPTIMIZE_INTERRUPTS

#include <DHT.h>
#include <Encoder.h>
#include <AccelStepper.h>

/*
    This is the firmware for an Arduino UNO based electronic focuser.
    Stupid Dog Observator
    Written and copyright by Jeff Voight, Feb 2020,

    In order to compile and deploy this, you will need to add the following libraries to your IDE.
    * AccelStepper
    * Encoder
    * DHT Sensor Library

    
    TODO: Put a normal license here.

    Meanwhile:
    Consider this to be a terribly restrictive license.
    This software contains no warranty of any kind and makes no claims as to it's performance. If it
    erases your favorite photos and murders your friends, it's all on you.
*/

#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN A5
#define ENC_A 5
#define ENC_B 6
#define TURBO_PIN 7
#define HALF_STEP_PIN A4
#define DHT_PIN 12
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

void error(String command){
  respond("Error in command: " + command);
}

/**
   Determine if a valid command has been received and act on it.
   This parser is built as a bit of a decision tree for performance reasons.
*/
void interpretSerial(String command) {
  // Parse the command out from between the chars
  String commandString = command.substring(1, command.length() - 1);

  switch (commandString.charAt(0)) {

    // Temperature Conversion. This probably isn't necessary
    case 'C':
      dht.read();
      break;

    // Feed Commands
    case 'F':
      switch (commandString.charAt(1)) {

        // Go-to (move) to new set position
        case 'G':
          stepper.moveTo(newMotorPosition);
          break;

        // HALT movement
        case 'Q':
          stepper.stop();
          while (stepper.isRunning()) { // Call run() as fast as possible to allow motor to halt as quickly as possible
            for (int i = 0; i < 20; i++) { // Can go even faster to halt if we take a bunch of steps between isRunning calls.
              stepper.run();
            }
          }
          motorPosition = stepper.currentPosition();
          newMotorPosition = motorPosition; // Make sure we don't take off again.
          break;

        // Error
        default:
          error(command);
      }
      break;

    // Getter Commands
    case 'G':
      switch (commandString.charAt(1)) {

        // Temperature coefficient
        case 'C':
          respond(format2UHex(temperatureCoefficient));
          break;

        // Stepping delay
        case 'D':
          respond(format2UHex(32));
          break;

        // Half-step
        case 'H':
          respond(format2UHex(isHalfStep ? 255 : 0));
          break;

        // Moving
        case 'I':
          respond(format2UHex(stepper.isRunning() ? 1 : 0));
          break;

        // New Position
        case 'N':
          respond(formatUHex(newMotorPosition));
          break;

        // Position
        case 'P':
          respond(formatUHex(stepper.currentPosition()));
          break;

        // Temperature
        case 'T':
          respond(formatFHex(dht.readTemperature()));
          break;

        // Version
        case 'V':
          respond(formatVersion(VERSION));
          break;

        // Error
        default:
          error(command);

      }
      break;

    // Setter commands
    case 'S':
      switch (commandString.charAt(1)) {

        // Temperature coefficient
        case 'C':
          temperatureCoefficient = parse2UHex(commandString.substring(2));
          break;

        // Stepping delay
        case 'D':
          // Ignored
          break;

        // Full-step mode
        case 'F':
          isHalfStep = false;
          digitalWrite(HALF_STEP_PIN, HIGH);
          break;

        // Half-step mode
        case 'H':
          isHalfStep = true;
          digitalWrite(HALF_STEP_PIN, HIGH);
          break;

        // New position
        case 'N':
          newMotorPosition = parseUHex(commandString.substring(2));
          break;

        // Current position
        case 'P':
          newMotorPosition = parseUHex(commandString.substring(2));
          stepper.setCurrentPosition(newMotorPosition);
          motorPosition = newMotorPosition; // Make sure everything points to the same number
          break;

        // Error
        default:
          error(command);
      }
      break;

    // Activate Temperature Compensation Focusing
    case '+':
      break; // Not implemented

    // Deactivate Temperature Compensation Focusing
    case '-':
      break; // Not implemented

    case 'P':
      break;

    // Error
    default:
      error(command);
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
