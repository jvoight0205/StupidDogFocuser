#define ENCODER_OPTIMIZE_INTERRUPTS

#include <DHT.h>
#include <Encoder.h>
#include <AccelStepper.h>

/*
    This is the firmware for an Arduino UNO based electronic focuser.
    Stupid Dog Observator
    Written and copyright by Jeff Voight, Feb 2020,

    In order to compile and deploy this, you will need to add the following libraries to your IDE.
      AccelStepper
      Encoder
      DHT Sensor Library


    TODO: Put a normal license here.

    Meanwhile:
    Consider this to be a terribly restrictive license.
    This software contains no warranty of any kind and makes no claims as to it's performance. If it
    erases your favorite photos and murders your friends, it's all on you.
*/

#define STEP_PIN 2
#define DIR_PIN 3
#define ENABLE_PIN A5
#define M0_PIN A4
#define M1_PIN A3
#define M2_PIN A2
#define ENC_A 5
#define ENC_B 6
#define TURBO_PIN 7
#define HALF_STEP_PIN A4
#define DHT_PIN 12
#define VERSION ".9"
#define TURBO_MULTIPLIER 200


#define DHTTYPE DHT11
//#define DHTTYPE DHT22

// TODO: Figure out how to link this directly to the ASCOM and INDI driver source includes.
#define HALT "HA"
#define IS_ENABLED "GE"
#define IS_REVERSED "GR"
#define GET_MICROSTEP "GM"
#define GET_HIGH_LIMIT "GH"
#define GET_LOW_LIMIT "GL"
#define GET_SPEED "GS"
#define GET_TEMPERATURE "GT"
#define GET_POSITION "GP"
#define IS_MOVING "GV"
#define ABSOLUTE_MOVE "AM%ld"
#define RELATIVE_MOVE "RM%ld"
#define REVERSE_DIR "RD"
#define FORWARD_DIR "FD"
#define SYNC_MOTOR "SY%ld"
#define ENABLE_MOTOR "EN"
#define DISABLE_MOTOR "DI"
#define SET_MICROSTEP "SM%u"
#define SET_SPEED "SP%u"
#define SET_HIGH_LIMIT "SH%ld"
#define SET_LOW_LIMIT "SL%ld"
#define TRUE_RESPONSE "T"
#define FALSE_RESPONSE "F"
#define SIGNED_RESPONSE "%d"
#define UNSIGNED_RESPONSE "%u"
#define LONG_RESPONSE "%ld"
#define FLOAT_RESPONSE "%f"
#define GET_VERSION "VE"

#define BUFFER_SIZE 15

int16_t encoderPosition = 0; // Set to be incorrect on purpose so that it gets updated in setup.
int32_t motorPosition = 0; //

// speed needs to map to 255 being the max
uint16_t maxStepperSpeed = 1500; // 1500 for full stepping. 15000 for 1/16th stepping
int indiSpeed = 255;
uint16_t accelRate = 3000; // I've had excellent results with 3000
uint8_t microstep = 1; // can only be multiples of 2
bool enabled = true;
bool reversed = false;
int32_t lowLimit = -10800; // 2 full turns of a 17:1 geared 200 step stepper
int32_t highLimit = 10800; // in both directions

char buffer[BUFFER_SIZE];
char commandLine[BUFFER_SIZE];
bool newData = false;

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN) ; // Skipping enable pin on this
Encoder encoder(ENC_A, ENC_B);
DHT dht(DHT_PIN, DHTTYPE);

void setup() {
  Serial.begin(115200);

  initializeDHT();
  initializeStepper();
  initializeEncoder();
  output(VERSION);
}

void output(String text) {
  Serial.print(text);
  Serial.println("!");
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

  readSerial();
  stepper.run();

  interpretSerial();
  stepper.run();

  interpretEncoder();
  stepper.run();

  motorPosition = stepper.currentPosition();
}

/**
   Determine if a valid command has been received and act on it.
   This parser is built as a bit of a decision tree for performance reasons.
*/
void interpretSerial() {
  if (newData) {
    long myLong = 0;
    double myDouble = 0.0;

    strcpy(buffer, commandLine);

    if (strcmp(commandLine, HALT) == 0) {
      stepper.stop();
      while (stepper.isRunning()) { // Call run() as fast as possible to allow motor to halt as quickly as possible
        for (int i = 0; i < 10; i++) { // Can go even faster to halt if we take a bunch of steps between isRunning calls.
          stepper.run();
        }
      }
      long currentPosition = stepper.currentPosition();
      sprintf(buffer, LONG_RESPONSE, currentPosition);
    }

    else if (strcmp(commandLine, IS_ENABLED) == 0) {
      strcpy(buffer, enabled ? TRUE_RESPONSE : FALSE_RESPONSE);
    }

    else if (strcmp(commandLine, IS_REVERSED) == 0) {
      strcpy(buffer, reversed ? REVERSE_DIR : FORWARD_DIR);
    }

    else if (strcmp(commandLine, IS_MOVING) == 0) {
      strcpy(buffer, stepper.isRunning() ? TRUE_RESPONSE : FALSE_RESPONSE);
    }

    else if (strcmp(commandLine, GET_VERSION) == 0) {
      sprintf(buffer, VERSION);
    }

    else if (strcmp(commandLine, GET_MICROSTEP) == 0) {
      sprintf(buffer, UNSIGNED_RESPONSE, microstep);
    }

    else if (strcmp(commandLine, GET_HIGH_LIMIT) == 0) {
      sprintf(buffer, LONG_RESPONSE, highLimit);
    }

    else if (strcmp(commandLine, GET_LOW_LIMIT) == 0) {
      sprintf(buffer, LONG_RESPONSE, lowLimit);
    }

    else if (strcmp(commandLine, GET_SPEED) == 0) {
      int indiSpeed = map(stepper.speed(), 1, maxStepperSpeed, 1, 255);
      sprintf(buffer, UNSIGNED_RESPONSE, indiSpeed);
    }

    else if (strcmp(commandLine, GET_TEMPERATURE) == 0) {
      // because of some kind of Arduino bullshit, %f doesn't work the way it's supposed to
      sprintf(buffer, SIGNED_RESPONSE, 33); //dht.readTemperature());
    }

    else if (strcmp(commandLine, GET_POSITION) == 0) {
      long currentPosition = stepper.currentPosition();
      sprintf(buffer, LONG_RESPONSE, currentPosition);
    }

    else if (strcmp(commandLine, ENABLE_MOTOR) == 0) {
      enabled = true;
      stepper.enableOutputs();
      digitalWrite(ENABLE_PIN, LOW);
      strcpy(buffer, enabled ? TRUE_RESPONSE : FALSE_RESPONSE);
    }

    else if (strcmp(commandLine, DISABLE_MOTOR) == 0) {
      enabled = false;
      stepper.disableOutputs();
      digitalWrite(ENABLE_PIN, HIGH);
      strcpy(buffer, enabled ? TRUE_RESPONSE : FALSE_RESPONSE);
    }

    else if (strcmp(commandLine, REVERSE_DIR) == 0) {
      reversed = true;
      strcpy(buffer, reversed ? REVERSE_DIR : FORWARD_DIR);
    }

    else if (strcmp(commandLine, FORWARD_DIR) == 0) {
      reversed = false;
      strcpy(buffer, reversed ? REVERSE_DIR : FORWARD_DIR);
    }

    else if (sscanf(commandLine, SYNC_MOTOR, &myLong) == 1) {
      stepper.setCurrentPosition(myLong);
      sprintf(buffer, LONG_RESPONSE, stepper.currentPosition());
    }

    else if (sscanf(commandLine, ABSOLUTE_MOVE, &myLong) == 1 ) {
      stepper.moveTo(myLong * (reversed ? -1 : 1));
      sprintf(buffer, LONG_RESPONSE, stepper.currentPosition());
    }

    else if (sscanf(commandLine, RELATIVE_MOVE, &myLong) == 1 ) {
      stepper.moveTo(stepper.currentPosition() + myLong * (reversed ? -1 : 1));
      sprintf(buffer, LONG_RESPONSE, stepper.currentPosition());
    }

    else if (sscanf(commandLine, SET_SPEED, &myLong) == 1 ) {
      indiSpeed = myLong;
      int stepperSpeed = map(myLong, 1, 255, 1, maxStepperSpeed);
      stepper.setMaxSpeed(stepperSpeed);
      sprintf(buffer, UNSIGNED_RESPONSE, indiSpeed);
    }

    else if (sscanf(commandLine, SET_MICROSTEP, &myLong) == 1 ) {
      setMicrostep((int)myLong);
      sprintf(buffer, UNSIGNED_RESPONSE, microstep);
    }

    else if (sscanf(commandLine, SET_HIGH_LIMIT, &myLong) == 1 ) {
      if (myLong < 999999999)
        highLimit = myLong;
      sprintf(buffer, LONG_RESPONSE, highLimit);
    }

    else if (sscanf(commandLine, SET_LOW_LIMIT, &myLong) == 1 ) {
      if (myLong > -999999999)
        lowLimit = myLong;
      sprintf(buffer, LONG_RESPONSE, lowLimit);
    }

    output(buffer);
    newData = false;


    //Serial.print("dbg:'");Serial.print(command);Serial.println("'");

    // Parse the command out from between the chars
    //  String commandString = command.substring(1, command.length() - 1);
    //
    //  switch (commandString.charAt(0)) {
    //
    //    // Temperature Conversion. This probably isn't necessary
    //    case 'C':
    //      dht.read();
    //      break;
    //
    //    // Feed Commands
    //    case 'F':
    //      switch (commandString.charAt(1)) {
    //
    //        // Go-to (move) to new set position
    //        case 'G':
    //          stepper.moveTo(newMotorPosition);
    //          break;
    //
    //        // HALT movement
    //        case 'Q':
    //          stepper.stop();
    //          while (stepper.isRunning()) { // Call run() as fast as possible to allow motor to halt as quickly as possible
    //            for (int i = 0; i < 20; i++) { // Can go even faster to halt if we take a bunch of steps between isRunning calls.
    //              stepper.run();
    //            }
    //          }
    //          motorPosition = stepper.currentPosition();
    //          newMotorPosition = motorPosition; // Make sure we don't take off again.
    //          break;
    //
    //        // Error
    //        default:
    //          error(command);
    //      }
    //      break;
    //
    //    // Getter Commands
    //    case 'G':
    //      switch (commandString.charAt(1)) {
    //
    //        // Temperature coefficient
    //        case 'C':
    //          respond(format2UHex(temperatureCoefficient));
    //          break;
    //
    //        // Stepping delay
    //        case 'D':
    //          respond(format2UHex(32));
    //          break;
    //
    //        // Half-step
    //        case 'H':
    //          respond(format2UHex(isHalfStep ? 255 : 0));
    //          break;
    //
    //        // Moving
    //        case 'I':
    //          respond(format2UHex(stepper.isRunning() ? 1 : 0));
    //          break;
    //
    //        // New Position
    //        case 'N':
    //          respond(formatUHex(newMotorPosition));
    //          break;
    //
    //        // Position
    //        case 'P':
    //          respond(formatUHex(stepper.currentPosition()));
    //          break;
    //
    //        // Temperature
    //        case 'T':
    //          respond(formatFHex(dht.readTemperature()));
    //          break;
    //
    //        // Version
    //        case 'V':
    //          respond(VERSION);
    //          break;
    //
    //        // Error
    //        default:
    //          error(command);
    //
    //      }
    //      break;
    //
    //    // Setter commands
    //    case 'S':
    //      switch (commandString.charAt(1)) {
    //
    //        // Temperature coefficient
    //        case 'C':
    //          temperatureCoefficient = parse2UHex(commandString.substring(2));
    //          break;
    //
    //        // Stepping delay
    //        case 'D':
    //          // Ignored
    //          break;
    //
    //        // Full-step mode
    //        case 'F':
    //          isHalfStep = false;
    //          digitalWrite(HALF_STEP_PIN, HIGH);
    //          break;
    //
    //        // Half-step mode
    //        case 'H':
    //          isHalfStep = true;
    //          digitalWrite(HALF_STEP_PIN, HIGH);
    //          break;
    //
    //        // New position
    //        case 'N':
    //          newMotorPosition = parseUHex(commandString.substring(2));
    //          break;
    //
    //        // Current position
    //        case 'P':
    //          newMotorPosition = parseUHex(commandString.substring(2));
    //          stepper.setCurrentPosition(newMotorPosition);
    //          motorPosition = newMotorPosition; // Make sure everything points to the same number
    //          break;
    //
    //        // Error
    //        default:
    //          error(command);
    //      }
    //      break;
    //
    //    // Activate Temperature Compensation Focusing
    //    case '+':
    //      break; // Not implemented
    //
    //    // Deactivate Temperature Compensation Focusing
    //    case '-':
    //      break; // Not implemented
    //
    //    case 'P':
    //      break;
    //
    //    // Error
    //    default:
    //      error(command);
    //  }
  }
}


void readSerial() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        commandLine[ndx] = rc;
        ndx++;
        if (ndx >= BUFFER_SIZE) {
          ndx = BUFFER_SIZE - 1;
        }
      }
      else {
        commandLine[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}



/*
   Determine if the encoder has moved this cycle. If so, make the adjustments
   to the stepper and update variables.
*/
void interpretEncoder() {
  long newEncoderPosition = encoder.read();
  if (newEncoderPosition != encoderPosition) {                  // If there's any change from the previous read,
    long encoderChange = newEncoderPosition - encoderPosition;  // determine the difference,
    stepper.moveTo(stepper.targetPosition() + encoderChange);                         // set the new target
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
  stepper.setMaxSpeed(map(indiSpeed, 1, 255, 1, maxStepperSpeed));
  stepper.setAcceleration(accelRate);
  stepper.setCurrentPosition(motorPosition);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);
  pinMode(M0_PIN, OUTPUT);
  pinMode(M1_PIN, OUTPUT);
  pinMode(M2_PIN, OUTPUT);
  setMicrostep(microstep);
}

void setMicrostep(int ms) {
  uint8_t m0 = LOW, m1 = LOW, m2 = LOW;
  switch (ms) {
    case 1:
      microstep = 1;
      break;
    case 2:
      microstep = 2;
      m0 = HIGH;
      break;
    case 4:
      microstep = 4;
      m1 = HIGH;
      break;
    case 8:
      microstep = 8;
      m0 = HIGH; m1 = HIGH;
      break;
    case 16:
      microstep = 16;
      m2 = HIGH;
      break;
    case 32:
      microstep = 32;
      m0 = HIGH; m2 = HIGH;
      break;
    default:
      break;
  }
  digitalWrite(M0_PIN, m0);
  digitalWrite(M1_PIN, m1);
  digitalWrite(M2_PIN, m2);
}
/*
   Initialize the encoder. For now, just the turbo button
*/
void initializeEncoder() {
  pinMode(TURBO_PIN, INPUT);
}

/*
   Initialize the temperature sensor
*/
void initializeDHT() {
  pinMode(DHT_PIN, INPUT_PULLUP);
  dht.begin();
}
