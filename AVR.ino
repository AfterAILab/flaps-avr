#include <Arduino.h>
#include <Wire.h>
#include <Stepper.h>
#include <EEPROM.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include "env.h"

unsigned long lastRotation = 0;

// globals
int displayedLetter = 0;          // currently shown letter
int displayedAtStepperSpeed = 10; // stepper speed at which the letter was displayed
int letterNumber = 0;             // letter to show
int stepperSpeed = 10;            // current speed of stepper
const String letters[] = {" ", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "Ä", "Ö", "Ü", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", ":", ".", "-", "?", "!"};
Stepper stepper(STEPS, STEPPERPIN1, STEPPERPIN3, STEPPERPIN2, STEPPERPIN4); // stepper setup
bool lastInd1 = false;                                                      // store last status of phase
bool lastInd2 = false;                                                      // store last status of phase
bool lastInd3 = false;                                                      // store last status of phase
bool lastInd4 = false;                                                      // store last status of phase
bool rotating = false;                                                      // 1 = drum is currently rotating, 0 = drum is standing still
float remainderSteps = 0;                                                   // remainder steps for precise rotation
int offset;                                                              // Offset for calibration in steps, stored in EEPROM, gets read in setup
int i2cAddress;
bool offsetUpdatedFlag = false;

volatile uint8_t vHighByte;
volatile uint8_t vLowByte;
volatile uint8_t vMagneticZeroPositionLetterIndex;

volatile int receivedCommandInts[4];
volatile bool applyCommandFlag = false;

const unsigned long WAIT_TIME = 1000;
unsigned long previousMillis = 0;
// setup
void setup()
{
  // i2c adress switch
  pinMode(ADDR_BIT0, INPUT_PULLUP);
  pinMode(ADDR_BIT1, INPUT_PULLUP);
  pinMode(ADDR_BIT2, INPUT_PULLUP);
  pinMode(ADDR_BIT3, INPUT_PULLUP);
  pinMode(ADDR_BIT4, INPUT_PULLUP);
  pinMode(ADDR_BIT5, INPUT_PULLUP);
  pinMode(ADDR_BIT6, INPUT_PULLUP);
  pinMode(ADDR_BIT7, INPUT_PULLUP);

  // hall sensor
  pinMode(HALLPIN, INPUT);

  i2cAddress = getaddress(); // get I2C Address and save in variable
  // initialize serial
  Serial.begin(BAUDRATE);
  Serial.println("===== AfterAI Flaps AVR 1.1.0 =====");
  Serial.println("starting unit");
  Serial.print("I2CAddress: ");
  Serial.println(i2cAddress);

  uint8_t restartTimes = EEPROM.read(0);
  EEPROM.write(0, restartTimes + 1);
  int magneticZeroPositionLetterIndex = getMagneticZeroPositionLetterIndex();
  if (magneticZeroPositionLetterIndex < 0 || magneticZeroPositionLetterIndex >= NUM_FLAPS)
  {
    EEPROM.write(EEPROM_ADDR_MAGNETIC_ZERO_POSITION_LETTER_INDEX_BYTE, 0);
  }
  getOffset();
  if (offset < 0 || offset >= STEPS)
  {
    EEPROM.write(EEPROM_ADDR_OFFSET_HIGHER_BYTE, 0);
    EEPROM.write(EEPROM_ADDR_OFFSET_LOWER_BYTE, 0);
  }


  // I2C function assignment
  Wire.begin(i2cAddress);         // i2c address of this unit
  Wire.setWireTimeout(25000, true); // set timeout to 25ms
  Wire.onReceive(commandHandler); // call-function for transfered letter via i2c
  Wire.onRequest(requestHandler); // call-function if master requests unit state

  calibrate(true); // home stepper after startup
}

void loop()
{
  vHighByte = EEPROM.read(EEPROM_ADDR_OFFSET_HIGHER_BYTE);
  vLowByte = EEPROM.read(EEPROM_ADDR_OFFSET_LOWER_BYTE);
  vMagneticZeroPositionLetterIndex = EEPROM.read(EEPROM_ADDR_MAGNETIC_ZERO_POSITION_LETTER_INDEX_BYTE);

  if (applyCommandFlag)
  {
    applyCommand();
    applyCommandFlag = false;
  }

  if (offsetUpdatedFlag)
  {
    offsetUpdatedFlag = false;
    calibrate(true);
  }

  // check if new letter was received through i2c
  if (displayedLetter != letterNumber || displayedAtStepperSpeed != stepperSpeed)
  {
    Serial.print("Value over serial received: ");
    Serial.print(letterNumber);
    Serial.print(" Letter: ");
    Serial.print(letters[letterNumber]);
    Serial.println();
    // rotate to new letter
    rotateToLetter(letterNumber, stepperSpeed);
  }
}

// rotate to letter
void rotateToLetter(int toLetter, int stepperSpeed)
{
  unsigned long currentMillis = millis();
  if (lastRotation != 0 && lastRotation < currentMillis && (currentMillis - lastRotation < OVERHEATING_TIMEOUT_MILLIS))
  {
    Serial.print("rotateToLetter: Overheating protection. currentMillis: ");
    Serial.print(currentMillis);
    Serial.print(" lastRotation: ");
    Serial.print(lastRotation);
    Serial.print(" diff: ");
    Serial.println(currentMillis - lastRotation);
    delay(OVERHEATING_TIMEOUT_MILLIS);
    return;
  }
  if (toLetter < 0)
  {
    Serial.print("rotateToLetter: Unknown letter received: ");
    Serial.println(toLetter);
    return;
  }
  lastRotation = currentMillis;
  Serial.print("rotateToLetter: go to letter: ");
  Serial.println(letters[toLetter]);
  int rawDistance = toLetter - displayedLetter;
  int modDistance = (rawDistance + NUM_FLAPS) % NUM_FLAPS;
  int distance = modDistance == 0 ? NUM_FLAPS : modDistance; // showing the next letter is difficult, so we make a full revolution
  startMotor();
  stepper.setSpeed(stepperSpeed);
  float stepsPerLetter = (float)STEPS / (float)NUM_FLAPS;
  int stepsPerLetterInt = (int)stepsPerLetter;
  float remainderStepsPerLetter = stepsPerLetter - stepsPerLetterInt;
  int magneticZeroPositionLetterIndex = getMagneticZeroPositionLetterIndex();
  for (int i = 0; i < distance; i++, displayedLetter++)
  {
    if ((displayedLetter + 1) % NUM_FLAPS == magneticZeroPositionLetterIndex)
    {
      // reaching marker, go to calibrated offset position
      for (int j = 0; j < stepsPerLetter * 2; j++)
      {
        bool magnetDetected = digitalRead(HALLPIN) == 0; // TCS40DPR outputs 0 when detected strong magnetic field
        if (magnetDetected)
        {
          continue;
        }
        stepper.step(ROTATIONDIRECTION * 3);
      }
      Serial.println("revolver calibrated");
      remainderSteps = 0;
      continue;
    }
    remainderSteps += remainderStepsPerLetter;
    if (remainderSteps >= 1)
    {
      stepper.step(ROTATIONDIRECTION * (stepsPerLetterInt + 1)); // rotate to new letter
      remainderSteps -= 1;
    }
    else
    {
      stepper.step((int)ROTATIONDIRECTION * stepsPerLetterInt); // rotate to new letter
    }
  }
  // store new position
  displayedLetter = toLetter;
  displayedAtStepperSpeed = stepperSpeed;
  delay(100); // important to stop rotation before shutting off the motor to avoid rotation after switching off current
  stopMotor();
}

// NOTE: It is advised not to use Serial in interrupt routines
void commandHandler(int numBytes)
{
  for (int i = 0; i < numBytes; i++)
  {
    receivedCommandInts[i] = Wire.read();
  }
  applyCommandFlag = true;
}

void applyCommand() {
  // Write received bytes to correct variables
  int kind = receivedCommandInts[0];

  Serial.print("Command received: ");
  Serial.println(kind);

  if (kind == COMMAND_UPDATE_OFFSET)
  {
    vHighByte = receivedCommandInts[EEPROM_ADDR_OFFSET_HIGHER_BYTE];
    vLowByte = receivedCommandInts[EEPROM_ADDR_OFFSET_LOWER_BYTE];
    vMagneticZeroPositionLetterIndex = receivedCommandInts[EEPROM_ADDR_MAGNETIC_ZERO_POSITION_LETTER_INDEX_BYTE];
    offset = vHighByte << 8 | vLowByte;

    EEPROM.update(EEPROM_ADDR_OFFSET_HIGHER_BYTE, receivedCommandInts[EEPROM_ADDR_OFFSET_HIGHER_BYTE]);
    EEPROM.update(EEPROM_ADDR_OFFSET_LOWER_BYTE, receivedCommandInts[EEPROM_ADDR_OFFSET_LOWER_BYTE]);
    EEPROM.update(EEPROM_ADDR_MAGNETIC_ZERO_POSITION_LETTER_INDEX_BYTE, receivedCommandInts[EEPROM_ADDR_MAGNETIC_ZERO_POSITION_LETTER_INDEX_BYTE]);
    offsetUpdatedFlag = true;
  }
  else if (kind == COMMAND_SHOW_LETTER)
  {
    letterNumber = receivedCommandInts[1];
    stepperSpeed = receivedCommandInts[2];
    Serial.print("Letter received: ");
    Serial.print(letters[letterNumber]);
    Serial.print(" Speed: ");
    Serial.println(stepperSpeed);
  }
  else
  {
    Serial.println("Unknown command");
  }
}

// NOTE: It is advised not to use Serial in interrupt routines
void requestHandler()
{
  Wire.write(rotating);
  Wire.write(vHighByte);
  Wire.write(vLowByte);
  Wire.write(vMagneticZeroPositionLetterIndex);
  /*
  Serial.print("Unit info sent: ");
  Serial.print(highByte);
  Serial.print(" ");
  Serial.print(lowByte);
  Serial.print(" ");
  Serial.println(magneticZeroPositionLetterIndex);
  */
}

// returns the adress of the unit as int
int getaddress()
{
  int address = !digitalRead(ADDR_BIT0) + (!digitalRead(ADDR_BIT1) * 2) + (!digitalRead(ADDR_BIT2) * 4) + (!digitalRead(ADDR_BIT3) * 8) + (!digitalRead(ADDR_BIT4) * 16) + (!digitalRead(ADDR_BIT5) * 32) + (!digitalRead(ADDR_BIT6) * 64) + (!digitalRead(ADDR_BIT7) * 128);
  return address;
}

// gets magnet sensor offset from EEPROM in steps
void getOffset()
{
  int highByte = EEPROM.read(EEPROM_ADDR_OFFSET_HIGHER_BYTE);
  int lowByte = EEPROM.read(EEPROM_ADDR_OFFSET_LOWER_BYTE);
  offset = highByte << 8 | lowByte;
  Serial.print("offset: ");
  Serial.println(offset);
}

int getMagneticZeroPositionLetterIndex()
{
  return EEPROM.read(EEPROM_ADDR_MAGNETIC_ZERO_POSITION_LETTER_INDEX_BYTE);
}

// doing a calibration of the revolver using the hall sensor
int calibrate(bool initialCalibration)
{
  Serial.println("calibrate revolver");
  rotating = true;
  stepper.setSpeed(stepperSpeed);

  // Kick off
  bool magnetDetected = digitalRead(HALLPIN) == 0; // TCS40DPR outputs 0 when detected strong magnetic field
  if (magnetDetected)
  {
    stepper.step(ROTATIONDIRECTION * (STEPS / NUM_FLAPS) * (NUM_FLAPS - 1));
  }

  for (int steps = 0; steps < STEPS * 2; steps++)
  {
    bool magnetDetected = digitalRead(HALLPIN) == 0; // TCS40DPR outputs 0 when detected strong magnetic field
    Serial.print("calibrate: magnet detected: ");
    Serial.println(magnetDetected);
    if (!magnetDetected)
    {
      // not reached yet
      stepper.step(ROTATIONDIRECTION * 3);
      continue;
    }
    else
    {
      // reached marker, go to calibrated offset position
      stepper.step(ROTATIONDIRECTION * offset); // TODO: check if this is correct
      displayedLetter = 0;
      remainderSteps = 0;
      Serial.println("revolver calibrated");
      // Only stop motor for initial calibration
      if (initialCalibration)
      {
        stopMotor();
      }
      return steps;
    }
  }
  // seems that there is a problem with the marker or the sensor. turn of the motor to avoid overheating.
  displayedLetter = 0;
  Serial.println("calibration revolver failed");
  stopMotor();
  return -1;
}

// switching off the motor driver
void stopMotor()
{
  lastInd1 = digitalRead(STEPPERPIN1);
  lastInd2 = digitalRead(STEPPERPIN2);
  lastInd3 = digitalRead(STEPPERPIN3);
  lastInd4 = digitalRead(STEPPERPIN4);

  digitalWrite(STEPPERPIN1, LOW);
  digitalWrite(STEPPERPIN2, LOW);
  digitalWrite(STEPPERPIN3, LOW);
  digitalWrite(STEPPERPIN4, LOW);
  Serial.println("Motor Stop");
  rotating = false;
  delay(100);
}

void startMotor()
{
  Serial.println("Motor Start");
  rotating = true;
  digitalWrite(STEPPERPIN1, lastInd1);
  digitalWrite(STEPPERPIN2, lastInd2);
  digitalWrite(STEPPERPIN3, lastInd3);
  digitalWrite(STEPPERPIN4, lastInd4);
}