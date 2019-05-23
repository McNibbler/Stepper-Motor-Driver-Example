/* Stepper Motor Driver Example
 * 5/22/2019
 * Thomas Kaunzinger
 * 
 * Quick script to show control example for motor driver and corresponding light sensor.
 * Here you go Philby lol.
 */

// Relevant libraries
#include <stdint.h>

// Preprocessor conditionals for debugging. Comment out either line to disable that feature
#define USING_MOTOR
#define USING_SENSOR

////////////////////////////////////
// CONSTANTS AND GLOBAL VARIABLES //
////////////////////////////////////
// Pinout constants
const uint8_t kEnablePin = 9;
const uint8_t kDirectionPin = 6;
const uint8_t kStepPin = 3;
const uint8_t kSensorPin = 2;

/////////////////////
// Motor Variables //
/////////////////////
// Motor Constants
uint16_t kStepsPerRotation = 200;    // Defined by user when setting up motor

// States of the stepper
bool stepState = 0;
bool directionPositive = false;
int32_t positionCounter = 0;
uint32_t stepCounter = 0;
const double baseSpeedRpm = 20.0;
double speedRpm = baseSpeedRpm;         // Max 3170rpm - only 50rpm for this current Step/Rotation resolution
double preChangeRpm = speedRpm;
bool speedyToggle = false;              // Determines if the loop is making the motor go fast or not
const uint32_t kSpeedyMultiplier = 10;  // How many times the current speed to multiply to in speedy mode

// Max rotations of the motor in either direction
const double kMaxRotationsPerDirection = 30.0;

// Timer variables - NOTE: milli() is in an unsigned 32 bit integer - timer will reset
// -- and likely break after ~50 days on time. I can fix this if it's really crucial.
uint32_t lastStepTime;
uint32_t speedyTimer;
uint32_t accelerationTimer;
const uint32_t kSpeedyTimerToggleMilliseconds = 2000;   // Toggles speed mode after this many milliseconds
const uint32_t kAccelerationMilliseconds = 200;         // Milliseconds of acceleration time
const uint32_t kAccelerationSteps = 200;                // Steps it takes to accelerate to a new speed
uint32_t accelerationStepCounter = 0;
bool accelerating = false;
bool changingDirection = false;
bool changingDirectionSlowing = false;

//////////////////////
// Sensor variables //
//////////////////////
// Increments with each actuation of the sensor
uint32_t sensorCounter = 0;

// Timer variables. Note: same overflow issue as motor timer can happen here.
uint32_t sensorDebounceTimer;
const uint32_t kSensorDebounceMilliseconds = 10;  // Sensor is actuated only actuated after this many ms
bool sensorActuated = false;                      // For updating actuation state


/////////////////////
// MAIN SETUP LOOP //
/////////////////////
// Sets pinmodes and begins USB communication at 9600 baud rate
void setup() {
  pinMode(kEnablePin, OUTPUT);
  pinMode(kDirectionPin, OUTPUT);
  pinMode(kStepPin, OUTPUT);
  pinMode(kSensorPin, INPUT_PULLUP);  // Don't think the pullup matters but w/e

  Serial.begin(9600);

  digitalWrite(kEnablePin, HIGH);

  lastStepTime = millis();
  speedyTimer = millis();
  sensorDebounceTimer = millis();
}


/////////////////////////
// MAIN EXECUTION LOOP //
/////////////////////////
void loop() {

  /////////////////////////
  // Motor functionality //
  /////////////////////////
  #ifdef USING_MOTOR
  
  // Stepper control: flips the step pin at the appropriate interval
  if (millis() - lastStepTime > msPerStep()) {
    lastStepTime = millis();
    WriteStep();
  }

  double accelerationDistanceCount = ((speedRpm / 60000.0) * (kAccelerationMilliseconds)) / 2;  // Math... disgusting
  // Flips the position after reaching the max number of rotations in either direction
  if (abs(positionCounter) > (fullRotationsAsSteps(kMaxRotationsPerDirection) - accelerationDistanceCount) && !(accelerating || changingDirection)) {
    accelerationStepCounter = 0;
    accelerating = true;
    accelerationTimer = millis();
    changingDirection = true;
    changingDirectionSlowing = true;
    preChangeRpm = speedRpm;
  }

  // Changes the speed of the motor every kSpeedyTimerToggleMilliseconds ms
  // -- toggles between a fast and slow mode - could be expanded easily.
  // -- Could easily to modify this to change after a number of rotations using
  // -- a counter like above.
  if ((millis() - speedyTimer > kSpeedyTimerToggleMilliseconds) && !(accelerating || changingDirection)) {
    accelerationStepCounter = 0;
    accelerating = true;
    speedyTimer = millis();
    accelerationTimer = millis();
  }

  // Acceleration controller - This is way nasty but I don't feel like restarting... it shouldn't be too bad to
  // -- read but let me know if it's too weird.
  if (accelerating) {
    double stepTime = kAccelerationMilliseconds * (1.0 / kAccelerationSteps);
    double fraction = static_cast<double>(accelerationStepCounter) / kAccelerationSteps;

    // If done stepping
    if (accelerationStepCounter >= kAccelerationSteps) {
      // Still accelerating but changing diretion
      if (changingDirection && changingDirectionSlowing) {
        changingDirectionSlowing = false;
        directionPositive = !directionPositive;
        digitalWrite(kDirectionPin, directionPositive);
      }
      // Done changing direction
      else if (changingDirection) {
        changingDirection = false;
        changingDirectionSlowing = false;
        positionCounter = 0;
        accelerating = false;
      }
      // Done speed changing
      else {
        accelerating = false;
        speedyToggle = !speedyToggle;
      }
      // Resets counter
      accelerationStepCounter = 0;
    }

    // Updates the speed every acceleration step
    else if (millis() - accelerationTimer > stepTime) {
      double newSpeed;
      // Direction change
      if (changingDirection) {
        if (!changingDirectionSlowing) { newSpeed = fraction * preChangeRpm; }
        else { newSpeed = preChangeRpm - fraction * preChangeRpm; }
      }
      // Speed toggle
      else {
        if (!speedyToggle) { newSpeed = baseSpeedRpm + fraction * (kSpeedyMultiplier - 1) * baseSpeedRpm; }
        else {newSpeed = kSpeedyMultiplier * baseSpeedRpm - fraction * (kSpeedyMultiplier - 1) * baseSpeedRpm; }
      }
      setSpeedRpm(newSpeed);
      accelerationStepCounter++;
      accelerationTimer = millis();
    } // if not updating speed
  } // accelerating
  
  #endif  // USING_MOTOR


  //////////////////////////
  // Sensor functionality //
  //////////////////////////
  #ifdef USING_SENSOR

  // Debounces the Sensor pin by only actuating after kSensorDebounceMilliseconds number of milliseconds,
  // -- then increments the sensor counter by one. Resets debounce timer and actuation state when low
  if ( digitalRead(kSensorPin) ) {
    if ((millis() - sensorDebounceTimer > kSensorDebounceMilliseconds) && !sensorActuated) {
      sensorActuated = true;
      sensorCounter++;
      String updateString = "Sensor Actuation Count: " + String(sensorCounter);
      Serial.println(updateString);
      updateString = "Total complete rotations: " + String(static_cast<uint32_t>(stepsAsFullRotations(stepCounter)));
      Serial.println(updateString);
    }
  }
  else {
    sensorDebounceTimer = millis();
    sensorActuated = false;
  }

  #endif  // USING_SENSOR

}


//////////////////////
// HELPER FUNCTIONS //
//////////////////////

// Performs a single step and adjusts the counter in the appropriate direction
void WriteStep() {
  directionPositive ? positionCounter += stepState : positionCounter -= stepState;  // Increments the directional steps
  stepCounter += stepState;                                                         // Increments the total steps (only when high)
  digitalWrite(kStepPin, stepState);                                                // Writes current step state
  stepState = !stepState;                                                           // Flips the next step state
}

// Returns the number of milliseconds per step for the motor to function at desired speeds
uint32_t msPerStep() {
  return static_cast<uint32_t>((60000.0 / speedRpm) / kStepsPerRotation / 2);
}

// Returns the number of motor steps required to perform a desired number of rotations
// -- Note: rotations should be positive.
uint32_t fullRotationsAsSteps(double rotations) {
  return static_cast<uint32_t>(rotations * kStepsPerRotation);
}

// Returns the number of full rotations given a number of steps
double stepsAsFullRotations(uint32_t steps) {
  return static_cast<double>(steps) / kStepsPerRotation;
}

// Function call to set a new RPM
void setSpeedRpm(double rpm) {
  speedRpm = rpm;
}
