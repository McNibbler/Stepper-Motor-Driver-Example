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
uint32_t numFlips = 0;
const double baseSpeedRpm = 100.0;
double speedRpm = baseSpeedRpm;         // Max 3170rpm - only 50rpm for this current Step/Rotation resolution

// Max rotations of the motor in either direction
const double kMaxRotationsPerDirection = 8.0;

// Timer variables - NOTE: milli() is in an unsigned 32 bit integer - timer will reset
// -- and likely break after ~50 days on time. I can fix this if it's really crucial.
uint32_t lastStepTime;
uint32_t accelerationTimer;
const uint32_t kAccelerationMilliseconds = 200;         // Milliseconds of acceleration time
const uint32_t kAccelerationSteps = 200;                // Steps it takes to accelerate to a new speed
uint32_t accelerationStepCounter = 0;
bool changingDirection = false;
bool slowing = false;

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
  if (abs(positionCounter) > (fullRotationsAsSteps(kMaxRotationsPerDirection) - accelerationDistanceCount) && !changingDirection) {
    accelerationStepCounter = 0;
    accelerationTimer = millis();
    changingDirection = true;
    slowing = true;
  }


  // Acceleration/direction controller - This is way nasty but I don't feel like restarting... it shouldn't be too bad to
  // -- read but let me know if it's too weird.
  if (changingDirection) {

    // stepTime is the amount of time between steps to update in an acceleration
    double stepTime = kAccelerationMilliseconds * (1.0 / kAccelerationSteps);
    // fraction is the number to multiply by the base speed to determine what the current RPM will be
    double fraction = static_cast<double>(accelerationStepCounter) / kAccelerationSteps;


    // If done stepping through an acceleration
    if (accelerationStepCounter >= kAccelerationSteps) {
      
      // Changing diretion after slowing
      if (slowing) {
        directionPositive = !directionPositive;
        digitalWrite(kDirectionPin, directionPositive);

        numFlips++;
        
        // Prints a status update after the driection change
        String updateString = "Sensor Actuation Count: " + String(sensorCounter);
        Serial.println(updateString);
        updateString = "Total complete rotations: " + String(static_cast<uint32_t>(stepsAsFullRotations(stepCounter)));
        Serial.println(updateString);
        updateString = directionPositive ? "Direction: Positive" : "Direction: Negative";
        Serial.println(updateString);
        updateString = "Total direction changes: " + String(numFlips);
        Serial.println(updateString);
        Serial.println("===================================================");
        Serial.println();
        
      }
      
      // Done changing direction
      else {
        changingDirection = false;
        positionCounter = 0;
      }

      // Resets everything
      slowing = false;
      accelerationStepCounter = 0;
    } // If done stepping through an accel
    

    // Updates the speed every acceleration step
    else if (millis() - accelerationTimer > stepTime) {
      double newSpeed = slowing ? baseSpeedRpm - (baseSpeedRpm * fraction) : baseSpeedRpm * fraction; 
      setSpeedRpm(newSpeed);
      accelerationStepCounter++;
      accelerationTimer = millis();
      
    } // if updating RPM
    
    
  } // changing direction
  
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
