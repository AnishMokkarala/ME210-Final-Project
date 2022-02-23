#include <Arduino.h>

/*---------------Pin Numbers----------------*/
#define SPIN_PIN_DIR 9
#define SPIN_PIN_EN 10

/*---------------State Definitions--------------------------*/
typedef enum {
    WAITING, LOADING, NAVIGATE_TO_SCORE, ALIGN_TO_SCORE, SCORING, NAVIGATE_TO_LOAD, ALIGN_TO_LOAD
} Robot_states;
typedef enum{
    FWD, BWD, TURN_LEFT, TURN_RIGHT, STILL
} Wheel_states;

/*---------------Module Variables---------------------------*/
Robot_states robot_state;
Wheel_states wheel_state;

int currentTime;
int spinDir = 1;
int spinEnable = 0;
 
/*---------------Function Declarations----------------*/
void checkGlobalEvents(void);
void spinFlap(void);

/*---------------IntervalTimer Declarations----------------*/

/*---------------Setup Code----------------*/
void setup() {
  Serial.begin(9600);
  
  // initialize all states here
  robot_state = WAITING;
  wheel_state = STILL;
    
  // initialize pin I/O here
  pinMode(SPIN_PIN_DIR, OUTPUT);
  pinMode(SPIN_PIN_EN, OUTPUT);
    
  // initialize interval timers here

  // other setup code here
  currentTime = millis();
  digitalWrite(SPIN_PIN_DIR, spinDir);
  digitalWrite(SPIN_PIN_EN, spinEnable);
}

/*---------------Main Loop----------------*/
void loop() { 
  checkGlobalEvents();
    
  switch (robot_state) {
    case WAITING:   // update this!
        currentTime = millis();
        robot_state = LOADING; // move to loading
        break;
    case LOADING:
        wheel_state = STILL; // ensure wheels are turned off
        spinFlap();
        break;
    case NAVIGATE_TO_SCORE:
        break;
    case ALIGN_TO_SCORE:
        break;
    case SCORING:
        break;
    case NAVIGATE_TO_LOAD:
        break;
    case ALIGN_TO_LOAD:
        break;
    default:
        wheel_state = STILL; // turn wheels off
        robot_state = WAITING; // enter waiting state
  }
    
  switch (wheel_state) {
    case STILL:
        break;
    case FWD:
        break;
    case BWD:
        break;
    case TURN_LEFT:
        break;
    case TURN_RIGHT:
        break;
    default:
        wheel_state = STILL;
  }
}

/*---------------Function Definitions----------------*/
void checkGlobalEvents(void){
  // watch for 2min10s timer
  // red vs blue team
}

void spinFlap(void) {
  if (spinEnable == 0) {
    spinEnable = 1;
    digitalWrite(SPIN_PIN_EN, spinEnable); // begin spinning motor if not already spinning
  }

  if (millis() - currentTime > 3000){ // spin for three seconds
    spinEnable = 0; // turn motor off
    digitalWrite(SPIN_PIN_EN, spinEnable);
    wheel_state = BWD; // set wheels to move backwards
    robot_state = NAVIGATE_TO_SCORE; // begin navigation to drop zone
  }
}