#include <Arduino.h>
#include <AccelStepper.h>

/*---------------Pin Numbers----------------*/
#define SPIN_PIN_DIR 9
#define SPIN_PIN_EN 10
#define ENABLE1 A8//input to EN pin of RIGHT MOTOR
#define ENABLE2 A9//input to EN pin of LEFT MOTOR
#define IN1 0//input to IN1
#define IN2 1//input to IN2
#define IN3 2//input to IN3
#define IN4 5//input to IN4
#define PIN_STEP 3//pinStep for Stepper Motor
#define PIN_DIR 4 //pinDirection for Stepper Motor
#define LIM1 13//top limit switch
#define LIM2 20//bottom limit switch
/*---------------State Definitions--------------------------*/
typedef enum {
    WAITING, LOADING, MOVE, TURN, ALIGN, SCORING
} Robot_states;

/*---------------Module Variables---------------------------*/
Robot_states robot_state;

int currentTime;
int spinDir = 1;
int spinEnable = 0;
int dist_th=15;//distance threshold
int rev=0;
int turns=0;
int hdes=0;
int head=0;
int b=0; //backup counter
AccelStepper myStepper1(1,PIN_STEP,PIN_DIR);//stepper object
int maxspeed=300;//maximum speed of stepper in steps/sec

/*---------------Function Declarations----------------*/
void checkGlobalEvents(void);
void spinFlap(void);
void motorsStraight(int v1, int v2, int speed=255);
/* values of v1 & v2 determine the direction of rotation of the motors.
v1=HIGH & v2=LOW, bot moves forward
v1=LOW & v2=HIGH, bot moves backward
speed is read from the potentiometer on the bot*/
void motorsStill(void);
void motorsTurn(int v1,int v2,int feedback=100,int speed=128);
void backUp(void);
int obsDist1(void);//output from Ultrasonic Sensor at the top
int obsDist2(void);//output from Ultrasonic Sensor at the bottom
int line1(void);//output from Line Sensor 1
int line2(void);//output from Line Sensor 2
/*---------------IntervalTimer Declarations----------------*/
IntervalTimer backUpTimer;

/*---------------Setup Code----------------*/
void setup() {
  Serial.begin(9600);
  
  // initialize all states here
  robot_state = WAITING;
  //wheel_state = STILL;
    
  // initialize pin I/O here
  pinMode(SPIN_PIN_DIR, OUTPUT);
  pinMode(SPIN_PIN_EN, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LIM1,INPUT);
  pinMode(LIM2,INPUT);
    
  // initialize interval timers here

  // other setup code here
  currentTime = millis();
  digitalWrite(SPIN_PIN_DIR, spinDir);
  digitalWrite(SPIN_PIN_EN, spinEnable);
  myStepper1.setMaxSpeed(maxspeed);
  myStepper1.setSpeed(maxspeed);
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
        //wheel_state = STILL; // ensure wheels are turned off
        motorsStill();
        spinFlap();
        break;
    case MOVE:
        if(obsDist1()<dist_th){
          hdes=0;
          motorsTurn(HIGH, LOW);
          robot_state=TURN;         
          turns++;
        }
        else if(obsDist2()<dist_th){
          hdes=90;
          robot_state=TURN; 
          turns++;
          if (rev==0) motorsTurn(LOW,HIGH); 
          else if (rev==1) motorsTurn(HIGH,LOW);         
        }
        break;
    case TURN:
        if(head==hdes){
          if(turns==1){
            robot_state=MOVE;
            motorsStraight(HIGH,HIGH);
          }
          else if(turns==2){
            robot_state=ALIGN;
            if (rev==0)motorsStraight(HIGH,HIGH);
            else if (rev==1) motorsStraight(LOW,LOW);
          }

        }
        break;
    case ALIGN:
        if (line1() == HIGH && line2() == HIGH){
          if(rev==0){
            motorsStill();
            myStepper1.setPinsInverted(true);
            myStepper1.runSpeed();
            robot_state=SCORING;
            rev=1;
          }
          else if(rev==1){
            robot_state=LOADING;
            turns=0;
          }
        } 
        break;
    case SCORING:
        if (digitalRead(LIM1)==HIGH){
          myStepper1.setPinsInverted(false);
          myStepper1.runSpeed();
          backUpTimer.begin(backUp,1000);          
        }
        else myStepper1.runSpeed();//keep running stepper motor if the Lim1 is not triggered
        break;
    default:
        //wheel_state = STILL; // turn wheels off
        motorsStill();
        robot_state = WAITING; // enter waiting state
  }
}

/*---------------Function Definitions----------------*/
void checkGlobalEvents(void){
  // watch for 2min10s timer
  // red vs blue team
  // read potentiometer input for motor speed determination
  if (digitalRead(LIM2)==LOW && robot_state!= SCORING){
          myStepper1.runSpeed();
  }
}
void spinFlap(void){
  if (spinEnable == 0) {
    spinEnable = 1;
    digitalWrite(SPIN_PIN_EN, spinEnable); // begin spinning motor if not already spinning
  }

  if (millis() - currentTime > 3000){ // spin for three seconds
    spinEnable = 0; // turn motor off
    digitalWrite(SPIN_PIN_EN, spinEnable);
    //wheel_state = BWD; // set wheels to move backwards
    robot_state = MOVE; // begin navigation to drop zone
    motorsStraight(HIGH, HIGH);
  }
}

void motorsStraight(int v1, int v2, int speed){
  analogWrite(ENABLE1,speed);
  analogWrite(ENABLE2,speed);
  digitalWrite(IN1,v1);
  digitalWrite(IN2,!v1);
  digitalWrite(IN3,v2);
  digitalWrite(IN4,!v2);
}

void motorsStill(void){
  digitalWrite(IN1,LOW);//or HIGH
  digitalWrite(IN2,LOW);//or HIGH
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

void motorsTurn(int v1,int v2,int feedback,int speed){
  analogWrite(ENABLE1,speed+feedback);
  analogWrite(ENABLE2,speed-feedback);
  digitalWrite(IN1,v1);
  digitalWrite(IN2,!v1);
  digitalWrite(IN3,v2);
  digitalWrite(IN4,!v2);
}

void backUp(void){
  if (b == 0){
    motorsStraight(LOW,LOW);
    b++;
  }
  else if (b == 1){
    backUpTimer.end();
    hdes=180;
    motorsTurn(LOW,HIGH);
    turns=1;
    robot_state=TURN;
    b=0;
  }  

}