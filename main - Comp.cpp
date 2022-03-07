#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

/*---------------Pin Numbers----------------*/
#define SPIN_PIN_DIR1 9
#define SPIN_PIN_DIR2 10
#define SPIN_PIN_EN 6
#define ENABLE1 A8//input to EN pin of LEFT MOTOR
#define ENABLE2 A9//input to EN pin of RIGHT MOTOR
#define IN1 0//input to IN1
#define IN2 1//input to IN2
#define IN3 2//input to IN3
#define IN4 5//input to IN4
#define PIN_STEP 3//pinStep for Stepper Motor
#define PIN_DIR 4 //pinDirection for Stepper Motor
#define EN_STEP 11//Enable for Stepper Motor
#define LIM_UP 13//top limit switch
#define LIM_LOW 12//bottom limit switch
#define START_SWITCH 7
#define RED_BLUE_SWTICH 8
#define ECHO_TOP 20
#define ECHO_BOT 21
#define TRIG_PIN 16 //16 Pin 12 for 2nd Sensor attach pin D2 Arduino to pin Echo of HC-SR04
#define LEFT_REFLECTOR A0
#define RIGHT_REFLECTOR A1

/*---------------State Definitions--------------------------*/
typedef enum {
    WAITING, LOADING, MOVE, TURN, ALIGN, SCORING, LOWERING, OFF
} Robot_states;

/*---------------Module Variables---------------------------*/
Robot_states robot_state;

int currentTime;
int team;
int spinDir1 = LOW;
int spinDir2 = HIGH;
int spinEnable = LOW;
int dist_th=1800;//distance threshold
int rev=0;
int turns=0;
float head=0;
int b=0; //backup counter
AccelStepper myStepper1(1,PIN_STEP,PIN_DIR);//stepper object
int maxspeed=300;//maximum speed of stepper in steps/sec
elapsedMillis gameTimer;
elapsedMillis lastPrint;
int trig = false;
int duration;
int distance;
int left_ref=400;//Left IR sensor threshold
int right_ref=400;//Right IR sensor threshold
//int black_line; // threshold for reading a black line
int sensor = 0;
int echoPin;
float Gz;
unsigned int dt = 30;
int raised = 0;


int i=0;
int j=0;
int k=0;

/*---------------Function Declarations----------------*/
void checkGlobalEvents(void);
void waitToStart(void);
void spinFlap(void);
void motorsStraight(int v1, int v2, int speed=240);
/* values of v1 & v2 determine the direction of rotation of the two motors respectively.
v1=HIGH & v2=HIGH, bot moves forward
v1=LOW & v2=LOW, bot moves backward
speed is read from the potentiometer on the bot*/
void motorsStill(void);
void motorsTurn(int v1,int v2,int feedback=0,int speed=240);
void backUp(void);
int obsDist1(void);//output from Ultrasonic Sensor at the top
int obsDist2(void);//output from Ultrasonic Sensor at the bottom
int line_right(void);//output from Line Sensor 1
int line_left(void);//output from Line Sensor 2
void alignWheels(void);
int readUltra(void);
void readGyro(void);
void jiggle(void);

/*---------------IntervalTimer Declarations----------------*/
IntervalTimer backUpTimer;

/*---------------IMU Variables----------------*/
LSM9DS1 imu;
float Gscale = (M_PI / 180.0) * 0.00875; //245 dps scale sensitivity = 8.75 mdps/LSB
int G_offset[3] = {-72, 132, 430};

/*---------------Setup Code----------------*/
void setup() {
  Serial.begin(9600);
  //while (!Serial); //wait for connection
  Wire.begin();

  // initialize all states here
  robot_state = WAITING;
    
  // initialize pin I/O here
  pinMode(SPIN_PIN_DIR1, OUTPUT);
  pinMode(SPIN_PIN_DIR2, OUTPUT);
  pinMode(SPIN_PIN_EN, OUTPUT);
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(LIM_UP,INPUT);
  pinMode(LIM_LOW,INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_BOT, INPUT);
  pinMode(ECHO_TOP, INPUT);
  pinMode(START_SWITCH, INPUT);
  pinMode(RED_BLUE_SWTICH, INPUT);
  pinMode(LEFT_REFLECTOR, INPUT);
  pinMode(RIGHT_REFLECTOR, INPUT);
  pinMode(EN_STEP,OUTPUT);
    
  // initialize interval timers here
  	if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    Serial.println(F("LSM9DS1 not detected"));
    while (1);
  }

  // other setup code here
  currentTime = millis();
  digitalWrite(EN_STEP,HIGH);

  myStepper1.setMaxSpeed(maxspeed);

}

/*---------------Main Loop----------------*/
void loop() { 
  checkGlobalEvents();
  //echoPin = ECHO_BOT;
  // Serial.println(digitalRead(RED_BLUE_SWTICH));
  // Serial.println(team);
  // delay(100);
  switch (robot_state) {
    case WAITING:   // update this!
        //currentTime = millis();
        if (digitalRead(START_SWITCH)){
          gameTimer=0;
          robot_state = LOADING; // move to loading
        }
        break;
    case LOADING:
        //wheel_state = STILL; // ensure wheels are turned off
        raised = 0;
        turns = 0;
        motorsStill();
        spinFlap();
        echoPin = ECHO_TOP;
        break;
    case MOVE:
        if (team==LOW && turns == 0){
          echoPin = ECHO_BOT;
          dist_th = 3200;
        }
        distance = readUltra();
        if(distance<dist_th && turns == 0){
          dist_th = 1800;
          motorsTurn(!team, team);
          robot_state=TURN;         
          turns++;
          echoPin = ECHO_BOT;
        }
        else if(distance<dist_th && turns == 1){
          robot_state=TURN; 
          turns++;
          if (rev==0) motorsTurn(team,!team); 
          else if (rev==1) motorsTurn(!team,team);         
        }
        //Feedback control
        // if (head>hdes){
        //   int err = head-hdes;
        //   if (rev==0)motorsStraight(HIGH,HIGH,err);
        //   else if (rev==1) motorsStraight(LOW,LOW,-1*err);
        // }
        // else if (head<hdes){
        //   int err = head-hdes;
        //   if (rev==0)motorsStraight(HIGH,HIGH,-1*err);
        //   else if (rev==1) motorsStraight(LOW,LOW,err);
        // }
        break;
    case TURN:
        readGyro();
        if(abs(head) > 85){
          if(turns==1){
            robot_state=MOVE;
            motorsStraight(LOW,LOW);
            head=0;
          }
          else if(turns==2){
            robot_state=ALIGN;
            if (rev==0)motorsStraight(LOW,LOW);
            else if (rev==1) motorsStraight(HIGH,HIGH);
            head=0;//check this
          }

        }
        break;
    case ALIGN:
        //alignWheels();
        if(line_right() < right_ref || line_left() < left_ref){
        //if (line1() == HIGH && line2() == HIGH){
        //NOTE sensors will not read high/low, they have 0-1023 value. BJ changed this "if" condition to the above line
          if(rev==0){
            motorsStill();
            digitalWrite(PIN_DIR,1);
            digitalWrite(EN_STEP,LOW);
            myStepper1.setSpeed(maxspeed);
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
        if (digitalRead(LIM_UP)==LOW && raised == 0){ 
          raised = 1;
          jiggle();
          delay(3000);
          digitalWrite(PIN_DIR,0);
          digitalWrite(EN_STEP,LOW);
          myStepper1.setSpeed(-1*maxspeed);
          myStepper1.runSpeed();
          robot_state = LOWERING;
        }
        else myStepper1.runSpeed();//keep running stepper motor if the Lim1 is not triggered        
        break;
    case LOWERING:
      myStepper1.runSpeed();
      if (digitalRead(LIM_LOW) == LOW && raised == 1){
        digitalWrite(EN_STEP,HIGH);
        //backUpTimer.begin(backUp,500000);
        motorsStraight(HIGH,HIGH);
        delay(500);
        motorsTurn(team,!team,30,225);
        turns=1;
        robot_state=TURN;
      }
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
  if (gameTimer>130000){
    motorsStill();
    robot_state = OFF;
  }

  // red vs blue team
  if(digitalRead(RED_BLUE_SWTICH)==LOW){
    team = LOW; //red;
    // SET OTHER VALUES HERE
  }
  else if (digitalRead(RED_BLUE_SWTICH)==HIGH){
    team = HIGH; //blue;
    // SET OTHER VALUES HERE
  }

  // if (digitalRead(LIM_LOW)==HIGH && raised == 1 && robot_state!= OFF){ //If bottom limit is unpressed and not scoring
  //   digitalWrite(EN_STEP,LOW);
  //   myStepper1.runSpeed();
  // }
  if (robot_state !=SCORING && robot_state !=LOWERING) digitalWrite(EN_STEP,HIGH);

}

void spinFlap(void){
  if (spinEnable == 0) {
    spinEnable = 1;
    currentTime = millis();
    digitalWrite(SPIN_PIN_DIR1, spinDir1);
    digitalWrite(SPIN_PIN_DIR2, spinDir2);
    digitalWrite(SPIN_PIN_EN, spinEnable); // begin spinning motor if not already spinning
  }

  if (millis() - currentTime > 10000){ // spin for three seconds
    spinEnable = 0; // turn motor off
    digitalWrite(SPIN_PIN_EN, spinEnable);
    digitalWrite(SPIN_PIN_DIR1, 0); // confirm off
    digitalWrite(SPIN_PIN_DIR2, 0); // confirm off
    robot_state = MOVE; // begin navigation to drop zone
    motorsStraight(LOW, LOW);
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

// void backUp(void){
//   if (b == 0){
//     motorsStraight(HIGH,HIGH);
//     b++;
//   }
//   else if (b == 1){
//     backUpTimer.end();
//     motorsTurn(!team,team);
//     turns=1;
//     robot_state=TURN;
//     b=0;
//   }  

// }

void alignWheels(void){
  // go forward when both wheels see white
  if(line_right() > right_ref && line_left() > left_ref && i==0) {
    if(rev==0)motorsStraight(LOW, LOW);
    if(rev==1)motorsStraight(HIGH,HIGH);
    i=1;
    j=0;
    k=0;
  }

  // turn left when left reads black
  if(line_right() > right_ref && line_left() < left_ref&&j==0) {
    if(rev==0)motorsTurn(HIGH, LOW,-150,150);
    if(rev==1)motorsTurn(LOW,HIGH,-150,150);
    j=1;
    i=0;
    k=0;
  }

  // turn right when right reads black
  if(line_right() < right_ref && line_left() > left_ref&&k==0) {
    if(rev==0)motorsTurn(LOW, HIGH,75,75);
    if(rev==1)motorsTurn(HIGH,LOW,75,75);
    k=1;
    i=0;
    j=0;
  }

  // stop when both wheels see black
  if(line_right() < right_ref && line_left() < left_ref) {
    motorsStill();
    i=0;
    j=0;
    k=0;
  }
}

int readUltra(void){
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(echoPin, HIGH);
  delay(10);
  return duration;
}

void readGyro(void){
  if ( imu.gyroAvailable() ) imu.readGyro();

  Gz = Gscale * (float(imu.gz) - G_offset[2]);

  if (lastPrint>dt){
    if (Gz > 0.02 || Gz < -0.02) {
      head = head+ 61.2*Gz*dt/1000;
    }
    lastPrint=0;
  }

}

int line_right(){
  return analogRead(RIGHT_REFLECTOR);
}

int line_left(){
  return analogRead(LEFT_REFLECTOR);
}

void jiggle(void){
  delay(200);
  motorsStraight(LOW,LOW,255);
  delay(200);
  motorsStill();
}