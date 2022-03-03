#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

/*---------------Pin Numbers----------------*/
#define SPIN_PIN_DIR1 9
#define SPIN_PIN_DIR2 10
#define SPIN_PIN_EN 6
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
#define START_SWITCH 7
#define RED_BLUE_SWTICH 8
#define ECHO_TOP 20//11
#define ECHO_BOT 21
#define TRIG_PIN 16 //16 Pin 12 for 2nd Sensor attach pin D2 Arduino to pin Echo of HC-SR04
#define LEFT_REFLECTOR A0
#define RIGHT_REFLECTOR A1

/*---------------State Definitions--------------------------*/
typedef enum {
    WAITING, LOADING, MOVE, TURN, ALIGN, SCORING
} Robot_states;

/*---------------Module Variables---------------------------*/
Robot_states robot_state;

int currentTime;
int team; // bool for red (1) or blue (0)
int spinDir1 = LOW;
int spinDir2 = HIGH;
int spinEnable = HIGH;
int dist_th=1600;//distance threshold
int rev=0;
int turns=0;
int hdes=0;
float head=0;
int b=0; //backup counter
AccelStepper myStepper1(1,PIN_STEP,PIN_DIR);//stepper object
int maxspeed=300;//maximum speed of stepper in steps/sec
elapsedMicros UltraSens;
elapsedMillis lastPrint;
int trig = false;
int duration;
int distance;
int left_ref;
int right_ref;
int black_line; // threshold for reading a black line
int sensor = 0;
int echoPin;
float Gz;
unsigned int dt = 30;

/*---------------Function Declarations----------------*/
void checkGlobalEvents(void);
void waitToStart(void);
void spinFlap(void);
void motorsStraight(int v1, int v2, int speed=135);
/* values of v1 & v2 determine the direction of rotation of the two motors respectively.
v1=HIGH & v2=HIGH, bot moves forward
v1=LOW & v2=LOW, bot moves backward
speed is read from the potentiometer on the bot*/
void motorsStill(void);
void motorsTurn(int v1,int v2,int feedback=0,int speed=180);
void backUp(void);
int obsDist1(void);//output from Ultrasonic Sensor at the top
int obsDist2(void);//output from Ultrasonic Sensor at the bottom
int line1(void);//output from Line Sensor 1
int line2(void);//output from Line Sensor 2
void alignWheels(void);
int readUltra(void);
void readGyro(void);

/*---------------IntervalTimer Declarations----------------*/
IntervalTimer backUpTimer;

/*---------------IMU Variables----------------*/
LSM9DS1 imu;
float Gscale = (M_PI / 180.0) * 0.00875; //245 dps scale sensitivity = 8.75 mdps/LSB
int G_offset[3] = {-72, 132, 430};

/*---------------Setup Code----------------*/
void setup() {
  //Serial.begin(9600);
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
  pinMode(LIM1,INPUT);
  pinMode(LIM2,INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_BOT, INPUT);
  pinMode(ECHO_TOP, INPUT);
  pinMode(START_SWITCH, INPUT);
  pinMode(RED_BLUE_SWTICH, INPUT);
  pinMode(LEFT_REFLECTOR, INPUT);
  pinMode(RIGHT_REFLECTOR, INPUT);
    
  // initialize interval timers here
  	if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    //Serial.println(F("LSM9DS1 not detected"));
    while (1);
  }

  // other setup code here
  currentTime = millis();
  digitalWrite(SPIN_PIN_DIR1, spinDir1);
  digitalWrite(SPIN_PIN_DIR2, spinDir2);
  digitalWrite(SPIN_PIN_EN, spinEnable);
  myStepper1.setMaxSpeed(maxspeed);
  myStepper1.setSpeed(maxspeed);
  black_line = 600;

}

/*---------------Main Loop----------------*/
void loop() { 
  // if (UltraSens >=2 && trig == false){ // waits 2 milliseconds
  //   digitalWrite(TRIG_PIN,HIGH);
  //   trig = true; // update case
  // }
  // else if (UltraSens >=12 && trig == true){ // waits 10 milliseconds before echo
  //   digitalWrite(TRIG_PIN, LOW); // set trig to low
  //   if (sensor == 1){
  //     duration = pulseIn(ECHO_TOP, HIGH);
  //     sensor = 0;
  //   }
  //   else if (sensor == 0){
  //     //duration = pulseIn(ECHO_BOT, HIGH);
  //     sensor = 1;
  //   }
  //   trig = false;
  //   distance = duration * 34 / 2000;
  //   UltraSens = 0;
  // }

  // checkGlobalEvents();
    
  switch (robot_state) {
    case WAITING:   // update this!
        currentTime = millis();
        robot_state = LOADING; // move to loading
        break;
    case LOADING:
        //wheel_state = STILL; // ensure wheels are turned off
        motorsStill();
        spinFlap();
        echoPin = ECHO_TOP;
        break;
    case MOVE:
        distance = readUltra();
        if(distance<dist_th && turns == 0){
          motorsTurn(LOW, HIGH);
          robot_state=TURN;         
          turns++;
          echoPin = ECHO_BOT;
        }
        else if(distance<dist_th && turns == 1){
          robot_state=TURN; 
          turns++;
          if (rev==0) motorsTurn(HIGH,LOW); 
          else if (rev==1) motorsTurn(HIGH,LOW);         
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
        if(abs(head) > 90){
          if(turns==1){
            robot_state=MOVE;
            motorsStraight(LOW,LOW);
            head=0;
          }
          else if(turns==2){
            motorsStill();
            // robot_state=ALIGN;
            // if (rev==0)motorsStraight(LOW,LOW);
            // else if (rev==1) motorsStraight(HIGH,HIGH);
          }

        }
        break;
    case ALIGN:
        alignWheels();
        if(right_ref < black_line && left_ref < black_line){
        //if (line1() == HIGH && line2() == HIGH){
        //NOTE sensors will not read high/low, they have 0-1023 value. BJ changed this "if" condition to the above line
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
          hdes = 180; //will be an if depending on team          
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
  if(digitalRead(RED_BLUE_SWTICH)){
    team = 1; // red;
    hdes = 0;
    // SET OTHER VALUES HERE
  }
  else{
    team = 0; //blue;
    hdes = 180;
    // SET OTHER VALUES HERE
  }

  // read potentiometer input for motor speed determination
  if (UltraSens >=2 && trig == false){ // waits 2 milliseconds
    digitalWrite(TRIG_PIN,HIGH);
    trig = true; // update case
  }
  else if (UltraSens >=12 && trig == true){ // waits 10 milliseconds before echo
    digitalWrite(TRIG_PIN, LOW); // set trig to low
    trig = false;
    duration = pulseIn(ECHO_TOP, HIGH);
    distance = duration * 34 / 2000;
    UltraSens = 0;
  }

  if (digitalRead(LIM2)==LOW && robot_state!= SCORING){ //If bottom limit is unpressed and not scoring
          myStepper1.runSpeed();
  }

}

// void waitToStart(void){
//   if (digitalRead(START_SWITCH)) {
//     start_heading = final_heading(); // get current imu reading
//     robot_state = LOADING; // move to loading when switch is flipped
//   }
//   else{
//     robot_state = WAITING; // redundant to ensure we stay in waiting until ready
//   }
// }

void spinFlap(void){
  if (spinEnable == 0) {
    spinEnable = 1;
    currentTime = millis();
    digitalWrite(SPIN_PIN_EN, spinEnable); // begin spinning motor if not already spinning
  }

  if (millis() - currentTime > 3000){ // spin for three seconds
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

void alignWheels(void){
  // go forward when both wheels see white
  if(right_ref > black_line && left_ref > black_line) {
    motorsStraight(HIGH, HIGH);
  }

  // turn left when left reads black
  if(right_ref > black_line && left_ref < black_line) {
    motorsTurn(LOW, HIGH);
  }

  // turn right when right reads black
  if(right_ref < black_line && left_ref > black_line) {
    motorsTurn(HIGH, LOW);
  }

  // stop when both wheels see black
  if(right_ref < black_line && left_ref < black_line) {
    motorsStill();
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