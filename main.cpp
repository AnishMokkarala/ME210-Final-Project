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
#define trigPin 16 //attach pin D3 Arduino to pin Trig of HC-SR04
#define START_SWITCH 7
#define RED_BLUE_SWTICH 8
#define TRIG_PIN 16
int echoPin = 11; // Pin 12 for 2nd Sensor attach pin D2 Arduino to pin Echo of HC-SR04
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
int spinDir1 = 0;
int spinDir2 = 1;
int spinEnable = 0;
int dist_th=15;//distance threshold
int rev=0;
int turns=0;
int hdes=0;
int head=0;
int b=0; //backup counter
AccelStepper myStepper1(1,PIN_STEP,PIN_DIR);//stepper object
int maxspeed=300;//maximum speed of stepper in steps/sec
elapsedMillis UltraSens;
int trig = false;
int duration;
int distance;
int left_ref;
int right_ref;
int black_line; // threshold for reading a black line

/*---------------Function Declarations----------------*/
void checkGlobalEvents(void);
void waitToStart(void);
void spinFlap(void);
void motorsStraight(int v1, int v2, int speed=255);
/* values of v1 & v2 determine the direction of rotation of the two motors respectively.
v1=HIGH & v2=HIGH, bot moves forward
v1=LOW & v2=LOW, bot moves backward
speed is read from the potentiometer on the bot*/
void motorsStill(void);
void motorsTurn(int v1,int v2,int feedback=100,int speed=128);
void backUp(void);
int obsDist1(void);//output from Ultrasonic Sensor at the top
int obsDist2(void);//output from Ultrasonic Sensor at the bottom
int line1(void);//output from Line Sensor 1
int line2(void);//output from Line Sensor 2
void vector_cross(float a[3], float b[3], float out[3]);
float vector_dot(float a[3], float b[3]);
void vector_normalize(float a[3]);
int get_heading(float acc[3], float mag[3], float p[3]);
void get_scaled_IMU(float Axyz[3], float Mxyz[3]);
int final_heading(void);
void alignWheels(void);

/*---------------IntervalTimer Declarations----------------*/
IntervalTimer backUpTimer;

/*---------------IMU Variables----------------*/
LSM9DS1 imu;
//Variables for smoothing
const int numReadings = 10;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average

// VERY IMPORTANT!
//These are the previously determined offsets and scale factors for accelerometer and magnetometer, using MPU9250_cal and Magneto
//The compass will NOT work well or at all if these are not correct
//Accel scale
float A_B[3]
 { -448.06, -105.63,  -72.05};

float A_Ainv[3][3]
{{  1.00203,  0.00021,  0.00468},
  {  0.00021,  1.00694, -0.00036},
  {  0.00468, -0.00036,  0.99763}};

//Mag scale
float M_B[3]
{ 5227.87, 3850.70, 2273.50};

float M_Ainv[3][3]
 {{  3.83475,  0.17160, -0.05174},
  {  0.17160,  3.01990,  0.03135},
  { -0.05174,  0.03135,  2.46193}};

// local magnetic declination in degrees
float declination = -13.6;

float p[] = {0, 1, 0};  //Y marking on sensor board points toward yaw = 0

/*---------------Setup Code----------------*/
void setup() {
  Serial.begin(9600);
  while (!Serial); //wait for connection
  Wire.begin();

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

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
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(START_SWITCH, INPUT);
  pinMode(RED_BLUE_SWTICH, INPUT);
  pinMode(LEFT_REFLECTOR, INPUT);
  pinMode(RIGHT_REFLECTOR, INPUT);
    
  // initialize interval timers here

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
        echoPin = 11;
        break;
    case MOVE:
        if(distance<dist_th && turns == 0){
          hdes=0;
          motorsTurn(HIGH, LOW);
          robot_state=TURN;         
          turns++;
          echoPin = 12;
        }
        else if(distance<dist_th && turns == 1){
          hdes=90;
          robot_state=TURN; 
          turns++;
          if (rev==0) motorsTurn(LOW,HIGH); 
          else if (rev==1) motorsTurn(HIGH,LOW);         
        }
        //Feedback control
        if (head>hdes){
          int err = head-hdes;
          if (rev==0)motorsStraight(HIGH,HIGH,err);
          else if (rev==1) motorsStraight(LOW,LOW,-1*err);
        }
        else if (head<hdes){
          int err = head-hdes;
          if (rev==0)motorsStraight(HIGH,HIGH,-1*err);
          else if (rev==1) motorsStraight(LOW,LOW,err);
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
    digitalWrite(trigPin,HIGH);
    trig = true; // update case
  }
  else if (UltraSens >=12 && trig == true){ // waits 10 milliseconds before echo
    digitalWrite(trigPin, LOW); // set trig to low
    trig = false;
    duration = pulseIn(echoPin, HIGH);
    distance = duration * 34 / 2000;
    UltraSens = 0;
  }

  if (digitalRead(LIM2)==LOW && robot_state!= SCORING){ //If bottom limit is unpressed and not scoring
          myStepper1.runSpeed();
  }

  head = final_heading();

}

void waitToStart(void){
  if (digitalRead(START_SWITCH)) {
    start_heading = final_heading() // get current imu reading
    robot_state = LOADING; // move to loading when switch is flipped
  }
  else{
    robot_state = WAITING; // redundant to ensure we stay in waiting until ready
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
    digitalWrite(SPIN_PIN_DIR1, 0); // confirm off
    digitalWrite(SPIN_PIN_DIR2, 0); // confirm off
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

// basic vector operations
void vector_cross(float a[3], float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

float vector_dot(float a[3], float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void vector_normalize(float a[3]) {
  float mag = sqrt(vector_dot(a, a));
  a[0] /= mag;
  a[1] /= mag;
  a[2] /= mag;
}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int get_heading(float acc[3], float mag[3], float p[3]) {
  float W[3], N[3]; //derived direction vectors

  // cross "Up" (acceleration vector, g) with magnetic vector (magnetic north + inclination) with  to produce "West"
  vector_cross(acc, mag, W);
  vector_normalize(W);

  // cross "West" with "Up" to produce "North" (parallel to the ground)
  vector_cross(W, acc, N);
  vector_normalize(N);

  // compute heading in horizontal plane, correct for local magnetic declination
  
  int heading = round(atan2(vector_dot(W, p), vector_dot(N, p)) * 180 / M_PI + declination);
  heading = -heading; //conventional nav, heading increases North to East
  heading = (heading + 720)%360; //apply compass wrap
  return heading;
}

// subtract offsets and correction matrix to accel and mag data
void get_scaled_IMU(float Axyz[3], float Mxyz[3]) {
  byte i;
  float temp[3];
    Axyz[0] = imu.ax;
    Axyz[1] = imu.ay;
    Axyz[2] = imu.az;
    Mxyz[0] = imu.mx;
    Mxyz[1] = imu.my;
    Mxyz[2] = imu.mz;
  //apply offsets (bias) and scale factors from Magneto
  for (i = 0; i < 3; i++) temp[i] = (Axyz[i] - A_B[i]);
  Axyz[0] = A_Ainv[0][0] * temp[0] + A_Ainv[0][1] * temp[1] + A_Ainv[0][2] * temp[2];
  Axyz[1] = A_Ainv[1][0] * temp[0] + A_Ainv[1][1] * temp[1] + A_Ainv[1][2] * temp[2];
  Axyz[2] = A_Ainv[2][0] * temp[0] + A_Ainv[2][1] * temp[1] + A_Ainv[2][2] * temp[2];
  vector_normalize(Axyz);

  //apply offsets (bias) and scale factors from Magneto
  for (int i = 0; i < 3; i++) temp[i] = (Mxyz[i] - M_B[i]);
  Mxyz[0] = M_Ainv[0][0] * temp[0] + M_Ainv[0][1] * temp[1] + M_Ainv[0][2] * temp[2];
  Mxyz[1] = M_Ainv[1][0] * temp[0] + M_Ainv[1][1] * temp[1] + M_Ainv[1][2] * temp[2];
  Mxyz[2] = M_Ainv[2][0] * temp[0] + M_Ainv[2][1] * temp[1] + M_Ainv[2][2] * temp[2];
  vector_normalize(Mxyz);
}

int final_heading(void){
  float Axyz[3], Mxyz[3];
  if ( imu.accelAvailable() ) imu.readAccel();
  if ( imu.magAvailable() )   imu.readMag();
  get_scaled_IMU(Axyz, Mxyz);    
  Axyz[0] = -Axyz[0];

  //Smoothing
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = get_heading(Axyz, Mxyz, p);
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;
  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }
  // calculate the average:
  average = total / numReadings;

  return average;
}
