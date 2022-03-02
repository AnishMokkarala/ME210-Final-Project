#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>

#define ENABLE1 A8//input to EN pin of RIGHT MOTOR
#define ENABLE2 A9//input to EN pin of LEFT MOTOR
#define IN1 0//input to IN1
#define IN2 1//input to IN2
#define IN3 2//input to IN3
#define IN4 5//input to IN4

LSM9DS1 imu;

//Variables for smoothing
const int numReadings = 500;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
elapsedMillis moveTime;
elapsedMillis readTime;
elapsedMillis collect;
elapsedMillis lastPrint;



//Accel scale 16457.0 to normalize
float A_B[3]
 { -275.01, -145.96,  -70.21};

float A_Ainv[3][3]
 {{  0.99256, -0.00172, -0.00608},
  { -0.00172,  0.99285, -0.00552},
  { -0.00608, -0.00552,  0.99686}};

//Mag scale 3746.0 to normalize
float M_B[3]
{ 3654.70,-1224.89,-1901.08};

float M_Ainv[3][3]
{{  3.33911, -0.00764,  0.42076},
  { -0.00764,  3.39214, -0.00599},
  {  0.42076, -0.00599,  4.61207}};

// local magnetic declination in degrees
float declination = -13.6;

float p[] = {0, 1, 0};  //Y marking on sensor board points toward yaw = 0


int currentTime=0;
int val=HIGH;
float head;
int hdes = 90;
int rev = 2;
int head_off = 0;
int inst = 0;
unsigned int time = 100;
int move = true;
int k = 10;
int off;
float Gz;
float dt = 30;

float Gscale = (M_PI / 180.0) * 0.00875; //245 dps scale sensitivity = 8.75 mdps/LSB
int G_offset[3] = {-72, 132, -600};

void motorsStraight(int v1, int v2, int speed=255);
void motorsStill(void);
void motorsTurn(int v1,int v2,int feedback=0,int speed=180);
void vector_cross(float a[3], float b[3], float out[3]);
float vector_dot(float a[3], float b[3]);
void vector_normalize(float a[3]);
int get_heading(float acc[3], float mag[3], float p[3]);
void get_scaled_IMU(float Axyz[3], float Mxyz[3]);
int final_heading(void);


void setup() {
  Serial.begin(9600);
  //while (!Serial); //wait for connection

  // put your setup code here, to run once:
  pinMode(ENABLE1, OUTPUT);
  pinMode(ENABLE2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(13,OUTPUT);
  motorsStill();

  Wire.begin();

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }
  currentTime=millis();

 	if (imu.begin() == false) // with no arguments, this uses default addresses (AG:0x6B, M:0x1E) and i2c port (Wire).
  {
    //Serial.println(F("LSM9DS1 not detected"));
    while (1);
  }
  
  
  // while(collect<1000){
  //   head = final_heading();
  // }
  //head_off = 180-final_heading();
  collect = 0;
  head=0.0;

  motorsTurn(HIGH,LOW);
}

void loop() {
  if ( imu.gyroAvailable() ) imu.readGyro();

  Gz = Gscale * (float(imu.gz) - G_offset[2]);

  if (lastPrint>dt){
    if (Gz > 0.02 || Gz < -0.02) {
      head = head+ 61.2*Gz*dt/1000;
    }
    lastPrint=0;
  }
  Serial.println(head);

  if (abs(head) > 90){
    motorsStill();
  }

  // if (Gz>0){
  //   Serial.println(imu.gz);
  // }

  //put your main code here, to run repeatedly:
  // while(collect<3000){
  //   head = final_heading();
  //   off = head;
  // }
  // head_off = 180-off;

  // if (lastPrint>200){
  //   Serial.println(head);
  //   lastPrint = 0;
  // }
  // head = final_heading();// - head_off;
  // //Serial.println(head_off);
  // if (readTime>400 && move == false){
  //   time = k*abs(head-hdes);
  //   //Serial.println(time);
  //   if (head>hdes+3){
  //     //Serial.println(1);
  //     motorsTurn(HIGH,HIGH);
  //   }
  //   else if (head<hdes-3){
  //     //Serial.println(2);

  //     motorsTurn(LOW,LOW);
  //   }  
  //   moveTime=0;
  //   move = true;
  // }

  // if (moveTime>time && move == true){
  //   motorsStill();
  //   //Serial.println(time);

  //   readTime=0;
  //   moveTime=0;
  //   move = false;
  // }
  
  // if(head-hdes<3 && head-hdes>-3 ){
  //   //rev=2;
  //   //inst = 0;
  //   digitalWrite(13,HIGH);
  // }
  // else{
  //   digitalWrite(13,LOW);
  // }
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
  heading = (heading + 720 + head_off)%360; //apply compass wrap
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