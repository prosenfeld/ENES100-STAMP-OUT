#include <Enes100.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

Adafruit_AMG88xx amg;
HUSKYLENS huskylens;
//HUSKYLENS green line >> SDA; blue line >> SCL

//VARIABLE DECLARATIONS FOR IR SENSOR
float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float heatmap[8][8];
// this gets updated dynamically for ambient; but putting a theoretical value to start.
float threshold = 40;

//VARIABLE DECLARATIONS FOR MOTOR PINS
const int RIGHT_MOTOR_ENABLE = 5;   // D5 (PWM)
const int LEFT_MOTOR_ENABLE  = 6;   // D6 (PWM)

const int RIGHT_MOTOR_IN1 = 13;     // D11
const int RIGHT_MOTOR_IN2 = 11;      // D3

const int LEFT_MOTOR_IN1  = 18;     // D18
const int LEFT_MOTOR_IN2  = 19;     // D19

// Define the Trig and Echo pin connections for the ultrasonic sensor
const int trigPin = 15;
const int echoPin = 14;



const int SERVO_PWM = 3;

//this can vary depending on what angle the arm gets attached; double check values are accurate before running full code
const int ARM_UP = 150;
const int ARM_DOWN = 100;
const int TURN_SPEED = 110;



/*-----------------START OF MAIN CODE--------------------*/

void setup() {
  // setup ENES100 library
  Enes100.begin("STAMP Out!", FIRE, 24, 1116, 4, 2);

  //setup HuskyLens
  Wire.begin();
  while (!huskylens.begin(Wire))
  {
    Serial.println(F("Begin failed!"));
      Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
      Serial.println(F("2.Please recheck the connection."));
      delay(100);
  }
  
  // initialize the IR sensor
  Serial.begin(9600);
    Serial.println(F("AMG88xx pixels"));

    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    
    Serial.println("-- Pixels Test --");

    Serial.println();

    delay(100); // let sensor boot up
  

  float distanceToObstacle = 0.3;
  boolean startedTop = false;

  // setup motor pins

  // Right motor pins
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);

  // Left motor pins
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);

  // arm servo
  pinMode(SERVO_PWM, OUTPUT);

  // Ultrasonic Sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // arm up
  setServo(ARM_UP);

  /*-----------------------END OF SETUP-------------------------*/

  // INITIAL POSITION -> MISSION SITE
  
  if(Enes100.getY() < 1 && Enes100.getY() > 0){
    // This will need tuning. Ultrasonic probably should be used.
    turnAngle(1.571);
    while (Enes100.getY() < 1.31) {
      driveForward();
      delay(1);
    }
    
    driveToPoint(0.27,1.31,1.571);

  } else if (Enes100.getY() > 0){
    turnAngle(-1.571);
      while (Enes100.getY() > 0.69) {
        driveForward();
        delay(1);
      }
    stopDriving();
    driveToPoint(0.27,0.69,-1.571);
  }

  //turnAngle(0);
  //do mission things here
  topDirection();
  //drive forward here
  Enes100.mission(NUM_CANDLES, numFlames());
  //additional mission things

  //MISSION SITE -> END OF ARENA
  driveToPoint(0.75, 0, 0);
  driveToPoint(2.61, 0, 0);

  //do limbo
  limbo();

}

/*------------------END OF MAIN CODE---------------------*/


// loop function
void loop() {

  Enes100.println("X");
  Enes100.println(Enes100.getX());
  Enes100.println("Y");
  Enes100.println(Enes100.getY());
  Enes100.println("Theta:");
  Enes100.println(Enes100.getTheta());

  delay(1000);


}

// Servo for the arm
void setServo(int pwm) {
  analogWrite(SERVO_PWM, pwm);
}

// do the limbo LMBO
void limbo() {
  // drive to line with limbo, lower arm
  driveToPoint(3, 1.5, 0);
  setServo(ARM_DOWN);
  // go through limbo, raise arm
  while (Enes100.getX() < 3.7) {
    driveForward();
    delay(10);
  }
  stopDriving();
  setServo(ARM_UP);
  // Done!
  Enes100.println("done!");
}


// function to turn to a specific given angle TANG
void turnAngle(float angle) {
  float diff = angleDiff(angle, Enes100.getTheta());

  Enes100.print("Turning to angle");
  Enes100.print(angle);

  // Angle tolerance. Will need tuning.
  float delta = 0.1;

  // if we are within error, don't move
  if (abs(Enes100.getTheta() - angle) < delta) {
    return;
  }

  // Pick which way we are turning.
  if (diff < 0) {
    turnLeft();
  } else {
    turnRight();
  }
  // Turn until we are within tolerance.
  while (abs(Enes100.getTheta() - angle) > delta) {
    delay(1);

  }

  Enes100.println("done!");
  Enes100.println(abs(Enes100.getTheta() - angle));
  stopDriving();

}

float angleDiff(float a, float b){
  float  diff = fmod(a-b + M_PI, 2*M_PI);
  if (diff < 0) 
    diff += 2*M_PI;
  return diff - M_PI;
}

// helper function for distance between points
float dist(float x0, float x1, float y0, float y1) {
  return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2));
}

// drive to a specific given point DTP
void driveToPoint(float x, float y, float theta) {
  // Calculate the angle to turn to
  float deltaX = x - Enes100.getX();
  float deltaY = y - Enes100.getY();
  float deltaTheta = atan2(deltaY, deltaX);
  float thetaError = 0.25;


  // turn to that angle
  turnAngle(deltaTheta);

  // drive until we are close enough to that point.
  driveForward();
  while (dist(Enes100.getX(), x, Enes100.getY(), y) > 0.1) {
    // adjust heading if needed, if we veer to a side.
    
    if (abs(Enes100.getTheta() - deltaTheta) > thetaError) {
      deltaX = x - Enes100.getX();
      deltaY = y - Enes100.getY();
      deltaTheta = atan2(deltaY, deltaX);
      turnAngle(deltaTheta);
      driveForward();
    }
     delay(100);
    
  }

  stopDriving();

  // Get the heading to the requested one
  turnAngle(theta);


}


// Driving functions

void setLeftMotorPWM(int leftMotor) {
  // Backwards
  if (leftMotor < 0) {
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    analogWrite(LEFT_MOTOR_ENABLE, abs(leftMotor));
  }
  // Forwards
  else {
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    analogWrite(LEFT_MOTOR_ENABLE, abs(leftMotor));
  }

}

void setRightMotorPWM(int rightMotor) {
  // Backwards
  if (rightMotor < 0) {

    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
    analogWrite(RIGHT_MOTOR_ENABLE, abs(rightMotor));
  }
  // Forwards
  else {

    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    analogWrite(RIGHT_MOTOR_ENABLE, abs(rightMotor));
  }

}

void setDriving(int leftMotor, int rightMotor) {
  setLeftMotorPWM(leftMotor);
  setRightMotorPWM(rightMotor);
}

void stopDriving() {
  setLeftMotorPWM(0);
  setRightMotorPWM(0);
}

void driveForward() {
  setLeftMotorPWM(255);
  setRightMotorPWM(255);
}

void driveBackward() {
  setLeftMotorPWM(-255);
  setRightMotorPWM(-255);
}

// tune turning speed based on how the vision system behaves.
void turnLeft() {
  setLeftMotorPWM(-TURN_SPEED);
  setRightMotorPWM(TURN_SPEED);
}

void turnRight() {
  setLeftMotorPWM(TURN_SPEED);
  setRightMotorPWM(-TURN_SPEED);
}

// FUNCTIONS FOR IR SENSOR

// Count how many corners are above our threshold
int countCorners() {
  int count = 0;

  // top-left region A4:C6  → rows 2–4, cols 0–2
  if (regionHasValueAbove(heatmap, 2, 4, 0, 2, threshold)) count++;

  // top-right region F4:H6 → rows 2–4, cols 5–7
  if (regionHasValueAbove(heatmap, 2, 4, 5, 7, threshold)) count++;

  // bottom-left region A8:C9 → rows 6–7, cols 0–2
  if (regionHasValueAbove(heatmap, 6, 7, 0, 2, threshold)) count++;

  // bottom-right region F8:H9 → rows 6–7, cols 5–7
  if (regionHasValueAbove(heatmap, 6, 7, 5, 7, threshold)) count++;

  return count;
}

// average the top two rows and add 8 to adjust the threshold for ambient.
float computeThreshold(float arr[8][8]) {
  float sum = 0;
  int count = 0;

  for (int r = 0; r < 2; r++) {        // first two rows: 0 and 1
    for (int c = 0; c < 8; c++) {
      sum += arr[r][c];
      count++;
    }
  }

  return sum / count + 8;
}

// check for values above the threshold
bool regionHasValueAbove(float arr[8][8], int r1, int r2, int c1, int c2, float th) {
  for (int r = r1; r <= r2; r++) {
    for (int c = c1; c <= c2; c++) {
      if (arr[r][c] > th) return true;
    }
  }
  return false;
}

// array of length 64 -> 8x8 matrix 
void arrayToMatrix8x8(float inArray[64], float outMatrix[8][8]) {
  int index = 0;
  for (int r = 0; r < 8; r++) {
    for (int c = 0; c < 8; c++) {
      outMatrix[r][c] = inArray[index++];
    }
  }
}

int numFlames(){
  amg.readPixels(pixels);
  // convert that data to a matrix
  arrayToMatrix8x8(pixels,heatmap);
  // dynamically adjust our threshold based on ambient based on the top of the frame
  threshold = computeThreshold(heatmap);
  return countCorners();
}

void topDirection(){
  if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    else
    {
      Serial.println(F("###########"));
        
      while (huskylens.available())
      {
        delay(1000);
        HUSKYLENSResult result = huskylens.read();
        printTopResult(result);
      }    
    }
}

void printTopResult(HUSKYLENSResult result){
  Serial.println(F("Topography direction: "));
  if (result.ID == 1){
    Serial.println(F("A"));
    Enes100.mission(TOPOGRAPHY, TOP_A);
  } else if (result.ID == 2){
    Serial.println(F("B"));
    Enes100.mission(TOPOGRAPHY, TOP_B);
  } else if (result.ID == 3){
    Serial.println(F("C"));
    Enes100.mission(TOPOGRAPHY, TOP_C);
  } else{
    Serial.println("Unknown!");
  }
}

int getUltrasonicDistance(){

  // Define variables to store duration and distance
  long duration;
  int distance;

    // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Sets the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  return distance;

}
