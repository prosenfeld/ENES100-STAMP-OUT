#include <Enes100.h>
#include <math.h>

const int RIGHT_MOTOR_ENABLE = 5;   // D5 (PWM)
const int LEFT_MOTOR_ENABLE  = 6;   // D6 (PWM)

const int RIGHT_MOTOR_IN1 = 13;     // D11
const int RIGHT_MOTOR_IN2 = 11;      // D3

const int LEFT_MOTOR_IN1  = 18;     // D18
const int LEFT_MOTOR_IN2  = 19;     // D19

const int SERVO_PWM = 3;



void setup() {
    
    float distanceToObstacle = 0.3;
    boolean startedTop = false; 


    // setup
    // Right motor pins
    pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);

    // Left motor pins
    pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);

    // driveForward();
    // don't drive
    turnRight();
    // servo testing:
    // setServo(127);

    

    // Enes100.begin("STAMP Out!", FIRE, 3, 1116, 8, 9);
    // Tank.begin();



    
    // // handle initial position -> mission site 
    // if(Enes100.getY() < 1){
        
    //     // This will need tuning. Ultrasonic probably should be used.
        
    //      driveToPoint(0.55,1.55,1.571);
    // } else {
        
    //     driveToPoint(0.55,0.55,-1.571);
    // }
    
    // turnAngle(0);
    // // do mission things here
    // delay(1000);
    
    
    // // drive forward until we find an obstacle
    // while((Tank.readDistanceSensor(1) > distanceToObstacle || Tank.readDistanceSensor(1) == -1) && Enes100.getY() < 2.7 ){
    //     driveForward();
    //     delay(1);
    // }
    // stopDriving();
    // // edge case of just the rumble
    // if(Enes100.getX() > 3.6){
   
    //     return;
    // } else if( Enes100.getX() > 2.7){
    //     limbo();
    //     return;
    // }
    
    // // turn  to try to find an opening
    // if(Enes100.getY() < 1){
    //     // turnAngle(-1.571);
    //     driveToPoint(Enes100.getX(),Enes100.getY() + 0.5,0);
        
    //       // if we are open, move forward. if not, do that again.
            
    //         if(Tank.readDistanceSensor(1) > 0.5){
                
    //             driveToPoint(3,Enes100.getY(),0);
        
    //         } else{
    //             // move down another row
    //             driveToPoint(Enes100.getX(),Enes100.getY() + 0.5,0);
                
    //         }
    
    // } else{
    //     // turnAngle(1.571)
    //     driveToPoint(Enes100.getX(),Enes100.getY() - 0.5,0);
    //     Enes100.println(Tank.readDistanceSensor(1));
        
    //        if(Tank.readDistanceSensor(1) > 0.5 || Tank.readDistanceSensor(1) == -1){
                
    //             driveToPoint(3,Enes100.getY(),0);
        
    //         } else{
    //             // move down another row
    //             driveToPoint(Enes100.getX(),Enes100.getY() - 0.5,0);
    //             // go forward
    //             driveToPoint(3,Enes100.getY(),0);

                
                
                
    //         }

    // }
    
    // limbo();
    
 
    
    
}

void setServo(int pwm){
    analogWrite(SERVO_PWM,pwm);
}

void limbo(){
          // drive to line with limbo
        driveToPoint(3,1.5,0);
        // go through limbo
        driveToPoint(3.7,1.5,0);
        // Done!
        Enes100.println("done!");
}

void driveToPoint(float x, float y, float theta){
    // Calculate the angle to turn to
  float deltaX = x - Enes100.getX();
  float deltaY = y - Enes100.getY();
  float deltaTheta = atan2(deltaY,deltaX);
  float thetaError = 0.1;
  
  
  // turn to that angle
  turnAngle(deltaTheta);
  
  // drive until we are close enough to that point.
  driveForward();
  while(dist(Enes100.getX(),x,Enes100.getY(),y) > 0.1){
      // adjust heading if needed, if we veer to a side.
      if(abs(Enes100.getTheta() - deltaTheta) > thetaError){
          deltaX = x - Enes100.getX();
          deltaY = y - Enes100.getY();
          deltaTheta = atan2(deltaY,deltaX);
          turnAngle(deltaTheta);
      }
      
      delay(1);
  }
  
  stopDriving();
  
  // Get the heading to the requested one
  turnAngle(theta);
  
    
}

float dist(float x0, float x1, float y0, float y1){
  return sqrt(pow(x0 - x1, 2) + pow(y0 - y1, 2));
}

void turnAngle(float angle){

  Enes100.print("Turning to angle");
  Enes100.print(angle);
  
  
  // Angle tolerance. Will need tuning.
    float delta = 0.1;
    
   // if we are within error, don't move
   if(abs(Enes100.getTheta() - angle) < delta){
       return;
   }

    
    
    // Pick which way we are turning.
    if(Enes100.getTheta() - angle < 0){
    turnLeft();
    } else{
        turnRight();
    }
    // Turn until we are within tolerance.
    while(abs(Enes100.getTheta() - angle) > delta){
        delay(1);

    }
    
    Enes100.println("done!");
     Enes100.println(abs(Enes100.getTheta() - angle));
    stopDriving();
    
}

void setLeftMotorPWM(int leftMotor){
    // Backwards
    if (leftMotor < 0){

        digitalWrite(LEFT_MOTOR_IN1, HIGH);
        digitalWrite(LEFT_MOTOR_IN2, LOW);
        analogWrite(LEFT_MOTOR_ENABLE, abs(leftMotor));
    }
    // Forwards
    else{

        digitalWrite(LEFT_MOTOR_IN1, LOW);
        digitalWrite(LEFT_MOTOR_IN2, HIGH);
        analogWrite(LEFT_MOTOR_ENABLE, abs(leftMotor));
    }

}

void setRightMotorPWM(int rightMotor){
    // Backwards
    if (rightMotor < 0){

        digitalWrite(RIGHT_MOTOR_IN1, HIGH);
        digitalWrite(RIGHT_MOTOR_IN2, LOW);
        analogWrite(RIGHT_MOTOR_ENABLE, abs(rightMotor));
    }
    // Forwards
    else{

        digitalWrite(RIGHT_MOTOR_IN1, LOW);
        digitalWrite(RIGHT_MOTOR_IN2, HIGH);
        analogWrite(RIGHT_MOTOR_ENABLE, abs(rightMotor));
    }

}

void setDriving(int leftMotor, int rightMotor){
        setLeftMotorPWM(leftMotor);
        setRightMotorPWM(rightMotor);
}

void stopDriving(){
        setLeftMotorPWM(0);
        setRightMotorPWM(0);
}

void driveForward(){
        setLeftMotorPWM(255);
        setRightMotorPWM(255);
}


void driveBackward(){
        setLeftMotorPWM(-127);
        setRightMotorPWM(-127);
}
// turns are slow so we can be accurate. this will need real life adjusting.
void turnLeft(){
        setLeftMotorPWM(-33);
        setRightMotorPWM(33);
}

void turnRight(){
        setLeftMotorPWM(255);
        setRightMotorPWM(-255);
}


void loop() {
        // setServo(0);
        // delay(1000);
        // setServo(127);
        // delay(1000);
}