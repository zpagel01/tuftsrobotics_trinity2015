#include <Servo.h>

#include "motor.h"
#include "motorcontrol.h"
#include "fireSensorArray.h"

//DEFINE ALL PINS HERE
  #define lineSensePin           A5
  #define distRightBackPin      A13
  #define distRightFrontPin     A15
  #define distFrontPin          A14
  #define fireSensePin1         A1
  #define fireSensePin2         A3
  #define fireSensePin3         A0
  #define fireSensePin4         A4
  #define fireSensePin5         A2
  #define startButton           8
  #define servoPin              9

  //Motor pins
  #define leftMotordig           4
  #define leftMotorpwm           5
  #define rightMotordig          2
  #define rightMotorpwm          3
  

//IS SERIAL COMM NEEDED?
//THIS FUCKS UP TIMING SOM'M BAD
#define DEBUG                 0

//Possible States
#define STARTPUSHED           8
#define INITIALIZATION        0
#define WALLFOLLOW            1
#define INROOM                2
#define FOUNDFIRE             3
#define RETURNHOME            4
#define PUTOUTFIRE            6
#define ALIGNFIRE             7
#define HOMEREACHED           9

#define INITIALIZATION_TIME   5500

//Wall-follow constants
#define FRONTOBSTACLEDIST     300

//Line sense & alignment constants
#define LINESENSING_INVERTED  0     //1 = look for black, 0 = look for white

#if LINESENSING_INVERTED
  #define LINESENSED            850
#else
  #define LINESENSED            45
#endif

#define LINE_INFRONT          0
#define LINE_BEHIND           1
#define ON_LINE               2
#define LINE_ADJ_SPEED        80

//Fire sensing constants
#define FIRESENSED            34
#define FIRECLOSE             480
#define FIREANGLETHRESH       4
#define FIRESWEEPTIME         200
#define FIRESENSE_TRIALS      10

//Motor controller object
//(motorcontrol.h)
MotorControl mcontrol(DERIVATIVE);

//Fire sensor object
//(fireSensorArray.h)
FireSensorArray fireSense;

//EXTINGUISHER SERVO CONSTANTS
#define SERVO_LOOSE           40
#define SERVO_TAUT            90


//Motor objects
Motor leftMotor;
Motor rightMotor;

//Servo for canister
Servo pullServo;

//Start state is STARTPUSHED
int STATE = STARTPUSHED;

void setup() {
  //Set up motors with proper pins
  leftMotor.attach(leftMotordig,leftMotorpwm);
  rightMotor.attach(rightMotordig,rightMotorpwm);
  
  pullServo.attach(10);
  pullServo.write(SERVO_LOOSE);
  
  //Flip both motors
  leftMotor.flip();
  rightMotor.flip();
  
  //Give these motors to the motor controller for wall following
  mcontrol.attach(&leftMotor,&rightMotor);
  
  //Initialize array of fire sense pins
  //and pass it to the fire sensor array object
  int fireSensePins[5] = {fireSensePin1,fireSensePin2,fireSensePin3,fireSensePin4,fireSensePin5};
  fireSense.attach(fireSensePins);
  
  
  #if DEBUG
    Serial.begin(9600);
  #endif
  
  pinMode(lineSensePin,INPUT);
  pinMode(distRightBackPin,INPUT);
  pinMode(distRightFrontPin,INPUT);
  pinMode(distFrontPin,INPUT);
  pinMode(startButton,INPUT);
  
  //Wait for serial to begin
  #if DEBUG
    while(!Serial);
    Serial.println("Comm ready");
  #endif
  
  //Wait for start button
  while(false && digitalRead(startButton)==LOW){
    if(millis()%1000==0){
      sensorDiagnostics();
    }
  }
  #if DEBUG
    Serial.println("Started");
  #endif
}

//These declarations are for line adjustment

boolean lineSensed;
int lineVal;
//END DECLARATIONS FOR LINE ADJUSTMENT

//These declarations are for fire sensing
int fire_motinertia = 80;//140;
int fire_motcorrection = 140;
int fAngle = 0;
int fStrength = 0;
int maxFireStrength = 0;
int curFireStrength = 0;
boolean maxFireFound = false;
int fireStrengthSum = 0;
int fireStrengthAvg = 0;
//END DECLARATIONS FOR FIRE SENSING

boolean rotatedAtStart = false;
int diagTimeCount=0;

int numRoomsChecked = 0;

//KEEP TRACK OF RUNTIME
unsigned long time = 0;
unsigned long startTime = millis();
unsigned long stateStart = 0;

void loop(){
  while(true){
    //statemachine();
    //sensorDiagnostics();
    //testRotation();
    testWallFollow();
  }
}

void testWallFollow(){
  int rightfront = avgSensorVal(distRightFrontPin,3);
  int rightback = avgSensorVal(distRightBackPin,3);
  int front = avgSensorVal(distFrontPin,3);

  mcontrol.drive(rightback,rightfront,255);
  
  if(avgSensorVal(distFrontPin,3)>FRONTOBSTACLEDIST){
    rotCCW90();
  }
}

void testRotation(){
  rotCW90();
  delay(1000);
  rotCCW90();
  delay(2000);
}

void statemachine() {
  //while(1) Serial.println("Test2");
  switch(STATE){
    case STARTPUSHED:
      time = 0;
      startTime = millis();
      stateStart = 0;
      STATE = INITIALIZATION;
      break;
    case INITIALIZATION:
      //Rotate 90 deg CW at start
      if(!rotatedAtStart){
        rotCW90();
        rotatedAtStart = true;
      }
      
      //Is there something in front of me? If so, rotate 90 CCW
      if(avgSensorVal(distFrontPin,3)>FRONTOBSTACLEDIST){
        rotCCW90();
      }
      //Wall follow
      mcontrol.drive(analogRead(distRightBackPin),analogRead(distRightFrontPin),180);
      
      //After some seconds, initialization is over, so go to next state
      if(time>=INITIALIZATION_TIME){
        STATE = WALLFOLLOW;
        stateStart = time;
        #if DEBUG
          Serial.println("Changed state to WALLFOLLOW");
        #endif
      }
      
      break;
      
    case WALLFOLLOW:
      //Serial.println("WALL FOLLOWING");
      //Look for front obstacles:
      //Is there something in front of me? If so, rotate 90 CCW
      if(avgSensorVal(distFrontPin,3)>FRONTOBSTACLEDIST){
        rotCCW90();
      }
      mcontrol.drive(analogRead(distRightBackPin),analogRead(distRightFrontPin),180);
      
      
      //Look for lines. If found, change state to aligning with line
      #if LINESENSING_INVERTED
        if(analogRead(lineLeftPin)>LINESENSED || analogRead(lineRightPin)>LINESENSED){
          //STATE = ALIGNLINE;
          rightMotor.brake();
          leftMotor.brake();
          delay(200);                                        //Pause
          leftMotor.drive(80);                               //Drive forward slightly to
          rightMotor.drive(80);                              //verify that we are now off
          delay(400);                                        //the line and not mistakenly
          leftMotor.brake();                                 //reading the start pad
          rightMotor.brake();
          lineLeft  = analogRead(lineLeftPin);
          lineRight = analogRead(lineRightPin);
          #if LINESENSING_INVERTED
            lineLeftSensed  = (lineLeft  > LINESENSED);
            lineRightSensed = (lineRight > LINESENSED);
          #else
            lineLeftSensed  = (lineLeft  < LINESENSED);
            lineRightSensed = (lineRight < LINESENSED);
          #endif
          if(!(lineLeftSensed && lineRightSensed)){
            numRoomsChecked++;
            STATE = INROOM;
            leftMotor.brake();
            rightMotor.brake();
            stateStart = time;
            #if DEBUG
              Serial.println("Changed state to ALIGNLINE");
            #endif
          }
          else{
            STATE = INITIALIZATION; //We're on the pad again, so get off it
          }
        }
      #else
        if(analogRead(lineSensePin)<LINESENSED){
          //STATE = ALIGNLINE;
          rightMotor.brake();
          leftMotor.brake();
          delay(200);                                        //Pause
          leftMotor.drive(80);                               //Drive forward slightly to
          rightMotor.drive(80);                              //verify that we are now off
          delay(400);                                        //the line and not mistakenly
          leftMotor.brake();                                 //reading the start pad
          rightMotor.brake();
          lineVal  = analogRead(lineSensePin);
          #if LINESENSING_INVERTED
            lineSensed  = (lineVal  > LINESENSED);
          #else
            lineSensed  = (lineVal  < LINESENSED);
          #endif
          if(!lineSensed){
            numRoomsChecked++;
            STATE = INROOM;
            leftMotor.brake();
            rightMotor.brake();
            stateStart = time;
            #if DEBUG
              Serial.println("Changed state to ALIGNLINE");
            #endif
          }
          else{
            STATE = INITIALIZATION; //We're on the pad again, so get off it
          }
        }
      #endif
      
      break;
      
    case INROOM:
      fireStrengthSum = 0;
      for(int i=0; i<FIRESENSE_TRIALS; i++){
        fireStrengthSum += fireSense.fireStrength();
        delay(1);
      }
      fireStrengthAvg = fireStrengthSum / FIRESENSE_TRIALS;
      #if DEBUG
        Serial.println(fireStrengthAvg);
        delay(400);
      #endif
      if(fireStrengthAvg>=FIRESENSED){
        STATE = FOUNDFIRE;
        //Avoid hitting corner if fire is in corner
        leftMotor.drive(200);
        rightMotor.drive(200);
        delay(800);
        leftMotor.brake();
        rightMotor.brake();
      }
      else{
        leaveRoom();
        STATE = WALLFOLLOW;
      }
      
      break;
      
    case FOUNDFIRE:
      //ROTATE UNTIL FIRE IS DIRECTLY AHEAD
      //DRIVE FORWARD UNTIL FIRE READS SIGNIFICANTLY HIGH
      //TRIGGER CO2
      //STATE = RETURNHOME;
      fAngle = fireSense.fireAngle();
      fStrength = fireSense.fireStrength();
      
      
      //if(abs(fAngle)>=7){
        if(fAngle > FIREANGLETHRESH)
        {
          leftMotor.drive(fire_motinertia + fire_motcorrection);
          rightMotor.drive(fire_motinertia - fire_motcorrection);
        }
        else if(fAngle < -FIREANGLETHRESH){
          leftMotor.drive(fire_motinertia - fire_motcorrection);
          rightMotor.drive(fire_motinertia + fire_motcorrection);
        }
        else{
          leftMotor.drive(fire_motinertia);
          rightMotor.drive(fire_motinertia);
        }
        
        if(fStrength>=FIRECLOSE){
          leftMotor.brake();
          rightMotor.brake();
          STATE = PUTOUTFIRE;
          stateStart = time;
        }
      //}
      
      break;
      
    case ALIGNFIRE:
    //Rotate until maximum read on middle fire sensor
      //Sweep left
      if(time-stateStart<FIRESWEEPTIME)
      {
        leftMotor.drive(100);
        rightMotor.drive(-100);
        curFireStrength = fireSense.fireStrength();
        if(maxFireStrength<curFireStrength){
          maxFireStrength=curFireStrength;
        }
      }
      //Sweep right
      else if(time-stateStart<FIRESWEEPTIME*3){
        leftMotor.drive(-100);
        rightMotor.drive(100);
        curFireStrength = fireSense.fireStrength();
        if(maxFireStrength<curFireStrength){
          maxFireStrength=curFireStrength;
        }
      }
      //Turn left slowly until max is reached,
      //ideally pointing right at fire
      else{
        if(fireSense.fireStrength()<maxFireStrength && !maxFireFound){
          leftMotor.drive(70);
          rightMotor.drive(-70);
        }
        else{
          maxFireFound = true;
          leftMotor.brake();
          rightMotor.brake();
          STATE = PUTOUTFIRE;
          stateStart = time;
        }
      }
    
      break;
      
      
    case PUTOUTFIRE:
      extinguish();
      delay(200);
      fireStrengthSum = 0;
      for(int i=0; i<FIRESENSE_TRIALS; i++){
        fireStrengthSum += fireSense.fireStrength();
        delay(1);
      }
      fireStrengthAvg = fireStrengthSum / FIRESENSE_TRIALS;
      if(fireStrengthAvg<FIRESENSED){
        STATE = RETURNHOME;
      }
      delay(500);
      break;
      
    case RETURNHOME:

    break;
    
    case HOMEREACHED:
    
    break;
  }
  diagTimeCount++;
  if(diagTimeCount>2000){
    sensorDiagnostics();
    diagTimeCount=0;
  }
  
  
  time = millis() - startTime;
}

void sensorDiagnostics(){
  #if DEBUG
    Serial.println("------------Printing robot information------------");
    
    Serial.print("LINE SENSOR - Left:            ");
    Serial.println(analogRead(lineSensePin));
    
    Serial.println();
    
    Serial.print("FIRE SENSOR - 1:              ");
    Serial.println(analogRead(fireSensePin1));
    
    Serial.print("FIRE SENSOR - 2:            ");
    Serial.println(analogRead(fireSensePin2));
    
    Serial.print("FIRE SENSOR - 3:             ");
    Serial.println(analogRead(fireSensePin3));
    
    Serial.print("FIRE SENSOR - 4:             ");
    Serial.println(analogRead(fireSensePin4));
    
    Serial.print("FIRE SENSOR - 5:             ");
    Serial.println(analogRead(fireSensePin5));
    
    Serial.println();
    
    Serial.print("FIRE SENSOR - Angle:             ");
    Serial.println(fireSense.fireAngle());
    
    Serial.print("FIRE SENSOR - Strength:          ");
    Serial.println(fireSense.fireStrength());
    
    Serial.println();
    
    Serial.print("DISTANCE SENSOR - Right-Back:  ");
    Serial.println(analogRead(distRightBackPin));
    
    Serial.print("DISTANCE SENSOR - Right-Front: ");
    Serial.println(analogRead(distRightFrontPin));
    
    Serial.print("DISTANCE SENSOR - Front:       ");
    Serial.println(analogRead(distFrontPin));
    
    Serial.println("--------------------------------------------------");
  #endif
}



//Function for rotation (dead recknoning!)

//Rotate clockwise 90 degrees
void rotCW90(){
  leftMotor.drive(150);
  rightMotor.drive(-150);
  delay(550);
  leftMotor.brake();
  rightMotor.brake();
}

//Rotate counterclockwise 90 degrees
void rotCCW90(){
  leftMotor.drive(-150);
  rightMotor.drive(150);
  delay(630);
  leftMotor.brake();
  rightMotor.brake();
}

void leaveRoom(){
  leftMotor.drive(-255);
  rightMotor.drive(255);
  delay(1800);
  leftMotor.drive(250);
  rightMotor.drive(250);
  delay(1200);
  leftMotor.brake();
  rightMotor.brake();
}

void extinguish(){
  int pos;
  for(pos = SERVO_LOOSE; pos <= SERVO_TAUT; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    pullServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(8);                       // waits 15ms for the servo to reach the position 
  }
  
  for(pos = SERVO_TAUT; pos>=SERVO_LOOSE; pos -= 1)     // goes from 180 degrees to 0 degrees 
  {                                
    pullServo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(8);                       // waits 15ms for the servo to reach the position 
  }
  
  //pullServo.write(SERVO_LOOSE); 
}

int avgSensorVal(int pin, int trials){
  int sum=0;
  for(int i=0; i<trials; i++){
    sum += analogRead(pin);
  }
  return sum/trials;
}

