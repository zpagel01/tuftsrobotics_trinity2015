#include <Servo.h>

#include "motor.h"
#include "motorcontrol.h"
#include "fireSensorArray.h"

//Set 0 if this bot is "Irrelephant"
#define IS_JUMBOLT              1

//DEFINE ALL PINS HERE
#if IS_DELUX
  #define lineLeftPin           A0
  #define lineRightPin          A1
  #define distRightBackPin      A12
  #define distRightFrontPin     A11
  #define distFrontPin          A10
  #define distLeftBackPin       A14
  #define distLeftFrontPin      A13
  #define fireSensePin1         A3
  #define fireSensePin2         A4
  #define fireSensePin3         A2
  #define fireSensePin4         -1
  #define fireSensePin5         -1
  #define startButton           8
  #define servoPin              10

  //Motor pins
  #define leftMotordig           4
  #define leftMotorpwm           5
  #define rightMotordig          7
  #define rightMotorpwm          6
  
#else

  #define lineLeftPin           A0
  #define lineRightPin          A1
  #define distRightBackPin      A12
  #define distRightFrontPin     A11
  #define distFrontPin          A10
  #define distLeftBackPin       A14
  #define distLeftFrontPin      A13
  #define fireSensePin1         A2
  #define fireSensePin2         A2
  #define fireSensePin3         A2
  #define fireSensePin4         -1
  #define fireSensePin5         -1
  #define startButton           9
  #define servoPin              10

  //Motor pins
  #define leftMotordig           4
  #define leftMotorpwm           5
  #define rightMotordig          7
  #define rightMotorpwm          6
#endif

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
#define ALIGNLINE             5
#define PUTOUTFIRE            6
#define ALIGNFIRE             7
#define HOMEREACHED           9

#define INITIALIZATION_TIME   5500

//Wall-follow constants
#define FRONTOBSTACLEDIST     370

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
  
  //Get left motor in proper orientation
  leftMotor.flip();
  
  //Give these motors to the motor controller for wall following
  mcontrol.attach(&leftMotor,&rightMotor);
  
  //Initialize array of fire sense pins
  //and pass it to the fire sensor array object
  int fireSensePins[3] = {fireSensePin1,fireSensePin2,fireSensePin3};
  fireSense.attach(fireSensePins);
  fireSense.flip();
  
  
  #if DEBUG
    Serial.begin(9600);
  #endif
  
  pinMode(lineLeftPin,INPUT);
  pinMode(lineRightPin,INPUT);
  pinMode(distRightBackPin,INPUT);
  pinMode(distRightFrontPin,INPUT);
  pinMode(distFrontPin,INPUT);
  pinMode(distLeftBackPin,INPUT);
  pinMode(distLeftFrontPin,INPUT);
  pinMode(startButton,INPUT);
  
  //Wait for serial to begin
  #if DEBUG
    while(!Serial);
    Serial.println("Comm ready");
  #endif
  
  //Wait for start button
  while(digitalRead(startButton)==LOW){
    if(millis()%1000==0){
      sensorDiagnostics();
    }
  }
  #if DEBUG
    Serial.println("Started");
  #endif
}

//These declarations are for line adjustment
int lineLeft,lineRight;
boolean lineLeftSensed, lineRightSensed;
int rLineSide = LINE_INFRONT;   //Constants define
int lLineSide = LINE_INFRONT;   //at top of code
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
void loop() {
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
        if(analogRead(lineLeftPin)<LINESENSED || analogRead(lineRightPin)<LINESENSED){
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
          if(!lineLeftSensed && !lineRightSensed){
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
     
    
    case ALIGNLINE:
      
      lineLeft  = analogRead(lineLeftPin);
      lineRight = analogRead(lineRightPin);
      
      #if LINESENSING_INVERTED
        lineLeftSensed  = (lineLeft  > LINESENSED);
        lineRightSensed = (lineRight > LINESENSED);
      #else
        lineLeftSensed  = (lineLeft  < LINESENSED);
        lineRightSensed = (lineRight < LINESENSED);
      #endif
      /*
      Serial.print("Left line side: ");
      Serial.println(lLineSide);
      Serial.print("Right line side: ");
      Serial.println(rLineSide);
      */
      
      //Currently on the line on right side?
      if(rLineSide == ON_LINE){
        
        //If moving forward, moving past line -> LINE_BEHIND
        //If moving backward, moving past line -> LINE_INFRONT
                                   
        //This is done in case the robot is about to
        //go back off the line, so we know what side
        //of the line it must be on when it does
        
        if(rightMotor.isMovingForward()){
          rLineSide = LINE_BEHIND;
        }
        else{
          rLineSide = LINE_INFRONT;
        }
                                   
      }
      if (lLineSide == ON_LINE){
        if(leftMotor.isMovingForward()){         //Ditto
          lLineSide = LINE_BEHIND;
        }
        else{
          lLineSide = LINE_INFRONT;
        }
      }
      
      if (lineRightSensed){        //If we are still on the line, store that fact
        rLineSide = ON_LINE;
      }
      if (lineLeftSensed){         //...and for the left side too
        lLineSide = ON_LINE;
      }
      
      if (rLineSide == ON_LINE && lLineSide == ON_LINE){   //Both sides on line!
        #if DEBUG
          Serial.println("ALIGNED WITH LINE!!");
        #endif
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
        if(lineRightSensed && lineLeftSensed){             //If line still sensed, we're on pad
          STATE = WALLFOLLOW;                              //so continue wall following
        }
        else{
          STATE = INROOM;                                  //otherwise, we sensed a line!
        }
        stateStart = time;
      }
      
      
      //Drive motors appropriately
      if(rLineSide == LINE_INFRONT){        //If right side behind line...
        rightMotor.drive(LINE_ADJ_SPEED);   //...drive right side forward
        if(lLineSide == ON_LINE){           //..and if left side on line...
          leftMotor.drive(-LINE_ADJ_SPEED-20); //...drive left side opposite to stay in place
        }
      }
      if(rLineSide == LINE_BEHIND){         //If right side in front of line...
        rightMotor.drive(-LINE_ADJ_SPEED);  //...drive right side backward
        if(lLineSide == ON_LINE){           //..and if left side on line...
          leftMotor.drive(LINE_ADJ_SPEED+20);  //...drive left side opposite to stay in place
        }
      }
      if(lLineSide == LINE_INFRONT){        //If left side behind line...
        leftMotor.drive(LINE_ADJ_SPEED);    //...drive left side forward
        if(rLineSide == ON_LINE){           //and if right side on line...
          rightMotor.drive(-LINE_ADJ_SPEED-20);//...drive right side opposite to stay in place
        }
      }
      if(lLineSide == LINE_BEHIND){         //If left side in front of line...
        leftMotor.drive(-LINE_ADJ_SPEED);   //...drive left backward
        if(rLineSide == ON_LINE){           //and if right side on line...
          rightMotor.drive(LINE_ADJ_SPEED+20); //...drive right side opposite to stay in place
        }
      }
        
      
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
      //Serial.println("WALL FOLLOWING");
      //Look for front obstacles:
      //Is there something in front of me? If so, rotate 90 CCW
      if(avgSensorVal(distFrontPin,3)>FRONTOBSTACLEDIST){
        rotCW90();
      }
      mcontrol.drive(analogRead(distLeftFrontPin),analogRead(distLeftBackPin),180);
      
      
      //Look for lines. If found, change state to aligning with line
      #if LINESENSING_INVERTED
        if(analogRead(lineLeftPin)>LINESENSED || analogRead(lineRightPin)>LINESENSED){
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
          if(lineLeftSensed || lineRightSensed){
            STATE = HOMEREACHED;
          }
        }
      #else
        if(analogRead(lineLeftPin)<LINESENSED || analogRead(lineRightPin)<LINESENSED){
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
          if(lineLeftSensed || lineRightSensed){
            STATE = HOMEREACHED;
          }
        }
      #endif
      
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
    
    Serial.print("ROBOT MONICKER:                 ");
    if(IS_DELUX){
      Serial.println("Delux");
    } else {
      Serial.println("Irrelephant");
    }
    
    Serial.print("LINE SENSOR - Left:            ");
    Serial.println(analogRead(lineLeftPin));
    
    Serial.print("LINE SENSOR - Right:           ");
    Serial.println(analogRead(lineRightPin));
    
    Serial.println();
    
    Serial.print("FIRE SENSOR - Left:              ");
    Serial.println(analogRead(fireSensePin1));
    
    Serial.print("FIRE SENSOR - Center:            ");
    Serial.println(analogRead(fireSensePin2));
    
    Serial.print("FIRE SENSOR - Right:             ");
    Serial.println(analogRead(fireSensePin3));
    
    Serial.println();
    
    Serial.print("FIRE SENSOR - Angle:             ");
    Serial.println(fireSense.fireAngle());
    
    Serial.print("FIRE SENSOR - Strength:          ");
    Serial.println(fireSense.fireStrength());
    
    Serial.println();
    
    Serial.print("DISTANCE SENSOR - Left-Back:   ");
    Serial.println(analogRead(distLeftBackPin));
    
    Serial.print("DISTANCE SENSOR - Left-Front:  ");
    Serial.println(analogRead(distLeftFrontPin));
    
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
  leftMotor.drive(255);
  rightMotor.drive(-255);
  delay(1000);
}

//Rotate counterclockwise 90 degrees
void rotCCW90(){
  leftMotor.drive(-255);
  rightMotor.drive(255);
  delay(1000);
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

