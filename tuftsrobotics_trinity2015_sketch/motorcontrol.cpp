#include "motorcontrol.h"

MotorControl::MotorControl(int cType){
	motor1 = motor2 = NULL;
        controlType = cType;
        r_last = 0;
        l_last = 0;
        I_r = 0;
        I_l = 0;
}

void MotorControl::attach(int dig1, int pwm1, int dig2, int pwm2){
	motor1 = new Motor();
	motor1->attach(dig1,pwm1);
	motor2 = new Motor();
	motor2->attach(dig2,pwm2);
}

void MotorControl::attach(Motor* mot1, Motor* mot2){
	motor1 = mot1;
	motor2 = mot2;
}

void MotorControl::flip(){
	flip1();
	flip2();
}

void MotorControl::flip1(){
	if(motor1!=NULL) motor1->flip();
}

void MotorControl::flip2(){
	if(motor2!=NULL) motor2->flip();
}

void MotorControl::swapMotors(){
  Motor* temp = motor1;
  motor1 = motor2;
  motor2 = temp;
}

void MotorControl::drive(int p_left, int p_right, int inertia){
  if(controlType == PROPORTIONAL){
    int leftSpeed  = KP * (p_left - OPT)  + inertia;
    int rightSpeed = KP * (p_right - OPT) + inertia;
    //Serial.println(rightSpeed);
    motor1->drive(leftSpeed);
    motor2->drive(rightSpeed);
    
  }
  else if(controlType == DERIVATIVE){
  /*
   //base errors
    int l_error = (p_left - OPT);
    int r_error = (p_right - OPT);
    
   
    //derivitive error
    int D_l = (l_error-l_last)*KD;
    int D_r = (r_error-r_last)*KD;
    //proportional error
    int P_l  = KP * l_error  + inertia;
    int P_r = KP * r_error + inertia;

    int leftSpeed = P_l + D_l;
    int rightSpeed = P_r + D_r;
    //Serial.println(rightSpeed);
    //drive motors
    motor1->drive(leftSpeed);
    motor2->drive(rightSpeed);
    //keep error values for next iter.
    l_last = l_error;
    r_last = r_error;
    */
      int dist_from_opt = OPT - min(p_left,p_right); //dist_from_opt negative when too close
      int angle = p_right - p_left; //angle positive when pointed at wall
      int ideal_angle = 0.8 * dist_from_opt; //scale??
      if (ideal_angle >0) ideal_angle = 0.03 * (ideal_angle*ideal_angle);
      if (ideal_angle >200) ideal_angle = 200;
      #if DEBUG
        Serial.println(ideal_angle);
      #endif
      
      int error = (ideal_angle - angle);
      
      
      
      //angle should be proportional to dist_from_opt
      int P_l = KP * error + inertia;
      int P_r = KP * -error + inertia;
      
      int D_l = (error-l_last) *KD;
      int D_r = (r_last-error)*KD;
      int leftSpeed = P_l + D_l;
      int rightSpeed = P_r + D_r;
      if (leftSpeed > 255){
        rightSpeed -= (leftSpeed-255);
        leftSpeed -= (leftSpeed-255);
      }
      if (rightSpeed > 255){
        rightSpeed -= (rightSpeed-255);
        leftSpeed -= (rightSpeed-255);
      }      
      motor1->drive(leftSpeed);
      motor2->drive(rightSpeed);
      
      l_last = error;
      r_last = error;
      
    
  }
  else if(controlType == INTEGRAL){
    
     //base error
    //int l_error = (p_left - OPT);
    //int r_error = (p_right - OPT);
    
    int dist_from_opt = OPT - (p_left + p_right)/2; //dist_from_opt negative when too close
      int angle = p_right - p_left; //angle positive when pointed at wall
      int ideal_angle = 0.1 * dist_from_opt; //scale??
      
      int error = (ideal_angle - angle);
    //derivative error
    int D_l = (error-l_last)*KD;
    int D_r = (error-r_last)*KD;
    //porportional error
    int P_l  = KP * error  + inertia;
    int P_r = KP * error + inertia; 
    //integral accumulative error
    I_l += error;
    I_l = I_l*KI;
    I_r += error;
    I_r = I_r*KI;
    
    int leftSpeed = P_l + D_l + I_l;
    int rightSpeed = P_r + D_r + I_r;
    //Serial.println(rightSpeed);
    //drive motors
    motor1->drive(leftSpeed);
    motor2->drive(rightSpeed);

    l_last = error;
    r_last = error;
  }
}

void MotorControl::spin(int spd){
	if(motor1==NULL || motor2==NULL) return;
	motor1->drive(spd);
	motor2->drive(-spd);
}

void MotorControl::brake(){
	motor1->drive(0);
	motor2->drive(0);
}

int MotorControl::sensorToMm(int sensorval){
  return (41395.464 * pow(sensorval, -1.0947));
}
