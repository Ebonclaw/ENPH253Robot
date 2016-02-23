#pragma once

#include <LiquidCrystal.h>
#include <motor.h>
#include <ServoTimer2.h>

extern LiquidCrystal LCD;
extern motorClass motor;

extern ServoTimer2 RCServo0;    // declare variables for up to eight servos.   Replaced old Servo253 implementation 2015Jan2
extern ServoTimer2 RCServo1; 
extern ServoTimer2 RCServo2;

extern RobotStatus psychobot;

int knob(int value) ;	//	{ return analogRead(knobInput[value]) ;}

void buzzer (int value) ;//     { return pulseOut(buzzerOutput, value*2) ;}

void buzzerOff () 	;//     { return pulseStop(buzzerOutput ) ;}

int startbutton() 	;	//{ return !digitalRead(startInput) ;}

int stopbutton() 	;	//{ return !digitalRead(stopInput) ;}

void portMode(int portval, int value) ;

void portWrite(int portval, int value) ;

int portRead(int portval) ;
