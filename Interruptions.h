/*
 * This sketch sets up pins that would interrupts our robot's status
 * Main code is from 253 website
 * [IMPORTANT] not to be included in any file other than "ENPHRobot.ino"
 *
 ********************************
 *            MODES             *
 *                              *
 *    0  Lost                   *
 *    1  Tape Follower          *
 *    2  Grabbing               *
 *    3  IR Navigator           *
 *    4  Escaping               *
 *    5  Idle                   *
 *                              *
 ******************************** 
 * Setup code example:
 *      enableExternalInterrupt(INT0, LOW);
 *      enableExternalInterrupt(INT1, FALLING);
 *      enableExternalInterrupt(INT2, RISING);
 *	enableExternalInterrupt(INT3, RISING);
 */
 
# ifndef INTERRUPTIONS
# define INTERRUPTIONS

  #include <arduino.h>
  #include <avr/interrupt.h>  
  #include <LiquidCrystal.h>
  #include "Phys.h"
  #include <RobotStatus.h>
  #include "Robot.h"
  
//========= DEFINE CONSTANTS ==========
 //MODES:
 #define LOST          0
 #define TAPE_FOLLOWER 1
 #define GRABBING      2  
 #define IR_NAVIGATOR  3
 #define ESCAPING      4
 #define IDLE_MODE     5
 
 extern const int LEFT_MOTOR;
 extern const int RIGHT_MOTOR;
   
  volatile unsigned int INT_0 = 0;
  volatile unsigned int INT_1 = 1;
  volatile unsigned int INT_2 = 0;
  volatile unsigned int INT_3 = 0;
  //volatile boolean grabFlag_Left = false;       
  //volatile boolean grabFlag_Right = false;
   volatile unsigned int interrupt_time;

ISR(INT2_vect) {  
                    
                    
                   if(digitalRead(2) == 1){
                        psychobot.changeModeTo(GRABBING);
                        motor.speed(LEFT_MOTOR,0);
                        motor.speed(RIGHT_MOTOR,0);
                        psychobot.addOneMarker();
                   }
                   
                  // interrupt_time = millis();
               };
//ISR(INT0_vect) {Serial.println("interrupted");}//grabFlag_Left = true;}
//ISR(INT0_vect) {LCD.clear(); LCD.home(); LCD.print("INT0: "); LCD.print(INT_0++);};
//ISR(INT1_vect) {LCD.clear(); LCD.home(); LCD.print("INT1: ");LCD.print(INT_1++);};
//ISR(INT2_vect) {LCD.clear(); LCD.home(); LCD.print("INT2: "); LCD.print(INT_2++);};
//ISR(INT3_vect) {LCD.clear(); LCD.home(); LCD.print("INT3: "); LCD.print(INT_3++);};
   
  /*  Enables an external interrupt pin
  INTX: Which interrupt should be configured?
      INT0    - will trigger ISR(INT0_vect)
      INT1    - will trigger ISR(INT1_vect)
      INT2    - will trigger ISR(INT2_vect)
      INT3    - will trigger ISR(INT3_vect)
  mode: Which pin state should trigger the interrupt?
      LOW     - trigger whenever pin state is LOW
      FALLING - trigger when pin state changes from HIGH to LOW
      RISING  - trigger when pin state changes from LOW  to HIGH 
  */
  void enableExternalInterrupt(unsigned int INTX, unsigned int mode)
  {
  	if (INTX > 3 || mode > 3 || mode == 1) return;
  	cli();
  	/* Allow pin to trigger interrupts        */
  	EIMSK |= (1 << INTX);
  	/* Clear the interrupt configuration bits */
  	EICRA &= ~(1 << (INTX*2+0));
  	EICRA &= ~(1 << (INTX*2+1));
        /* Set new interrupt configuration bits   */
	EICRA |= mode << (INTX*2);
  	sei();
  
  }
   
  void disableExternalInterrupt(unsigned int INTX)
  {
  	if (INTX > 3) return;
  	EIMSK &= ~(1 << INTX);
  }

# endif
