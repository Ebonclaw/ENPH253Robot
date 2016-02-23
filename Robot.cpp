/*
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
 * Main Functions/Modes of Robot
 * Author: Ziyue Hu
 */

#include "arduino.h"
#include "Robot.h" 
#include "Phys.h"


#define WHITE_COURSE
//#define FIND_TAPE

#define TEST

    #ifdef TEST
      #define TESTTF
      //#define TESTGRAB
      #define TESTIR
      #define TESTESCAPE
    #endif
#define RUN_BACK
    
#define DISPLAY_STATUS

//extern RobotStatus psychobot;//omg

 
 
 //========= DEFINE CONSTANTS ========== 
 //MODES:
 #define LOST          0
 #define TAPE_FOLLOWER 1
 #define GRABBING      2  
 #define IR_NAVIGATOR  3
 #define ESCAPING      4
 #define IDLE_MODE     5
 
//Course status 
#define WHITE_COURSE   0
#define RED_COURSE     1

 //Markers
 #define FOURTH_MARKER 4
 
 //PINS(defined in main file):
 //ANALOG:
 extern const int FT_LEFT_QRD;
 extern const int FT_RIGHT_QRD;
 extern const int IR_F_LEFT;
 extern const int IR_F_RIGHT;
 extern const int DOLL_MARKER;
 
 //DIGITAL:
 // input:
 extern const int BASKET_READY;
 extern const int DOLL_PICKED;
 extern const int EX_SWITCH;
 extern const int DIGI_DOLL_MARKER;
 extern const int SIXTH_STOPPER;
 extern const int DOLL_PICKED;


 
 //MOTORS (defined in main file):
 extern const int LEFT_MOTOR;
 extern const int RIGHT_MOTOR;
 
 //Servos:
 //Upper arm : Servo2
 //Base :      Servo1
 

 
/**   [ TAPE FOLLOWER ]
 *        [MODE 1]
 *   - Purpose: Follow tape based on the PID Control
 *   - Mode: May change to GrabMode#(by interruption)/IR Navigator
 *
 *   @Param: 1.PGain: Proportional Gain for PID from menu input (stored in EEPROM) 
 *           2. DGain: Derivative Gain for PID from menu input (stored in EEPROM)
 *           3. Threshold: threshold of defferentiate white from black
 *           4. psychobot :  status of robot
 *  [adding]: if(digitalRead(DOLL_MARKER)==HIGH){psychobot.changeModeTo(GRABBING);
                        motor.speed(LEFT_MOTOR,0);
                        motor.speed(RIGHT_MOTOR,0);
                        psychobot.addOneMarker();}
 *   
 */
 #define TF_BACK_TIME 2300 //time for arm back
  byte TapeFollower(int Speed, int PGain, int DGain, int L_Threshold, int R_Threshold,int Mark_Threshold){
    
    //For Protection (knob now able to access negative scale)
     if( PGain<0 ) PGain = 28;
     if( DGain<0 ) DGain = 3;
     if( L_Threshold < 0) L_Threshold = 41;
     if( R_Threshold < 0) R_Threshold = 41;
     
     int error = 0; // defined error
     int prev_error = psychobot.TFReturnPrevError();
     int recent_error = 0;
     float prop = 0;
     float deri = 0;
     int q = 0;//
     int m = 0;//
     int con = 0;// correction P+D
     
     int ButtonCounter = 0;
     
     int ti = 0;
     int t = millis();
// NATURAL FOLLOWER:
     while(psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() <= 3){
       //>>>================== START PID ====================<<<<
       #ifdef DISPLAY_STATUS
       LCD.clear();LCD.home();
       LCD.setCursor(0,1);
       LCD.print("TapeFollowing");
       #endif
       //time interval
        // ti = millis() - t;
        // t = millis();
         //ht = millis();//time for hardcode
         
       //Doll marker:  
        if(analogRead(DOLL_MARKER) > Mark_Threshold && millis() - psychobot.lastSaved() > 500){
          
                        psychobot.addOneMarker();
                        //when mode at first three, stop and change to grab
                        if(psychobot.getMarkerNumber() < 4) {
                            psychobot.changeModeTo(GRABBING);
                            motor.speed(LEFT_MOTOR,15);
                            motor.speed(RIGHT_MOTOR,-15);
                          }
                          //when mode at fourth, go grab
                          else {
                            motor.speed(LEFT_MOTOR,0);
                            motor.speed(RIGHT_MOTOR,0);
                           // delay(1000);
                           psychobot.changeModeTo(GRABBING);
                          }
         }
                     
                        
        float left = analogRead(FT_LEFT_QRD);
        float right = analogRead(FT_RIGHT_QRD);
        
        if(left>L_Threshold && right>R_Threshold) error = 0;
        if(left>L_Threshold && right<R_Threshold) error = -1; //off right
        if(left<L_Threshold && right>R_Threshold) error = 1; //off left
        if(left<L_Threshold && right<R_Threshold) {
          if(prev_error > 0) error = 5; //off to left
          if(prev_error <= 0) error = -5; //off to right
        }
      
      // for derivative approximation
        if(!(error == prev_error)){
          recent_error = prev_error;
          q=m;
          m=1;
        }
      
        prop = PGain*error;
        deri = (int)(float)DGain*(float)(error-recent_error)/(float)(q+m);
        
        con = prop+deri;
        
        
        m +=1;
        //we need to deccelerate when in white course
      if(psychobot.GetCourse() == WHITE_COURSE){
        //check for random interrupt
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() != 1)  motor.speed(LEFT_MOTOR,Speed+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() != 1)  motor.speed(RIGHT_MOTOR,-Speed+con);//right -
        
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 1)  motor.speed(LEFT_MOTOR,4*Speed/5+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 1)  motor.speed(RIGHT_MOTOR,-4*Speed/5+con);//right -
      }

      else if(psychobot.GetCourse() == RED_COURSE){
        //check for random interrupt
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() != 3)  motor.speed(LEFT_MOTOR,Speed+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() != 3 )  motor.speed(RIGHT_MOTOR,-Speed+con);//right -
       
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 3)  motor.speed(LEFT_MOTOR,4*Speed/5+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 3 )  motor.speed(RIGHT_MOTOR,-4*Speed/5+con);//right -

    }

        
        prev_error = error;
        
        psychobot.TFSaveError(prev_error);//save for sake of losing error track.
        
       //>>>>====================== END PID ======================<<<<<
       
       //>>>>===================== Mode Check ====================<<<<<
      
       if(stopbutton() || digitalRead(EX_SWITCH) == LOW)    {psychobot.changeModeTo(IDLE_MODE);}//if stop is pressed twice       //MODE CHANGE
       
       //psychobot.changeModeTo(IDLE_MODE);
     
     }

// ************************************PATCH UP [Transfer from TF to IR] ***********************************************
//time interval restrain:
     //int restrain = 400;
     while(psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 4 ){
       //>>>================== START PID ====================<<<<
       #ifdef DISPLAY_STATUS
       LCD.clear();LCD.home();
       LCD.setCursor(0,1);
       LCD.print("TapeFollowing");
       #endif
       //time interval
         ti = millis() - t;
         
         
       //Doll marker:  
       /* if(digitalRead(DIGI_DOLL_MARKER) == HIGH){*/
                       // in fourth doll mode, we go to grab only when digital doll marker hits marker
                        motor.speed(LEFT_MOTOR,0);
                        motor.speed(RIGHT_MOTOR,0);
                       // psychobot.changeModeTo(GRABBING);//for four doll marker none stop
                       psychobot.changeModeTo(LOST);//for Four doll marker stops
                        //psychobot.changeModeTo(IR_NAVIGATOR);
                        //}
                        
                        
        float left = analogRead(FT_LEFT_QRD);
        float right = analogRead(FT_RIGHT_QRD);
        
        if(left>L_Threshold && right>R_Threshold) error = 0;
        if(left>L_Threshold && right<R_Threshold) error = -1; //off right
        if(left<L_Threshold && right>R_Threshold) error = 1; //off left
        if(left<L_Threshold && right<R_Threshold) {
          if(prev_error > 0) error = 5; //off to left
          if(prev_error <= 0) error = -5; //off to right
        }
      
      // for derivative approximation
        if(!(error == prev_error)){
          recent_error = prev_error;
          q=m;
          m=1;
        }
      
        prop = PGain*error;
        deri = (int)(float)DGain*(float)(error-recent_error)/(float)(q+m);
        
        con = prop+deri;
        
        
        m +=1;
        
        //check for random interrupt
        if (psychobot.getMode() == TAPE_FOLLOWER)  motor.speed(LEFT_MOTOR,Speed/3+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER)  motor.speed(RIGHT_MOTOR,-Speed/3+con);//right -
        
        prev_error = error;
        
        psychobot.TFSaveError(prev_error);//save for sake of losing error track.
        
       //>>>>====================== END PID ======================<<<<<
       
       //>>>>===================== Mode Check ====================<<<<<
      
       if(stopbutton() || digitalRead(EX_SWITCH) == LOW)    {psychobot.changeModeTo(IDLE_MODE);}//if stop is pressed tweice       //MODE CHANGE
       
       
     
     }
     
// *********************************************************************************************
   
   
// go back ********************* [AFTER ESCAPING]************************************************************* 
     boolean arm_back = false;  //arm status
     
     while(psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() > 4 ){
       //>>>================== START PID ====================<<<<
 /*      #ifdef DISPLAY_STATUS
       LCD.clear();LCD.home();
       LCD.setCursor(0,1);
       LCD.print("TapeFollowing");
       #endif*/
      
       //time interval
         ti = millis() - t;
                     
        if(analogRead(DOLL_MARKER) > Mark_Threshold && millis() - psychobot.lastSaved() > 500){
                     psychobot.addOneMarker();
        } 
       
        //long rt = millis();
        /*if(analogRead(DOLL_MARKER) > Mark_Threshold && millis() - psychobot.lastSaved() > 500){
                        LCD.clear();LCD.home();
                        LCD.print(psychobot.getMarkerNumber());
                        psychobot.addOneMarker(); // 6 7 8 9
        */                //when mode at doll return , stop and change to grab
                        if( arm_back==false && ti > TF_BACK_TIME) {
                            
                            motor.speed(LEFT_MOTOR,0);
                            motor.speed(RIGHT_MOTOR,0);
                            delay(500);
                            //arm go up a bit
                            int upper_angle = 150;
                            RCServo2.write(upper_angle);
                            delay(700);
                            //arm go front
                            int base_angle = 245; //CHANGE THIS
                            int base_angle_ser = map(base_angle,0,277,180,0);
                            RCServo1.write(base_angle_ser);
                            delay(900);
                            //arm go down
                            upper_angle = 30;
                            RCServo2.write(upper_angle);
                            
                            arm_back = true;
                            psychobot.saveTime(millis());
                          }
                          
         //}

        float left = analogRead(FT_LEFT_QRD);
        float right = analogRead(FT_RIGHT_QRD);
        
        if(left>L_Threshold && right>R_Threshold) error = 0;
        if(left>L_Threshold && right<R_Threshold) error = -1; //off right
        if(left<L_Threshold && right>R_Threshold) error = 1; //off left
        if(left<L_Threshold && right<R_Threshold) {
          if(prev_error > 0) error = 5; //off to left
          if(prev_error <= 0) error = -5; //off to right
        }
      
      // for derivative approximation
        if(!(error == prev_error)){
          recent_error = prev_error;
          q=m;
          m=1;
        }
      
        prop = PGain*error;
        deri = (int)(float)DGain*(float)(error-recent_error)/(float)(q+m);
        
        con = prop+deri;
        
        
        m +=1;
        if (psychobot.getMode() == TAPE_FOLLOWER && arm_back == false)  motor.speed(LEFT_MOTOR,2*Speed/3+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && arm_back == false)  motor.speed(RIGHT_MOTOR,-2*Speed/3+con);//right -
       
//        
//        //check for random interrupt
        if (psychobot.getMode() == TAPE_FOLLOWER && arm_back == true)  motor.speed(LEFT_MOTOR,4*Speed/7+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && arm_back == true)  motor.speed(RIGHT_MOTOR,-4*Speed/7+con);//right -

        prev_error = error;
        
        psychobot.TFSaveError(prev_error);//save for sake of losing error track.
        
       //>>>>====================== END PID ======================<<<<<
       
       //>>>>===================== Mode Check ====================<<<<<
      
       if(stopbutton() || digitalRead(EX_SWITCH) == LOW)    {psychobot.changeModeTo(IDLE_MODE);}//if stop is pressed tweice       //MODE CHANGE
   
     }
     
     return psychobot.getMode();
     
  }
  
  
  
  
  
  
  
 /**  [ GRABBING ]
  *    [MODE 2]
  *    This is the function that would do all grabings, different doll will trigger different 
  *    grabbing path.(called grabbing mode's secondary mode stored in RobotStatus object)
  *    the secodary mode is changed by how many doll marker interrupts has occured.
  *    
  *    Purpose: to performe grabbing mechanism
  *    Deteminating Inputs: 1. microswitch that checks if dolls are being attached.
  *                         2. timer to check if task should be abolished due to time constrain. 
  */
  #define FIRSTTHREE  0
  #define FOURTH      1
  #define SIXTH       2
  #define DOLL_RETURN 3
 //#define SWING//no swing
  
  byte Grabbing(void){
    
    #ifdef TESTGRAB
    
    int testCount = 3;
    while(testCount > 0){
    
       LCD.clear();LCD.home();
       LCD.setCursor(0,0);
       LCD.print("Grabbing ");LCD.print(testCount);
       LCD.setCursor(0,1);
       LCD.print("Grabed ");LCD.print(psychobot.getDollsNumber());
       delay(50);
    delay(1000);
    testCount--;
    }
    
    psychobot.addOneDoll();
    

   psychobot.changeModeTo(TAPE_FOLLOWER);                                                                       // MODE CHANGE
    

    
    #endif //TESTGRAB
    
    
    
    #ifndef TEST
    #ifdef DISPLAY_STATUS
       LCD.clear();LCD.home();
       LCD.setCursor(0,1);
      // LCD.print("Grabbing ");
    #endif // DISPLAY_STATUS
    #endif //TEST  
   
   #ifndef TESTGRAB
    //Upper arm : Servo2
    //Base :      Servo1
    //Release:    Servo0
    
    //FIRSTTHREE doll path
     if(psychobot.getGrabMode() == FIRSTTHREE){
       
       /* first three dolls Code Here */
       
       int Base_grabInterval   = 3000;
       int Base_discretSteps   = 1;
       int Base_finalAngle     = 150;//150
       int Base_initialAngle   = 0;
       
       int Upper_grabInterval  = 3000;
       int Upper_discretSteps  = 1;
       int Upper_finalAngle    = 0;//0
       int Upper_initialAngle  = 165; 
       
       int base_time   = 0;
       int upper_time  = 0;
       int upper_angle = 165;
       int base_angle  = 0; 
       
       int base_angle_ser = 0;
       
       if(psychobot.getMarkerNumber() == 1 ) {RCServo1.write(180);delay(1000);RCServo2.write(Upper_initialAngle);delay(400);}

       while (base_angle < Base_finalAngle && base_time < Base_grabInterval){
         #ifdef DISPLAY_STATUS
         LCD.clear();LCD.home();
         LCD.print("Sub 0 - base ");
         #endif
         //delay(800);// TEST
         base_angle += Base_finalAngle/Base_discretSteps;
         int base_angle_rc;
         
         base_angle_rc = map(base_angle,0,277,180,0);
         RCServo1.write(base_angle_rc);
         //delay(Base_grabInterval/Base_discretSteps);
         delay(200);
         base_time += Base_grabInterval/Base_discretSteps;
         
         LCD.print(base_time);

       }

       //go down
       delay(300);
       RCServo2.write(0);
       delay(800);
       //go up
       RCServo2.write(165);
       delay(500);
       
     
      // second try if not picked
      if(digitalRead(DOLL_PICKED) == LOW){
       // adjust
       base_angle += 20;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(300);
       
       //go down
       RCServo2.write(0);
       delay(800);
       //go up
       RCServo2.write(165);
       delay(500);
      }
/*      
     // third try  
       base_angle -= 20;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);     
      
       
       delay(800);
       RCServo2.write(0);
       delay(800);
       RCServo2.write(165);
       delay(800);
 */  
//       base_angle = Base_finalAngle;
//       base_angle_ser = map(base_angle,0,277,180,0);
//       RCServo1.write(base_angle_ser);

      // go back: different doll different angle

       //delay(800);
       
       //go back
       //fist step
       base_angle = 90;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       //second step
       if(psychobot.getMarkerNumber() == 1)         base_angle = 0;
       else if(psychobot.getMarkerNumber() == 2)    base_angle = 30;
       else if(psychobot.getMarkerNumber() == 3)    base_angle = 0; // no swing: 50  swing: 277                                 
       else                                         base_angle = 0;
       

       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       

       

       //release
       RCServo0.write(160);//!!!!!!!!!!!!!!!!!!!!
       delay(500);
       //tighten
       RCServo0.write(0);
       
       if(psychobot.getMarkerNumber() == 3) RCServo2.write(120);
 #ifndef FIND_TAPE
        psychobot.changeModeTo(TAPE_FOLLOWER);
 #endif
       
 #ifdef FIND_TAPE
        psychobot.changeModeTo(LOST);
 #endif
       
     }
     
     //FOURTH doll path
     else if(psychobot.getGrabMode() == FOURTH){
              
       int Base_grabInterval   = 3000;
       int Base_discretSteps   = 1;
       int Base_finalAngle     = 250;
       int Base_initialAngle   = 0;
       int Clearance_Angle     = 140; // CHANGE This 
       int Swipe_Angle         = 195;
       
       int Upper_grabInterval  = 3000;
       int Upper_discretSteps  = 1;
       int Upper_finalAngle    = 0;
       int Upper_initialAngle  = 165; 
       
       int base_time   = 0;
       int upper_time  = 0;
       int upper_angle = 165;
       int base_angle  = 0; 
       
        int base_angle_ser = 0;



        //go to initial
        RCServo2.write(Upper_initialAngle);
        //delay(500);
        
       //release
       RCServo0.write(160);//!!!!!!!!!!!!!!!!!!!!
       delay(500);
       //tighten
       RCServo0.write(0);
       //delay(500);
       
       //go to front
       base_angle = Base_finalAngle;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(800);
       
       //go down
       upper_angle = Upper_finalAngle;
       RCServo2.write(upper_angle);
       delay(800);
       //go up
       upper_angle = Upper_initialAngle;
       RCServo2.write(upper_angle);
       delay(500);
       
       //second try
       if(digitalRead(DOLL_PICKED) == LOW){
       // adjust
       base_angle += 15;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       
       //go down
       RCServo2.write(0);
       delay(800);
       //go up
       RCServo2.write(165);
       delay(500);
      }
       
      #ifndef SWING
      //go back
      //1st step
       base_angle -= 70 ; //50 for 5 steps
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       //2nd step
       base_angle -= 70; //50 for 5 steps
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       //3nd step
 /*      base_angle -= 50;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       //4th step
       base_angle -= 50;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);*/
       //final step
       base_angle = Base_initialAngle;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       #endif //SWING
  
  #ifdef SWING
       //ready to swing
       RCServo1.write(5);
       delay(600);
       //Swing!
       base_angle = Base_finalAngle/3;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(300);
  #endif  
       
       //release
       RCServo0.write(160);
       delay(500);
       
       //tighten
       RCServo0.write(0);
/*       
       // Go front
       base_angle = Base_finalAngle + 30;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(700);
       //go down
       upper_angle = Upper_finalAngle;
       RCServo2.write(upper_angle);
       delay(500);
       //swipe right
       base_angle = Swipe_Angle; //change this
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(800);
       //go up
       upper_angle = Upper_initialAngle;
       RCServo2.write(upper_angle);
       delay(500); */
       //go gear clearance angle 
       base_angle_ser = Clearance_Angle;
       RCServo1.write(base_angle_ser);
       delay(500);
       
       /* fourth doll Code Here */
       
       // tape to IR transfer
       psychobot.changeModeTo(LOST);  
       
       
       
     }
     
     else if(psychobot.getGrabMode() == SIXTH ){
       
       /* fifth doll Code Here */
       int Base_grabInterval   = 3000;
       int Base_discretSteps   = 1;
       int Base_finalAngle     = 260;
       int Base_initialAngle   = 0;
       int Base_ReturnAngle    = 100;
       
       int Upper_grabInterval  = 3000;
       int Upper_discretSteps  = 1;
       int Upper_finalAngle    = 0;
       int Upper_initialAngle  = 165; 
       
       int base_time   = 0;
       int upper_time  = 0;
       int upper_angle = 165;
       int base_angle  = 0; 
       
       int base_angle_ser = 0;

       //release
       RCServo0.write(160);//!!!!!!!!!!!!!!!!!!!!
       delay(500);
       //tighten
       RCServo0.write(0);
       //delay(500);
       
       //go down a bit
       upper_angle = Upper_initialAngle - 30;
       RCServo2.write(upper_angle);
       delay(500);
       
       //go front
       base_angle = Base_finalAngle;
       base_angle_ser = 0; //map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(800);
       
       //go down
       upper_angle = Upper_finalAngle + 15;
       RCServo2.write(upper_angle);
       delay(800);
       
       //swipe right 1
       base_angle -= 75;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(300);
   
       //swipe left 1
       base_angle += 140;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(300);
       
       //swipe right 2
       base_angle -= 140;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(300);
       
       //swipe left 2
       base_angle += 140;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(300);

  
       //swipe right 3
       base_angle -= 140;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(300);
       
       //swipe left 3
       base_angle += 140;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(300);
       
       //go up
       upper_angle = Upper_initialAngle - 110;
       RCServo2.write(upper_angle);
       delay(500);
       
       //go right
       base_angle -= 60;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       
       //go down
       upper_angle = Upper_finalAngle;
       RCServo2.write(upper_angle);
       delay(500);
       
      //swipe right
       base_angle -= 70;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);       
       
       //swipe left
       base_angle += 100;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       
       //go up
       upper_angle = Upper_initialAngle - 110;
       RCServo2.write(upper_angle);
       delay(500);
       
/*       //swipe left
       base_angle += 30;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
       
       //go down
       upper_angle = Upper_finalAngle;
       RCServo2.write(upper_angle);
       delay(500);
       
       //swipe right
       base_angle -= 30;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);    
       //swipe left
       base_angle += 45;
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(500);
*/       
       //go up
       upper_angle = Upper_initialAngle - 10;
       RCServo2.write(upper_angle);
       delay(800);
       
       //go back
       base_angle = Base_ReturnAngle;//150
       base_angle_ser = map(base_angle,0,277,180,0);
       RCServo1.write(base_angle_ser);
       delay(800);
       
       //go down a bit
       upper_angle -= 20;
       RCServo2.write(upper_angle);
       delay(300);
       
       
  /*     //go down
       upper_angle = Upper_finalAngle/2;
       RCServo2.write(upper_angle);
       delay(500);*/
       
       psychobot.changeModeTo(IR_NAVIGATOR);
     }
     
     else {
       
       /* error? */
       psychobot.changeModeTo(TAPE_FOLLOWER);
     }
     
    #endif //not TEST
    psychobot.addOneDoll();
    psychobot.saveTime(millis());
    return psychobot.getMode();
  }
  
  
  
  
  
  
  
/**      [ IR NAVIGATOR ] 
 *           [MODE 3]
 *   - Purpose: use PID control to navigate through IR area 
 *   - Mode Change: Manual change to other modes / dolls picked 
 *   - [IMPORTANT] : Do history Mode check before use
 *
 *   @Param: IR P-gain, IR D-Gain,IR Threshold, speed
 *   @return: 
 */
 #define IR_BACK_UP 3000
 byte IRNavigator(int Speed, int PGain, int DGain, int SensorDiff,int mark_threshold){
   
     //For Protection (knob now able to access negative scale)
     if( PGain<0 ) PGain = 28;
     if( DGain<0 ) DGain = 3;

 
     int error = 0; // defined error
     int prev_error = 0;
     int recent_error = 0;
     float prop = 0;
     float deri = 0;
     int q = 0;//
     int m = 0;//
     int con = 0;// correction P+D
     
     int ButtonCounter = 0;
// ------- NORMAL IR NAVIGATION: ----------  
            // for turn around time count 
     long tir = millis();
     while(psychobot.getMode() == IR_NAVIGATOR && psychobot.getGrabMode() < SIXTH){
       //>>>================== START PID ====================<<<<
       #ifdef DISPLAY_STATUS
       LCD.clear();LCD.home();
       LCD.print("IR_L ");LCD.print(analogRead(IR_F_LEFT));
       LCD.setCursor(0,1);
       LCD.print("IR_R ");LCD.print(analogRead(IR_F_RIGHT));
       #endif
       
       //"Doll marker":  
        if(digitalRead(SIXTH_STOPPER) == HIGH ){
                       // in fifth doll mode, we go to grab only when sixth_stopper hits "marker"
                       psychobot.addOneMarker();
                        motor.speed(LEFT_MOTOR,0);
                        motor.speed(RIGHT_MOTOR,0);
                        psychobot.changeModeTo(GRABBING);
                       
                        
                        }
       else if(millis() - tir > IR_BACK_UP ){
                       // in fifth doll mode, we go to grab only when sixth_stopper hits "marker"
                       psychobot.addOneMarker();
                        motor.speed(LEFT_MOTOR,0);
                        motor.speed(RIGHT_MOTOR,0);
                        delay(300);
        }
       
       float read_difference = analogRead(IR_F_LEFT)-analogRead(IR_F_RIGHT); 
       
       
       if(read_difference == SensorDiff) error = 0;
       if( read_difference > SensorDiff) error = -1; //off right
       if( read_difference < SensorDiff) error = 1;  //off left
       //if( read_difference > SensorDiff && read_difference < SensorDiff + abs(SensorDiff/2)) error = -1; //off right
       //if( read_difference > SensorDiff - abs(SensorDiff/2) && read_difference < SensorDiff  ) error = 1; //off left
       
       
       //!!!!!!!!!! PROBLEMATIC !!!!!!!!!!!!!!!
       // not triggering for now
       if(read_difference > SensorDiff + abs(SensorDiff) && read_difference < SensorDiff + abs(SensorDiff) ) {
          if(prev_error > 0) error = 5; //off to left
          if(prev_error <= 0) error = -5; //off to right
        }
        
        #ifdef TESTIR
        LCD.clear();LCD.home();
        LCD.print("IR_L ");LCD.print(analogRead(IR_F_LEFT));
        LCD.setCursor(0,1);
        LCD.print("IR_R ");LCD.print(analogRead(IR_F_RIGHT));
        LCD.print("error: "); LCD.print(error);
        #endif
         // for derivative approximation
        if(!(error == prev_error)){
          recent_error = prev_error;
          q=m;
          m=1;
        }
      
        prop = PGain*error;
        deri = (int)(float)DGain*(float)(error-recent_error)/(float)(q+m);
        
        con = prop+deri;
        
        
        m +=1;
        
        //check for random interrupt
        if (psychobot.getMode() == IR_NAVIGATOR)  motor.speed(LEFT_MOTOR,Speed*5/4+con);//left
        if (psychobot.getMode() == IR_NAVIGATOR)  motor.speed(RIGHT_MOTOR,-Speed*5/4+con);//right
       
         prev_error = error;
       
       //>>>>====================== END PID ======================<<<<<
       
       //>>>>===================== Mode Check ====================<<<<<
      
       if(stopbutton() || digitalRead(EX_SWITCH) == LOW)      {psychobot.changeModeTo(IDLE_MODE);}//if stop is pressed        // MODE CHANGE
       
     }
    
    
#ifdef RUN_BACK 
     // ------- REVERSED IR NAVIGATION: ---------- ***************************************
     int ti = 0;           // for turn around time count 
     long t = millis();
     const int time_limit = 2100;
     
      while(psychobot.getMode() == IR_NAVIGATOR /*&& psychobot.getGrabMode() == SIXTH*/ ){
       //>>>================== START PID ====================<<<<
       #ifdef DISPLAY_STATUS
       LCD.clear();LCD.home();
       LCD.print("IR Navigating");
       #endif
       //"Doll marker": 
       ti = millis() - t; 
       if( ti >= time_limit){
                       // when coming back, we got lost mode and spin until finds tape
                      
                        motor.speed(LEFT_MOTOR,0);
                        motor.speed(RIGHT_MOTOR,0);
                        psychobot.changeModeTo(ESCAPING);
                       
                        
                        }
       
       float read_difference = analogRead(IR_F_LEFT)-analogRead(IR_F_RIGHT); 
       
       
       if(read_difference == SensorDiff) error = 0;
       if( read_difference > SensorDiff) error = -1; //off right
       if( read_difference < SensorDiff) error = 1;  //off left
       //if( read_difference > SensorDiff && read_difference < SensorDiff + abs(SensorDiff/2)) error = -1; //off right
       //if( read_difference > SensorDiff - abs(SensorDiff/2) && read_difference < SensorDiff  ) error = 1; //off left
       
       
       //!!!!!!!!!! PROBLEMATIC !!!!!!!!!!!!!!!
       // not triggering for now
       if(read_difference > SensorDiff + abs(SensorDiff) && read_difference < SensorDiff + abs(SensorDiff) ) {
          if(prev_error > 0) error = 5; //off to left
          if(prev_error <= 0) error = -5; //off to right
        }
        
        #ifdef TESTIR
        LCD.setCursor(0,1);
        LCD.print("error: "); LCD.print(error);
        #endif
         // for derivative approximation
        if(!(error == prev_error)){
          recent_error = prev_error;
          q=m;
          m=1;
        }
      
        prop = PGain*error;
        deri = (int)(float)DGain*(float)(error-recent_error)/(float)(q+m);
        
        con = prop+deri;
        
        
        m +=1;
        
        //check for random interrupt
        if (psychobot.getMode() == IR_NAVIGATOR)  motor.speed(LEFT_MOTOR,-Speed*5/4+con);//left
        if (psychobot.getMode() == IR_NAVIGATOR)  motor.speed(RIGHT_MOTOR,Speed*5/4+con);//right
       
         prev_error = error;
       
       //>>>>====================== END PID ======================<<<<<
       
       //>>>>===================== Mode Check ====================<<<<<
      
       if(stopbutton() || digitalRead(EX_SWITCH) == LOW)      {psychobot.changeModeTo(IDLE_MODE);}//if stop is pressed        // MODE CHANGE
       
     }//REVERSE IR NAVIGATION  
     #endif //RUN_BACK
     
     return psychobot.getMode();
 }
 
 /**    [ ESCAPE ] 
 *       [MODE 4]
 *   - Purpose: to patch up transition from IR to tapeFollow
 *   - Mode Change: triggerd by doll markers hit after sixth
 *   - [IMPORTANT]: OPEN LOOP
 *   @Param: NONE 
 */

 
 byte Escape(int robot_speed,int left_threshold,int right_threshold){
     
   
   while(psychobot.getMode() == ESCAPING
           && analogRead(FT_LEFT_QRD) < left_threshold && analogRead(FT_RIGHT_QRD) < right_threshold ){
    LCD.clear();LCD.home();LCD.print("FLEE !");    
     motor.speed(LEFT_MOTOR,robot_speed*4/5); motor.speed(RIGHT_MOTOR,0);
         if(stopbutton() || digitalRead(EX_SWITCH) == LOW)      {psychobot.changeModeTo(IDLE_MODE);}
         
   }
   motor.speed(LEFT_MOTOR,0); motor.speed(RIGHT_MOTOR,0);
   
   psychobot.changeModeTo(TAPE_FOLLOWER);
 
   psychobot.saveTime(millis());
   return psychobot.getMode();
 }
 
 
 
 
 /**    [ IDLING ] ------ Use with EXTREME caution
 *        [MODE 5]
 *   - Purpose: to make our stay in place, halt all motions 
 *   - Mode Change: Manual change to other modes
 *   - [IMPORTANT]: Use with EXTREME caution, halt all program until manual input
 *
 *   @Param: NONE 
 */
 
 byte Idling(void){
   motor.speed(LEFT_MOTOR,0);motor.speed(RIGHT_MOTOR,0);
   return psychobot.getMode();
 }
 
 
  /**     [ LOST ]
 *        [MODE 0]
 *   - Purpose: to patch up transition from tapeFollow to IR
 *   - Mode Change: triggerd by doll markers hit after fourth
 *   - [IMPORTANT]: OPEN LOOP
 *
 *   @Param: speed of robot 
 */
 #define LOST_THRESH 50
 #define LOST_TIME   650//ms
 #define FIND_TIME   350//ms
 byte Lost(int robot_speed,int left_threshold,int right_threshold){
     #ifdef DISPLAY_STATUS
      LCD.clear();LCD.home();
       LCD.setCursor(0,1);
       LCD.print("Lost");
     #endif
     

    if(psychobot.getMarkerNumber() == 4){
     int ti = 0;           // for noise resistance 
     long t = millis();
     
    while(millis() - t < LOST_TIME ) {
       motor.speed(RIGHT_MOTOR, -robot_speed*2/3); motor.speed(LEFT_MOTOR,robot_speed*4/5);}
     
    while(psychobot.getMode() == LOST
           && analogRead(IR_F_LEFT) < LOST_THRESH && analogRead(IR_F_RIGHT) < LOST_THRESH ){
             
          LCD.clear();LCD.home();LCD.print("LOST !");    
          motor.speed(LEFT_MOTOR,0); motor.speed(RIGHT_MOTOR,-robot_speed*4/5);
          
         if(stopbutton() || digitalRead(EX_SWITCH) == LOW)      {psychobot.changeModeTo(IDLE_MODE);}
         
    }
   
      psychobot.changeModeTo(IR_NAVIGATOR);
   }
   #ifdef FIND_TAPE
   
    else if(psychobot.getMarkerNumber() < 3){
     int ti = 0;           // for noise resistance 
     long t = millis();
      while(psychobot.getMode() == LOST
           && analogRead(FT_LEFT_QRD) < left_threshold && analogRead(FT_RIGHT_QRD) < right_threshold 
           && millis() - t < FIND_TIME){
            motor.speed(LEFT_MOTOR,robot_speed/2); motor.speed(RIGHT_MOTOR,robot_speed/2);
            if(stopbutton() || digitalRead(EX_SWITCH) == LOW)      {psychobot.changeModeTo(IDLE_MODE);}        
       }
       if(analogRead(FT_LEFT_QRD) > left_threshold && analogRead(FT_RIGHT_QRD) > right_threshold){
         psychobot.changeModeTo(TAPE_FOLLOWER);
         }
       
       while(psychobot.getMode() == LOST
           && analogRead(FT_LEFT_QRD) < left_threshold && analogRead(FT_RIGHT_QRD) < right_threshold 
           && millis() - t < FIND_TIME*2){
            motor.speed(LEFT_MOTOR,-robot_speed/2); motor.speed(RIGHT_MOTOR,-robot_speed/2);
            if(stopbutton() || digitalRead(EX_SWITCH) == LOW)      {psychobot.changeModeTo(IDLE_MODE);}        
       }
       if(analogRead(FT_LEFT_QRD) > left_threshold && analogRead(FT_RIGHT_QRD) > right_threshold){
         psychobot.changeModeTo(TAPE_FOLLOWER);
       }
       psychobot.changeModeTo(TAPE_FOLLOWER);
     }
   #endif
 }
 
 /****************************************************************************
  ****************************************************************************
  ***************           HALF COURSE FUNCTIONS       **********************
  ****************************************************************************
  ****************************************************************************/
  
  
  /**       [ HALF Tapefollower ] 
    *        [MODE 1 (half)]
    *
    *   - Purpose: Follow tape based on the PID Control
    *   - Mode: May change to GrabMode#(by interruption)/IR Navigator
    *
    *   @Param: 1.PGain: Proportional Gain for PID from menu input (stored in EEPROM) 
    *           2. DGain: Derivative Gain for PID from menu input (stored in EEPROM)
    *           3. Threshold: threshold of defferentiate white from black
    *           4. Speed of robot
    *   
    *   Note:   consists of transfer from tape follower to half Lost  
    */
    
    #define HALF_BACK 1000
    byte HalfTapeFollower(int Speed, int PGain, int DGain, int L_Threshold, int R_Threshold,int Mark_Threshold){
      
         //For Protection (knob now able to access negative scale)
         if( PGain<0 ) PGain = 28;
         if( DGain<0 ) DGain = 3;
         if( L_Threshold < 0) L_Threshold = 41;
         if( R_Threshold < 0) R_Threshold = 41;
         
         int error = 0; // defined error
         int prev_error = psychobot.TFReturnPrevError();
         int recent_error = 0;
         float prop = 0;
         float deri = 0;
         int q = 0;//
         int m = 0;//
         int con = 0;// correction P+D
         
         int ButtonCounter = 0;
         
         int ti = 0;
         int t = millis();
    // NATURAL FOLLOWER:
         while(psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() <= 3){
           //>>>================== START PID ====================<<<<
           #ifdef DISPLAY_STATUS
           LCD.clear();LCD.home();
           LCD.setCursor(0,1);
           LCD.print("Half TapeFollow");
           #endif
           //time interval
            // ti = millis() - t;
            // t = millis();
             //ht = millis();//time for hardcode
             
           //Doll marker:  
            if(analogRead(DOLL_MARKER) > Mark_Threshold && millis() - psychobot.lastSaved() > 500){
              
                            psychobot.addOneMarker();
                            //when mode at first three, stop and change to grab
                            if(psychobot.getMarkerNumber() < 4) {
                                psychobot.changeModeTo(GRABBING);
                                motor.speed(LEFT_MOTOR,10);
                                motor.speed(RIGHT_MOTOR,-10);
                              }
    
             }
             
             // go to lost after third
             if(psychobot.getDollsNumber() == 3 && millis() - t > HALF_BACK ){psychobot.changeModeTo(LOST);}
                         
                            
            float left = analogRead(FT_LEFT_QRD);
            float right = analogRead(FT_RIGHT_QRD);
            
            if(left>L_Threshold && right>R_Threshold) error = 0;
            if(left>L_Threshold && right<R_Threshold) error = -1; //off right
            if(left<L_Threshold && right>R_Threshold) error = 1; //off left
            if(left<L_Threshold && right<R_Threshold) {
              if(prev_error > 0) error = 5; //off to left
              if(prev_error <= 0) error = -5; //off to right
            }
          
          // for derivative approximation
            if(!(error == prev_error)){
              recent_error = prev_error;
              q=m;
              m=1;
            }
          
            prop = PGain*error;
            deri = (int)(float)DGain*(float)(error-recent_error)/(float)(q+m);
            
            con = prop+deri;
            
            
            m +=1;
            
      if(psychobot.GetCourse() == WHITE_COURSE){
        //check for random interrupt
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() != 1)  motor.speed(LEFT_MOTOR,Speed+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() != 1)  motor.speed(RIGHT_MOTOR,-Speed+con);//right -
        
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 1)  motor.speed(LEFT_MOTOR,4*Speed/5+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 1)  motor.speed(RIGHT_MOTOR,-4*Speed/5+con);//right -
      }

      else if(psychobot.GetCourse() == RED_COURSE){
        //check for random interrupt
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() != 3)  motor.speed(LEFT_MOTOR,Speed+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() != 3 )  motor.speed(RIGHT_MOTOR,-Speed+con);//right -
       
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 3)  motor.speed(LEFT_MOTOR,4*Speed/5+con);//left
        if (psychobot.getMode() == TAPE_FOLLOWER && psychobot.getMarkerNumber() == 3 )  motor.speed(RIGHT_MOTOR,-4*Speed/5+con);//right -

    }
           prev_error = error;
            
            psychobot.TFSaveError(prev_error);//save for sake of losing error track.
            
           //>>>>====================== END PID ======================<<<<<
           
           //>>>>===================== Mode Check ====================<<<<<
          
           if(stopbutton() || digitalRead(EX_SWITCH) == LOW)    {psychobot.changeModeTo(IDLE_MODE);}//if stop is pressed twice       //MODE CHANGE
           
           return psychobot.getMode();
          
          }
    }
    
    /**     [ ESCAPE ]
     *        [MODE 4]
     *   - Purpose: to patch up 360 turning of robot in half course mode
     *   - Mode Change: triggerd by doll markers hit after fourth
     *   - [IMPORTANT]: OPEN LOOP
     *
     *   @Param: speed of robot 
     */
     
     byte HalfEscape(int robot_speed,int left_threshold,int right_threshold){
        while(psychobot.getMode() == ESCAPING
           && analogRead(FT_LEFT_QRD) < left_threshold && analogRead(FT_RIGHT_QRD) < right_threshold ){
                  LCD.clear();LCD.home();LCD.print("FLEE !");    
                  motor.speed(LEFT_MOTOR,robot_speed*4/5); motor.speed(RIGHT_MOTOR,robot_speed*2/3);
                  
                  if(stopbutton() || digitalRead(EX_SWITCH) == LOW)      {psychobot.changeModeTo(IDLE_MODE);}
         
         }
        motor.speed(LEFT_MOTOR,0); motor.speed(RIGHT_MOTOR,0);
   
        psychobot.changeModeTo(TAPE_FOLLOWER);
        return psychobot.getMode();
     }
