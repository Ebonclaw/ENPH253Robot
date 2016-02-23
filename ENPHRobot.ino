/*
 *
/************************************************** HARDWARE **************************************************
 *                                                    |                                                       *
 *             Digital Pin Assignment                 |                    Analog Pin Assignment(Reversed)    *
 * PIN     Function                 Input/Output      |      * PIN     Function                  Input/Output *
 * ---------------------------------------------------------------------------------------------------------- *
 * D0                                   Input         |      * A0      TapeSensor_L                  Input    *
 * D1        Doll Marker                Input         |      * A1      TapeSensor_R                  Input    *
 * D2                                   Input         |      * A2      IR_FrontLeft                  Input    *
 * D3                                   Input         |      * A3      IR_FrontRight                 Input    *
 * D4                                   Input         |      * A4      IR_RearLeft                   Input    *
 * D5                                   Input         |      * A5      IR_RearRight                  Input    *
 * D6                                   Input         |      * A6      Knob6/DollMarker              Input    *
 * D7                                   Input         |      * A7      Knob7/                        Input    *
 * D8                                   Input         |                                                       *
 * D9                                   Input         |                                                       *
 * D10                                  Input         |                                                       *
 * D11                                  Input         |                                                       *
 * D12                                  Input         |                                                       *
 * D13                                  Input         |                                                       *
 * ========================================================================================================== *
 *                                                    |                                                       *
 *                                                    |                                                       *
 *         Motor Output Pin Assignment                |                                                       *
 * PIN     Function                  Input/Output     |                                                       *
 * ---------------------------------------------------------------------------------------------------------- *
 * M1                                                 |                                                       *
 * M2                                                 |                                                       *
 * M3                                                 |                                                       *
 * M4                                                 |                                                       *
 **************************************************************************************************************/

#include <RobotStatus.h>
#include "Robot.h"
#include "Interruptions.h"
#include <phys253.h> //Problematic, can only be included once
#include <LiquidCrystal.h>
#include "Menu.h"


//>>>======= Define Constants ========<<<
// MODE 
 #define LOST          0
 #define TAPE_FOLLOWER 1
 #define GRABBING      2  
 #define IR_NAVIGATOR  3
 #define ESCAPING      4
 #define IDLE_MODE     5
 
//Competition Mode
#define FULL_COURSE    0
#define HALF_COURSE    1

//Course status
#define WHITE_COURSE   0
#define RED_COURSE     1
 
//Menu item index (important)
  #define SPEED               0 
  #define TF_P_VALUE          1
  #define TF_D_VALUE          2
  #define LEFT_TF_THRESHOLD   3
  #define RIGHT_TF_THRESHOLD  4
  #define IR_P_VALUE          5
  #define IR_D_VALUE          6
  #define IR_SENSOR_DIFFERENCE  7
  #define MARK_THRESHOLD      8
  
  
//MOTOR (not tobe changed EXTREME CAUTIONS)
 extern const int  LEFT_MOTOR  =  1 ;
 extern const int RIGHT_MOTOR  =  2 ;
//ANALOG
 extern const int FT_RIGHT_QRD =  1;//6
 extern const int FT_LEFT_QRD  =  2;//5
 
 extern const int IR_F_LEFT    =  4;//3
 extern const int IR_F_RIGHT   =  5;//2
 extern const int DOLL_MARKER  =  0;//7
//Digital Pin:
//input:

extern const int DOLL_PICKED      = 8;
extern const int EX_SWITCH        = 7;
extern const int DIGI_DOLL_MARKER = 2;
extern const int SIXTH_STOPPER    = 4;



//Not to be uploaded(Blocker Bug)

//>>>======== TINAH MODES ============<<<<<<<  [CONTROL CENTER]




 
 //When define competition. undefine all others : including MENU/TIMETRACK/MANUAL_MODE
 //#define COMPETITION 
 
 #define TEST
  
     #ifdef TEST
         
          //#define digitalTEST  // passed
          //#define getModeTEST  // passed
          //#define changeModeTEST // passed
          //#define TapeFollowerTEST
          //#define MenuReturnValTEST // passed
          //#define externObjTEST //passed
          //#define InterruptionTEST
          
          //-------Tape Follower TESTS-------
          //#define TapeFollowerStopperTEST
         #define TapeFollowerToIRNavigatorTEST
     #endif  
  
  #define MANUAL_MODE   
  #define MENU      
  #define TIMETRACK
  
  
  



//>>>======== Initialize Variables ======<<<
  int ti;//time interval
  int t;//current time
  int c = 0;
  
  
 #ifndef COMPETITION
  //          INDEX:        0,      1,       2,      3,       4,       5,       6 
  String ManualMenu[] = {"LOST","T-Follow","Grab","IR Nav","Escape","Thresh","Compete"};
#endif

#ifdef COMPETITION
 //          INDEX:        0,               1,       2,    3,     4
  String ManualMenu[] = {"White Cour","Red Cour","Thresh","W/B","R/B"};
#endif
  

//Initalizing robot:
 
 RobotStatus psychobot = RobotStatus((byte)5);
//extern boolean grabFlag = false;
 
 
 
//
  void setup() {
    
    Serial.begin(9600);
    #include <phys253setup.txt>
    ti = 0;//time interval for time track
    t = 0;//time read at 
    
    //RCServo initial position:
    RCServo1.write(165);//165
    RCServo2.write(100);//140
    RCServo0.write(0); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    //Interrupts:
    //enableExternalInterrupt(INT2, FALLING); // doll marker interruption
    
    //Digital Pin initialization:

    pinMode(DOLL_PICKED,INPUT);
    pinMode(EX_SWITCH,INPUT);
    pinMode(SIXTH_STOPPER,INPUT);
    
    pinMode(DOLL_MARKER,INPUT);
    
    
    
    
    #ifdef TEST
    
      #ifdef digitalTEST
       pinMode(7,INPUT);
      #endif
      
    #endif//TEST
   
  
  }
  
  void loop() {
  
     #ifdef TIMETRACK //July2
       ti = millis() - t;
       t = millis();
       Serial.print(ti);Serial.println(" ms");
     #endif //timetrack
  
    
     #ifdef MENU //June 2   
      	LCD.clear(); LCD.home();
  	LCD.print("start for Menu");
  	delay(100);
   
  	if (startbutton()){
  		delay(100);
  		if (startbutton())
  		{
  			Menu();
  		}
  	}
    #endif //MENU
    
    #ifdef MANUAL_MODE
    
     #ifndef COMPETITION
    int MMid = knob(7)/146;
    LCD.setCursor(0,1);
     
     LCD.print("SetMode ");LCD.print(ManualMenu[MMid]);
     delay(100);
     
     if (stopbutton() || digitalRead(EX_SWITCH) == LOW){
         delay(50);
         if (stopbutton() || digitalRead(EX_SWITCH) == LOW){
          //MMid < 5 : setting mode
             if(MMid < 5){
               //reset to clear memories.
               psychobot.resetRobot();
               
               psychobot.changeModeTo(MMid);
               delay(300);
             }
          //MMid = 5 : threshold checking
          else if(MMid == 5){
               while(!startbutton()){
                 LCD.clear();LCD.home();
                 //QRD Threshold test
                 if(knob(7) < 170){
                   LCD.setCursor(0,0); LCD.print("L-QRD:");LCD.print(analogRead(FT_LEFT_QRD));
                   LCD.setCursor(0,1); LCD.print("R-QRD:");LCD.print(analogRead(FT_RIGHT_QRD)); 
                   
                 }
                 //IR Threshold test
                 else if(knob(7) > 170 && knob(7) < 2*170){
                   LCD.setCursor(0,0); LCD.print("L-IR:");LCD.print(analogRead(IR_F_LEFT));
                   LCD.setCursor(0,1); LCD.print("R-IR:");LCD.print(analogRead(IR_F_RIGHT));
                   LCD.print(" ");LCD.print(analogRead(IR_F_LEFT)-analogRead(IR_F_RIGHT));
                 }
                 //DIGI_DOLL_MARKER
                 else if(knob(7) > 2*170 && knob(7) < 3*170) {LCD.print("Digi-Mark: "); LCD.print(digitalRead(DIGI_DOLL_MARKER));}
                  
                 //DOLL_MARKER
                 else if(knob(7) > 3*170 && knob(7) < 4*170){LCD.print("Marker: ");LCD.print(analogRead(DOLL_MARKER));}
                  
                 //SIXTH_SWITCH
                 else if(knob(7) > 4*170 && knob(7) < 5*170) {LCD.print("sixth switch: "); LCD.print(digitalRead(SIXTH_STOPPER));}  
                 
                 //DOLL_PICKED
                 else {LCD.print("DOLL_PICKED: "); LCD.print(digitalRead(DOLL_PICKED));}
                 delay(100);
                 
               }
          }
          //competition mode: full run/ half run
          else if(MMid == 6){
            
             while(!startbutton()){
               LCD.clear();LCD.home();
               LCD.print("compMode: ");
               if(psychobot.GetCompetitionMode()==0) LCD.print("Full");
               if(psychobot.GetCompetitionMode()==1) LCD.print("Half");
               LCD.setCursor(0,1);
               LCD.print("Set To: ");
               delay(100);
               if(knob(7) < 500)       LCD.print("Full");
               else if(knob(7) >= 500) LCD.print("Half");
               delay(100);
               
               if(knob(7) < 500){
                  if (stopbutton() || digitalRead(EX_SWITCH) == LOW){
                    delay(500);
                    psychobot.resetRobot();
                    psychobot.SaveCompetitionMode(0);
                    delay(200);
                  }
               }
               if(knob(7) > 500){
                  if (stopbutton() || digitalRead(EX_SWITCH) == LOW){
                    delay(500);
                    psychobot.resetRobot();
                    psychobot.SaveCompetitionMode(1);
                    delay(200);
                  }
               }
               
             }
          }
        }
     }
    #endif //not competition
    //==================   COMPETITION MENU =======================
    #ifdef COMPETITION
    int MMid = knob(7)/204;
    LCD.setCursor(0,1);
     
     LCD.print("SetMode ");LCD.print(ManualMenu[MMid]);
     delay(100);
     
      if (stopbutton() || digitalRead(EX_SWITCH) == LOW){
         delay(50);
         if (stopbutton() || digitalRead(EX_SWITCH) == LOW){
          //MMid = 0 : white
             if(MMid == 0){
               //reset to clear memories.
               psychobot.resetRobot();
               psychobot.SetCourseTo(WHITE_COURSE);
               psychobot.SaveCompetitionMode(FULL_COURSE); 
               psychobot.changeModeTo(TAPE_FOLLOWER);       
               delay(300);
             }
          //MMid = 1 : red
             else if(MMid == 1){
               psychobot.resetRobot();
               psychobot.SetCourseTo(RED_COURSE);
               psychobot.SaveCompetitionMode(FULL_COURSE); 
               psychobot.changeModeTo(TAPE_FOLLOWER);    
               delay(300);
             }   
             //MMid = 2: test Threshold 
             else if(MMid == 2){
                while(!startbutton()){
                 LCD.clear();LCD.home();
                 //QRD Threshold test
                 if(knob(7) < 170){
                   LCD.setCursor(0,0); LCD.print("L-QRD:");LCD.print(analogRead(FT_LEFT_QRD));
                   LCD.setCursor(0,1); LCD.print("R-QRD:");LCD.print(analogRead(FT_RIGHT_QRD)); 
                   
                 }
                 //IR Threshold test
                 else if(knob(7) > 170 && knob(7) < 2*170){
                   LCD.setCursor(0,0); LCD.print("L-IR:");LCD.print(analogRead(IR_F_LEFT));
                   LCD.setCursor(0,1); LCD.print("R-IR:");LCD.print(analogRead(IR_F_RIGHT));
                   LCD.print(" ");LCD.print(analogRead(IR_F_LEFT)-analogRead(IR_F_RIGHT));
                 }
                 //DIGI_DOLL_MARKER
                 else if(knob(7) > 2*170 && knob(7) < 3*170) {LCD.print("Digi-Mark: "); LCD.print(digitalRead(DIGI_DOLL_MARKER));}
                  
                 //DOLL_MARKER
                 else if(knob(7) > 3*170 && knob(7) < 4*170){LCD.print("Marker: ");LCD.print(analogRead(DOLL_MARKER));}
                  
                 //SIXTH_SWITCH
                 else if(knob(7) > 4*170 && knob(7) < 5*170) {LCD.print("sixth switch: "); LCD.print(digitalRead(SIXTH_STOPPER));}  
                 
                 //DOLL_PICKED
                 else {LCD.print("DOLL_PICKED: "); LCD.print(digitalRead(DOLL_PICKED));}
                 delay(100);
                 
               }
             }
             //id = 3: white back up
             else if(MMid == 3){
               psychobot.resetRobot();
               psychobot.SetCourseTo(WHITE_COURSE);
               psychobot.SaveCompetitionMode(HALF_COURSE); 
               psychobot.changeModeTo(TAPE_FOLLOWER); 
               delay(300); 
             }
             //id = 4: red back up
             else if(MMid == 4){
               psychobot.resetRobot();
               psychobot.SetCourseTo(RED_COURSE);
               psychobot.SaveCompetitionMode(HALF_COURSE); 
               psychobot.changeModeTo(TAPE_FOLLOWER); 
               delay(300); 
             }
         }
      }
     
    #endif   
      
    #endif //Manual Mode
    
   //>>>>>>==================== GENERAL TEST========================<<<<<<<<
    
    #ifdef TEST
    
      #ifdef digitalTEST //July 3 
        Serial.println(digitalRead(7));
        LCD.clear();LCD.home();
        LCD.print(digitalRead(7));
        delay(300);
      #endif //digitalTEST
      
    //getMode ------------------------------------------------  [PASSED]  on 6.26
      #ifdef getModeTEST
        byte currentMode = psychobot.getMode();//test getMode(),
        Serial.println(currentMode);
        LCD.clear();LCD.home();
        LCD.print(currentMode);
      #endif //getModeTEST
    
    //changeMode --------------------------------------------   [PASSED] on 6.26
      #ifdef changeModeTEST
        int Mode = knob(7)/102;//knob input was divide into 10 modes, 
                               //which is not going to happen. we are 
                               //here to check rep invariant protection
        boolean okay = psychobot.changeModeTo(Mode); // Test Passed
        byte currentMode = psychobot.getMode();
        
        Serial.print(okay);Serial.print(" ");Serial.println(currentMode);
        LCD.clear();LCD.home();
        if(okay == true)  LCD.print("change successful");
        else LCD.print("change not successful");
      #endif //change modeTEST
      
   //access values changed in menu -----------------------   [PASSED] on 7.2 Notes: value only change when STOP is been
      #ifdef MenuReturnValTEST                                               //     pressed b/c menu is blocking rest program   
        Serial.println(returnValue(TF_THRESHOLD));
      #endif//MenuReturnValTEST
      
      #ifdef externObjTEST
        Serial.print(psychobot.getMode());Serial.print(" ");
        psychobot.changeModeTo((byte)5);
        Serial.println(psychobot.getMode());
      #endif //externObjTEST
      
      #ifdef InterruptionTEST
        
      #endif
      
   //>>>>>>========================= TapeFollower test ===========================<<<<<
      #ifdef TapeFollowerStopperTEST
      
      if(psychobot.getMode() == TAPE_FOLLOWER)
      
          byte modeCheck = TapeFollower(returnValue(SPEED),returnValue(TF_P_VALUE),returnValue(TF_D_VALUE),
                                      returnValue(LEFT_TF_THRESHOLD),returnValue(RIGHT_TF_THRESHOLD));
      if(psychobot.getMode() == IDLE_MODE)
         
          byte modeCheck = Idling();
           
       if(psychobot.getMode()== GRABBING) 
         
         byte modeCheck = Grabbing();
          
       Serial.println(psychobot.getMode());
       //LCD.setCursor(0,1);
       //LCD.print("Mode ");LCD.print(psychobot.getMode());
       delay(100);
       
      #endif // tapefollowerStopperTEST 
      
      #ifdef TapeFollowerToIRNavigatorTEST
      
      byte modeCheck = 233;
      
      if(psychobot.getMode() == TAPE_FOLLOWER)
      
            modeCheck = TapeFollower(returnValue(SPEED),returnValue(TF_P_VALUE),returnValue(TF_D_VALUE),
                                      returnValue(LEFT_TF_THRESHOLD),returnValue(RIGHT_TF_THRESHOLD),returnValue(MARK_THRESHOLD));
      else if(psychobot.getMode() == IDLE_MODE)
         
            modeCheck = Idling();
           
      else if(psychobot.getMode()== GRABBING) 
         
            modeCheck = Grabbing();
            
      else if(psychobot.getMode() == ESCAPING)
            
            modeCheck = Escape(returnValue(SPEED),returnValue(LEFT_TF_THRESHOLD),returnValue(RIGHT_TF_THRESHOLD));
     
            
      else if(psychobot.getMode() == IR_NAVIGATOR)
            
            modeCheck = IRNavigator(returnValue(SPEED),returnValue(IR_P_VALUE),returnValue(IR_D_VALUE),
                                      returnValue(IR_SENSOR_DIFFERENCE),returnValue(MARK_THRESHOLD));
                                      
     // else if(psychobot.getMode() == LOST)
    
            //modeCheck = Lost(returnValue(SPEED));
           
      
      Serial.println(modeCheck);
      
      #endif //TapeFollower to IR Navigator test
     
     
      
    #endif//TEST
    
    
    
    
    //>>>>================== COMPETITION ROUTINE ======================<<<<<<
    
    #ifdef COMPETITION
    byte modeCheck = 0;
    // FULL COURSE
    if(psychobot.GetCompetitionMode() == FULL_COURSE){
      
           if(psychobot.getMode() == TAPE_FOLLOWER)
            
                  modeCheck = TapeFollower(returnValue(SPEED),returnValue(TF_P_VALUE),returnValue(TF_D_VALUE),
                                            returnValue(LEFT_TF_THRESHOLD),returnValue(RIGHT_TF_THRESHOLD),returnValue(MARK_THRESHOLD));
                                            
           else if(psychobot.getMode()== GRABBING) 
               
                  modeCheck = Grabbing();
           
           else if(psychobot.getMode() == IR_NAVIGATOR)
                  
                  modeCheck = IRNavigator(returnValue(SPEED),returnValue(IR_P_VALUE),returnValue(IR_D_VALUE),
                                            returnValue(IR_SENSOR_DIFFERENCE),returnValue(MARK_THRESHOLD));
                                            
           else if(psychobot.getMode() == ESCAPING)
                  
                  modeCheck = Escape(returnValue(SPEED),returnValue(LEFT_TF_THRESHOLD),returnValue(RIGHT_TF_THRESHOLD));
           
           else if(psychobot.getMode() == IDLE_MODE)
                   
                  modeCheck = Idling();
           
          else if(psychobot.getMode() == LOST)
          
                  modeCheck = Lost(returnValue(SPEED),returnValue(LEFT_TF_THRESHOLD),returnValue(RIGHT_TF_THRESHOLD));
    }   
    // HALF COURSE
   else if(psychobot.GetCompetitionMode() == HALF_COURSE)  {
         
          if(psychobot.getMode() == TAPE_FOLLOWER)
                  
                  modeCheck = HalfTapeFollower(returnValue(SPEED),returnValue(TF_P_VALUE),returnValue(TF_D_VALUE),
                                            returnValue(LEFT_TF_THRESHOLD),returnValue(RIGHT_TF_THRESHOLD),returnValue(MARK_THRESHOLD));
          
          else if(psychobot.getMode() == GRABBING)
          
                 modeCheck = Grabbing();
                 
          else if(psychobot.getMode() == LOST)
                 
                 modeCheck = HalfEscape(returnValue(SPEED),returnValue(LEFT_TF_THRESHOLD),returnValue(RIGHT_TF_THRESHOLD));
                 
          else if(psychobot.getMode() == IDLE_MODE)
                  
                 modeCheck = Idling();
   }
   
   
    
            
    #endif // Competition
    

  }


