/* This is the main header of the program, used to set different Competition/
 * Testing Scenarios
 * Author: Ziyue HU
 * Date: June 16
 */
 #ifndef ROBOT_ON
 #define ROBOT_ON
 
 /* Library */
 #include <RobotStatus.h>

 
 byte TapeFollower(int,int,int,int,int,int);
 
 
 byte IRNavigator(int,int,int,int,int);
 

 byte Grabbing(void);
 
 
 byte Escape(int,int,int);
 

 byte Idling(void);
 
 
 byte Lost(int,int,int);
 
 
 //------- for when we do not have time for fire wall--------//
 
 byte HalfTapeFollower(int,int,int,int,int,int);
 
 byte HalfEscape(int,int,int);
 
 #endif
