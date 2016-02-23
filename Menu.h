/* This class is for menu management
 * [IMPORTANT] not to be included in any file other than "ENPHRobot.ino"
 */
 
 #pragma once
 
 #include <avr/EEPROM.h>
 #include "Phys.h"
 #include <arduino.h>
 #include <RobotStatus.h>

 class MenuItem
{
public:
	String    Name;
	uint16_t  Value;
	uint16_t* EEPROMAddress;
	static uint16_t MenuItemCount;
	MenuItem(String name)
	{
		MenuItemCount++;
		EEPROMAddress = (uint16_t*)(2 * MenuItemCount);
		Name 		  = name;
		Value         = eeprom_read_word(EEPROMAddress);
	}
	void Save()
	{
		eeprom_write_word(EEPROMAddress, Value);
	}
};

uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem Speed            = MenuItem("Speed");//            0   ITEM INDEX 
MenuItem TFProportionalGain = MenuItem("TFP-gain");//       1
MenuItem TFDerivativeGain   = MenuItem("TFD-gain");//       2
//MenuItem IntegralGain     = MenuItem("I-gain");//      
MenuItem L_TFThreshold      = MenuItem("L-TF-Thresh");//    3
MenuItem R_TFThreshold      = MenuItem("R-TF-Thresh");//    4
MenuItem IRProportionalGain = MenuItem("IRP-gain");//       5
MenuItem IRDerivativeGain   = MenuItem("IRD-gain");//       6
MenuItem Sensor_Diff        = MenuItem("IR_SD");//           7
MenuItem Mark_Threshold     = MenuItem("Marker");//8
                //id number:   0            1                2                3                 4                5                6          
MenuItem menuItems[]      = {Speed, TFProportionalGain, TFDerivativeGain,L_TFThreshold,R_TFThreshold,IRProportionalGain,IRDerivativeGain,
               //                    7                 
                              Sensor_Diff,Mark_Threshold};

void Menu()
{
	LCD.clear(); LCD.home();
	LCD.print("Entering menu");
	delay(500);
 
	while (true)
	{
		/* Show MenuItem value and knob value */
		int menuIndex = knob(6) * (MenuItem::MenuItemCount) / 1024;
		LCD.clear(); LCD.home();
		LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
		LCD.setCursor(0, 1);
		LCD.print("Set to "); LCD.print(knob(7)/2); LCD.print("?");
		delay(100);
 
		/* Press start button to save the new value */
		if (startbutton())
		{
			delay(100);
			if (startbutton())
			{
				menuItems[menuIndex].Value = knob(7)/2;
				menuItems[menuIndex].Save();
				delay(250);
			}
		}
 
		/* Press stop button to exit menu */
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				LCD.clear(); LCD.home();
				LCD.print("Leaving menu");
				delay(500);
				return;
			}
		}
	}
}

/** Returns the number gotten by the menu knob
 *
 */
int returnValue(int ItemIndex){
  return menuItems[ItemIndex].Value;
}
 
 
