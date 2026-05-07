/****************************************************************************************************************************************
*****************************************************************************************************************************************
**************************************************    Date     :  11/15/2025			*************************************************
**************************************************    Name     :   Mario Emad Boles 	*************************************************
**************************************************    Version  :   1.0	            	*************************************************
**************************************************    SWC      :  NEMA23_DRIVER	        *************************************************
*****************************************************************************************************************************************
*****************************************************************************************************************************************
*/
//LIBS WE NEED FOR ESP32 FIRMWARE
#include <Arduino.h>
#include "DRI_interface.hpp"

void DRI_init(void)
{
	//SET THE PINS TO OUTPUT	
	pinMode(DIR_PIN,OUTPUT);
	pinMode(PUL_PIN,OUTPUT);
}

void DRI_riseup(void)//rise up
{

	//FIRST LETS SET THE DIRECTION TO CLOCKWISE
	digitalWrite(DIR_PIN,HIGH);//CLK WISE
	delayMicroseconds(10);//must to wait a 5 micro seconds for the driver to understand
	DRI_PWM();

} 

void DRI_falldown(void)// fall down
{
	

	//FIRST LETS SET THE DIRECTION TO CONTERCLOCKWISE
	digitalWrite(DIR_PIN,LOW);//CONTERCLK WISE
	delayMicroseconds(10);//must to wait a 5 micro seconds for the driver to understand
	DRI_PWM();

}


void DRI_PWM(void)
{
	
	int moving_steps = 3;


	for (int i = 0 ; i < moving_steps; i++)
	{
		int check=0;

		while(DRI_STEPS>check)//this loop will run in the amount of the steps number
		{
			//LOW THE POWERSCREW

			digitalWrite(PUL_PIN,HIGH);
			delayMicroseconds(DRI_ON_TIME);
			digitalWrite(PUL_PIN,LOW);
			delayMicroseconds(DRI_OFF_TIME);
			check++;

			vTaskDelay(2 / portTICK_PERIOD_MS);//change this number from 2000 --> 4000
		}
	}
}
