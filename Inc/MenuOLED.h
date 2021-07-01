/*
 * MenuOLED.h
 *
 *  Created on: Jan 30, 2021
 *      Author: LeeSeng
 */

#ifndef MENUOLED_H_
#define MENUOLED_H_


#include <MPU6050_HMC5883L_ITC.h>
#include "stdio.h"
#include "BMP180.h"

//home screen
void homeINFO();

//Check MPU6050 Angle
char angleXOLED[20], angleYOLED[20], angleZOLED[20];
//Read the Altitude
char headingOLED[20], AltitudeOLED[20];

void homeINFO()
{
	SSD1306_Fill (0);
	SSD1306_GotoXY(10, 0);
	SSD1306_Puts("DRONE INFO", &Font_11x18, 1);
	sprintf(angleXOLED, "%.1f", getAngleX());
	sprintf(angleYOLED, "%.1f", getAngleY());
	sprintf(angleZOLED, "%.1f", getAngleZ());
	sprintf(AltitudeOLED, "%.1f", BMP180_GetAlt(0));
	sprintf(headingOLED, "%d", HMC5883L_getCompass());
	SSD1306_GotoXY(90, 20);
	SSD1306_Puts(AltitudeOLED, &Font_7x10, 1);
	SSD1306_GotoXY(95, 32);
	SSD1306_Puts(headingOLED, &Font_7x10, 1);
	SSD1306_GotoXY(20, 20);
	SSD1306_Puts(angleXOLED, &Font_7x10, 1);
	SSD1306_GotoXY(20, 32);
	SSD1306_Puts(angleYOLED, &Font_7x10, 1);
	SSD1306_GotoXY(20, 44);
	SSD1306_Puts(angleZOLED, &Font_7x10, 1);
	SSD1306_GotoXY(0, 20);
	SSD1306_Puts("R:", &Font_7x10, 1);
	SSD1306_GotoXY(60, 20);
	SSD1306_Puts("Alt:", &Font_7x10, 1);
	SSD1306_GotoXY(0, 32);
	SSD1306_Puts("P:", &Font_7x10, 1);
	SSD1306_GotoXY(60, 32);
	SSD1306_Puts("Head:", &Font_7x10, 1);
	SSD1306_GotoXY(0, 44);
	SSD1306_Puts("Y:", &Font_7x10, 1);
	SSD1306_GotoXY(60, 44);
	SSD1306_Puts("Sys:GOOD", &Font_7x10, 1);
	SSD1306_UpdateScreen(); //display
}
#endif /* MENUOLED_H_ */
