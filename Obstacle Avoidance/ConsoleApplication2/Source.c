// obstacle_avoid.c : Defines the entry point for the console application.
/********************************************************************************
 Written by:Tushar Chaskar, NEX Robotics Pvt. Ltd.
 Edited by: Anant Malewar, NEX Robotics Pvt. Ltd.
 version: 1.0
 Microsoft Visual Studio Express 2012 (Windows Platform)
 Code blocks 13.12 (Linux Platform)

 Date: 3rd September 2015

 This console application demonstrates "Obstacle Avoidance" on Fire Bird VI Robot.
 This application uses all 8 ultrasonic sensors of robot for obstacle detection.



A representation of sensor locations


					  Front

		Front			2			Front
		Left			|			Right
				1 	  _____     3
				 \  /       \  /
				   /         \
Left		 0 __ |			  | __ 4		Right
				  |           |
				   \         /
				 /	\ _____ /  \
				7               5
						|
		Back			6			Back
		Left						Right

					  Back


 This application uses static library functions for robot.

 Communication media: wired(USB) or Wireless(RS232/WiFi/Bluetooth).

********************************************************************************

   Copyright (c) 2015, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)

*********************************************************************************/

#define _CRT_SECURE_NO_WARNINGS
#ifdef WIN32
#include <windows.h>
#endif // WIN32
#include <stdio.h>
#include <math.h>
#include "FireBirdClib.h"

#define		Position_Mode				1																	// close loop control mode
#define		Accelearation				6
#define		Safety_ON					1
#define		Safety_OFF					0
#define		Sensor_range_limit			6																	// For which sensor will detect an object and reaspond
#define		TimeOut						2

void* hCom;
//#ifdef WIN32
char* ComPort = "\\\\.\\COM7";
//#else
//char* ComPort = "/dev/ttyUSB0";
//#endif
uint8 sonar[8];
uint8 Front_Sensor, Left_Sensor, Right_Sensor, Back_Sensor, Front_Left_Sensor, Front_Right_Sensor, Back_Left_Sensor, Back_Right_Sensor;


// ==============================================================================================================================
//	Function 							: open_comport()
//	Return type							: void pointer
//	Parameters							: PORT name
//	Description 						: This function open requested com port for communication
//
//  List of library functions included	: connect_comm(comport)
// ==============================================================================================================================

void* open_comport(char* Com)
{
	return connect_comm(Com);
}

// ==============================================================================================================================
//	Function 							: set_motor_parameters()
//	Return type							: BOOL
//	Parameters							: communication handler
//	Description 						: This function will set robot motor parameters such as (mode,safety and acceleration)
//
//  List of library functions included	: 1. SetMode(handler,mode)
//										: 2. SetSafety(handler,safety_ON/OFF)
//										: 3. SetAcceleration(handler,acceleration)
// ==============================================================================================================================

BOOL set_motor_parameters(void* hCom)
{
	if (setSafety(hCom, Safety_OFF))
	{
		if (setMode(hCom, Position_Mode))
		{
			if (setAcceleration(hCom, Accelearation))
			{
				if (setSafetyTimeout(hCom, TimeOut))
				{
					return TRUE;
				}
				else
					return FALSE;
			}
			else
				return FALSE;
		}
		else
			return FALSE;
	}
	else
		return FALSE;
}

// ==============================================================================================================================
//	Function 							: Collect_data_into_variable()
//	Return type							: void
//	Parameters							: communication handler, data buffer which collects sensor's data
//	Description 						: This function will just copy collected sensor's data to appropriate variables
//
// ==============================================================================================================================

void Collect_data_into_variable(void* hCom, uint8* data)
{
	Left_Sensor = data[0];
	Front_Left_Sensor = data[1];
	Front_Sensor = data[2];
	Front_Right_Sensor = data[3];
	Right_Sensor = data[4];
	Back_Right_Sensor = data[5];
	Back_Sensor = data[6];
	Back_Left_Sensor = data[7];
}


// ==============================================================================================================================
//	Function 							: main()
//	Return type							: int
//	Parameters							: none
//	Description 						: Main Function
//  List of library functions included	: 1. setSonarSensorON(handler)
//										: 2. disconnect_comm(handler)
//										: 3. getSonarSensorArray(handler)
//										: 4. SetVelocity_meterspersec(handler,left velocity,right velocity)
//										: 5. forward(handler)
// ==============================================================================================================================

int main()
{
	hCom = open_comport(ComPort);																								// Open Communication port

	if (hCom != 0)
	{
		if (setSonarSensorON(hCom));																								// Turn ON Sonar Sensors
		else
		{
			disconnect_comm(hCom);
			printf(" setSonarSensorON fail\n");
			return FALSE;
		}
		if (set_motor_parameters(hCom));																							// Set motor parameters like (mode, acceleration and safety control)
		else
		{
			disconnect_comm(hCom);
			printf(" set_motor_parameters fail\n");
			return FALSE;
		}

		while (TRUE)
		{
			if (getSonarSensorArray(hCom, sonar))
			{
				Collect_data_into_variable(hCom, sonar);

				if (Back_Sensor > Sensor_range_limit)
				{
					setVelocity_meterspersec(hCom, -0.10f, -0.15f);
				}

				if (Front_Left_Sensor > Sensor_range_limit)
				{
					setVelocity_meterspersec(hCom, -0.09f, 0.09f);
				}

				if (Front_Right_Sensor > Sensor_range_limit)
				{
					setVelocity_meterspersec(hCom, 0.09f, -0.09f);
				}

				if (Front_Sensor > Sensor_range_limit && Front_Left_Sensor > Sensor_range_limit && Front_Right_Sensor > Sensor_range_limit)
				{
					setVelocity_meterspersec(hCom, 0.15f, 0.15f);
				}

				forward(hCom);
			}
		}
	}
	else
	{
		printf("\n CAN NOT ESTABLISH COMMUNICATION \n");
		disconnect_comm(hCom);
		return FALSE;
	}
}
