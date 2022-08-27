// wal_folow.c : Defines the entry point for the console application.
/********************************************************************************
 Written by:Tushar Chaskar, NEX Robotics Pvt. Ltd.
 Edited by: Anant Malewar, NEX Robotics Pvt. Ltd.
 version: 1.0
 Microsoft Visual Studio Express 2012 (Windows Platform)
 Code Blocks 13.12 (Linux Platform)
 Date: 2nd September 2015

 This console application demonstrates "Left Wall following" on Fire Bird VI using sharp sensors of Robot.
 Robot has total 8 sharp range sensors, out of which we wre only using 5 sharp sensors, from 0 to 4

 This application uses static library functions for robot.

 Algorithm:
 1. Algorithm is designed for left wall following.
 2. Robot will continuesly try to move towards wall i.e. Left side
 3. If Front Sharp sensor(2nd) is clear (No obstacle) then check for left sharp sensor(0th).
 4. If value of 0th sharp sensor is less than 20 then move robot at right direction otherwise move left.
 5. Also check 1st sharp sensor. If it is less than 39 then move right otherwise move left.
 6. If 2nd sharp sensor detects an object, it will check both left side (0th sharp sensor) and right side (4th sharp sensor) of robot and whichever side is free robot will rotate to that side.
 7. If front sharp sensor(2nd),right sharp sensor(4th) and left sharp sensor(0th) detects an object then robot will simply rotate clockwise till its front sharp sensor detects no object.

 Communication media: wired(USB) or Wireless(RS232/WiFi/Bluetooth).

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


#define		Position_Mode				1																		// Open loop velocity control mode
#define		Accelearation				3
#define		Safety_ON					1
#define		Safety_OFF					0
#define		TimeOut						2

void* hCom;
//#ifdef WIN32
char* ComPort = "\\\\.\\COM7";																							// Communication port string
//#else
//char* ComPort = "/dev/ttyUSB0";
//#endif
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
	if (setMode(hCom, Position_Mode))
	{
		if (setSafety(hCom, Safety_OFF))
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
//	Function 							: Robot_Left()
//	Return type							: void
//	Parameters							: None
//	Description 						: This function will set robot motor velocities to move it in left direction
//
//  List of library functions included	: 1. SetVelocity_meterspersec(handler,left velocity,right velocity)
//										: 2. forward(handler)
// ==============================================================================================================================
void Robot_Left(void)
{
	if (setVelocity_meterspersec(hCom, 0.10f, 0.13f))
	{
		forward(hCom);
	}
}

// ==============================================================================================================================
//	Function 							: Robot_Right()
//	Return type							: void
//	Parameters							: None
//	Description 						: This function will set robot motor velocities to move it in right direction
//
//  List of library functions included	: 1. SetVelocity_meterspersec(handler,left velocity,right velocity)
//										: 2. forward(handler)
// ==============================================================================================================================
void Robot_Right(void)
{
	if (setVelocity_meterspersec(hCom, 0.13f, 0.10f))
	{
		forward(hCom);
	}
}

// ==============================================================================================================================
//	Function 							: Robot_Stop()
//	Return type							: void
//	Parameters							: None
//	Description 						: This function will set robot motor velocities to stop the robot
//
//  List of library functions included	: 1. SetVelocity_meterspersec(handler,left velocity,right velocity)
//										: 2. forward(handler)
// ==============================================================================================================================
void Robot_Stop(void)
{
	if (setVelocity_meterspersec(hCom, 0.00f, 0.00f))
	{
		forward(hCom);
	}
}


// ==============================================================================================================================
//	Function 							: Robot_rotate_clockwise()
//	Return type							: void
//	Parameters							: None
//	Description 						: This function will set robot motor velocities to move it in clockwise direction
//
//  List of library functions included	: 1. SetVelocity_meterspersec(handler,left velocity,right velocity)
//										: 2. forward(handler)
// ==============================================================================================================================
void Robot_rotate_clockwise(void)
{
	if (setVelocity_meterspersec(hCom, 0.20f, -0.20f))
	{
		forward(hCom);
	}
}

// ==============================================================================================================================
//	Function 							: Robot_rotate_anticlockwise()
//	Return type							: void
//	Parameters							: None
//	Description 						: This function will set robot motor velocities to move it in anticlockwise direction
//
//  List of library functions included	: 1. SetVelocity_meterspersec(handler,left velocity,right velocity)
//										: 2. forward(handler)
// ==============================================================================================================================
void Robot_rotate_anticlockwise(void)
{
	if (setVelocity_meterspersec(hCom, -0.20f, 0.20f))
	{
		forward(hCom);
	}
}

// ==============================================================================================================================
//	Function 							: main()
//	Return type							: int
//	Parameters							: none
//	Description 						: Main Function
//
//  List of library functions included	: 1. setIRProximitySensorON(handler)
//										: 2. disconnect_comm(handler)
//										: 3. SetVelocity_meterspersec(handler,left velocity,right velocity)
//										: 4. getIRProximitySensorArray(handler,	array buffer)
// ==============================================================================================================================

int main()
{
	uint8 sharp_array[8], i;
	uint8 sharp[8];

	hCom = open_comport(ComPort);																// Open communication com port

	if (hCom != 0)
	{
		if (set_motor_parameters(hCom));															// Set motor parameters like acceleration, safety control and mode
		else
		{
			printf("\n set_motor_parameters fail \n");
			disconnect_comm(hCom);
			return FALSE;
		}
		if (setIRDistanceSensorON(hCom));														// Turn ON sharp sensor's of robot
		else
		{
			printf("\n setIRProximitySensorON fail \n");
			disconnect_comm(hCom);
			return FALSE;
		}
		if (setVelocity_meterspersec(hCom, 0.00f, 0.00f));											// Set initial velocities of left and right motors
		else
		{
			printf("\n SetVelocity_meterspersec fail \n");
			disconnect_comm(hCom);
			return FALSE;
		}

		//DelaymSec(hCom,3000);

		while (TRUE)
		{
			if (getIRDistanceSensorArray(hCom, sharp_array))
			{
				for (i = 0; i < 5; i++)
				{
					sharp[i] = (int)((2799.6 * (1.00 / (pow(sharp_array[i], 1.1546)))));					// Convert raw data from sharp to cm
					if (sharp[i] > 80)																// for 10 - 80 cm sharp, limit eange to 80cm
						sharp[i] = 80;
				}
				Collect_data_into_variable(hCom, (uint8*)sharp);
				if (Front_Sensor >= 45)																// No obstacle on front sharp
				{
					if (Left_Sensor < 20 || Front_Left_Sensor < 36)									// moving clos er to the wall
					{
						Robot_Right();																// Rotate robot right
					}
					else																			// Going away from wall
					{
						Robot_Left();																// Rotate robot left
					}
				}
				else																				// Obstacle detected at front sharp
				{
					while (Front_Sensor < 45)														// stay in while loop till fornt sharp sensor becomes obstacle free
					{
						if (getIRDistanceSensorArray(hCom, sharp_array))								// Collect data from sharp ranger to get latest data of sharp sensors
						{
							for (i = 0; i < 5; i++)
							{
								sharp[i] = (int)((2799.6 * (1.00 / (pow(sharp_array[i], 1.1546)))));		// Convert raw data from sharp to cm
								if (sharp[i] > 80)													// for 10 - 80 cm sharp, limit eange to 80cm
									sharp[i] = 80;
							}
						}

						Collect_data_into_variable(hCom, (uint8*)sharp);
						if (Right_Sensor > 50)														// check right side of robot to move right
						{
							Robot_rotate_clockwise();
						}
						if (Left_Sensor > 50)														// Check left side of robot to move left
							Robot_rotate_anticlockwise();
						if (Left_Sensor < 30 && Right_Sensor < 30)									// Stop the robot is both side has obstacle	and move clockwise
						{
							Robot_Stop();
							Robot_rotate_clockwise();
							break;
						}
					}
				}
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
