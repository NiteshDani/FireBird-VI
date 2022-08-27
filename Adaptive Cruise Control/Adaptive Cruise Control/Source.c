// Adaptive_CC.c : Defines the entry point for the console application.
/********************************************************************************
 Written by:Tushar Chaskar, NEX Robotics Pvt. Ltd.
 Edited by: Anant Malewar, NEX Robotics Pvt. Ltd.
 version: 1.0
 Microsoft Visual Studio Express 2012 (Windows Version)
 Code blocks 13.12 (Linux version)

 Date: 2nd September 2015

 This console application demonstrates "Adaptive Cruise Control" on Fire Bird VI Robot.

 Algrithm for this application follows simple concept that as distance between object and front ultrasonic sensor increses, motor velocity of robot also increases
 Where as distance decreases velocity of robot also decreases.


 Algorithm:   (Consider that an movable plane surface object is placed infront of 2nd ultrasonic sensor of robot)
 1. Collect data of Front ultrasonic sensor(2nd ultrasonic sensor) of robot (See following representation of Ultrasonic sensor locations)
 2. Check reading of 2nd ultrasonic sensor which is a nothing but a distance between object and robot.
 3. If distance is less than "Stop_Limit" i.e. 6 then set velocity of both motor as 0 and stop the robot.
 4. else use prescaler to set velocity of both motors according to the distance between object and robot and move the robot forward till this distance decreases and reaches to Stop_Limit.
 5. As the distance between object and robot increases, velocity of robot also increases and vice versa.
 6. If output of 2nd ultrasonic sensor goes beyond 50 then motor should attend its maximum velocity till it detects an object in the span of 6 to 50. (ultrasonic sensor output ranges from 3 to 255)

 Prescaler calculation:
 Prescaler calculations for sensor reading between  6 to 50

 Prescaler = 0.635/50 = 0.0127
 Where,
 0.635 is the maximum forward velocity of robot in meters/sec
 50 is output value of 2nd ultrasonic sharp sensor.

 example:
 If output o 2nd ultrasonic sensor is 30 then velocity of both motors is as follows:

 Velocity = 30 * 0.0127 = 0.381

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


 This application uses dynamic library functions for robot.

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
#include "FireBirdClib.h"


#define		SensorRange_MaxSpeed		50
#define		Prescaler_to_adjust_Speed	(Max_Speed/SensorRange_MaxSpeed)
#define		Stop_Limit					6
#define		Position_Mode				1																// Close loop velocity control
#define		Accelearation				10
#define		Safety_Mode					0																// 0 = Safety OFF, 1 = safety ON
#define		TimeOut						2

//#ifdef WIN32
char* ComPort = "\\\\.\\COM7";
//#else
//char* ComPort = "/dev/ttyUSB0";
//#endif
const float Max_Speed = 0.4f;

// ==============================================================================================================================
//	Function 							: open_comport()
//	Return type							: void pointer
//	Parameters							: handler
//	Description 						: This function open requested communication port for communication
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
//	Parameters							: com port handler
//	Description 						: This function will set robot motor parameters such as mode,safety and acceleration
//
//  List of library functions included	: 1. SetMode(handler,mode)
//										: 2. SetSafety(handler,safety_ON/OFF)
//										: 3. SetAcceleration(handler,acceleration)
// ==============================================================================================================================

BOOL set_motor_parameters(void* hCom)
{
	if (setMode(hCom, Position_Mode))
	{
		if (setSafety(hCom, Safety_Mode))
		{
			if (setAcceleration(hCom, Accelearation))
			{
				if (setSafetyTimeout(hCom, TimeOut))
					return TRUE;
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
//	Function 							: processing_data()
//	Return type							: BOOL
//	Parameters							: com port handler
//	Description 						: This function will check the data from ultrasonic sensor and accordingly it will set the velocity of robot for forward direction
//
//  List of library functions included	: 1. getSonar(handler,sensor number,data buffer)
//										: 2. SetLinearVelocity_meterspersec(handler,velocity in m/Sec)
//										: 3. forward(handler)
//										: 4. stop(handler)
// ==============================================================================================================================

BOOL processing_data(void* hCom)
{
	uint8 Front_sensor;
	float velocity_mtrpersec;

	if (getSonar(hCom, 2, &Front_sensor))
	{
		//printf("ultrasonic data = %d\n",Front_sensor);													// uncommenting this line will print sensor data
		if (Front_sensor < Stop_Limit)																		// If ultrsonic data is less than Stop_Limit then stop the robot
		{
			if (setLinearVelocity_meterspersec(hCom, 0))
			{
				if (stop(hCom));
			}
		}
		else
		{
			velocity_mtrpersec = (float)((float)Front_sensor * Prescaler_to_adjust_Speed);						// 50 is limit from ultrasonic sensor. (0.4/50 = 0.008)
																												// at ultrasonic data > 50, robot follows maximum speed
			if (setLinearVelocity_meterspersec(hCom, velocity_mtrpersec))
			{
				if (forward(hCom));
			}
		}
	}
	return TRUE;
}

// ==============================================================================================================================
//	Function 							: main()
//	Return type							: int
//	Parameters							: none
//	Description 						: Main Function
//  List of library functions included	: 1. setSonarSensorON(handler)
//										: 2. disconnect_comm(handler)
// ==============================================================================================================================

int main()
{
	void* hCom;

	hCom = open_comport(ComPort);

	if (hCom != 0)
	{
		if (set_motor_parameters(hCom));
		else
		{
			disconnect_comm(hCom);
			printf(" set_motor_parameters fail \n");
			return FALSE;
		}
		if (setSonarSensorON(hCom));
		else
		{
			disconnect_comm(hCom);
			printf(" setSonarSensorON fail \n");
			return FALSE;
		}

		while (TRUE)
		{
			if (processing_data(hCom));
		}
	}
	else
	{
		disconnect_comm(hCom);
		printf("\n CAN NOT ESTABLISH COMMUNICATION \n");
		return FALSE;
	}

}
