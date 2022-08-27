
#ifdef WIN32

#include <Windows.h>
#include <stdio.h>

#else

#include <stdio.h> /* Standard input/output definitions */
#include <string.h> /* String function definitions */
#include <fcntl.h> /* File control definitions */
#include <errno.h> /* Error number definitions */
#include <stdbool.h>
#include <sys/types.h>
typedef bool			BOOL;

#endif
#define TRUE	1
#define FALSE	0

#define		ERROR_CONNECT			1
#define		ERROR_DISCONNECT		2
#define		ERROR_TRANSMIT			3
#define		ERROR_RECEIVE			4
#define		ERROR_RECEIVE_TIMEOUT	5
#define		ERROR_CHECKSUM			6
#define		ERROR_FAILURE_RESPONSE	7
#define		ERROR_ROBOT_CONNECT		8



typedef	unsigned char	uint8;
typedef char			int8;
typedef unsigned short	uint16;
typedef short			int16;
typedef unsigned int	uint32;
typedef int				int32;
typedef unsigned long	DWORD;

uint8 Error_Status;

#ifdef WIN32

    #ifdef FIREBIRDCLIBRARY_EXPORTS
        #define FIREBIRDCLIBRARY_API __declspec(dllexport)
    #else
        #define FIREBIRDCLIBRARY_API __declspec(dllimport)
    #endif

#else

    #ifdef FIREBIRDCLIBRARY_EXPORTS
        #define FIREBIRDCLIBRARY_API
    #else
        #define FIREBIRDCLIBRARY_API

#endif

#endif // WIN32

FIREBIRDCLIBRARY_API void *connect_comm(const char* Port);
FIREBIRDCLIBRARY_API BOOL disconnect_comm(void *hSerial);
FIREBIRDCLIBRARY_API void DelaymSec(void *hSerial,DWORD Delayms);
FIREBIRDCLIBRARY_API BOOL initPeripherals(void *hSerial);
FIREBIRDCLIBRARY_API BOOL buzzerOn(void *hSerial);
FIREBIRDCLIBRARY_API BOOL buzzerOff(void *hSerial);
FIREBIRDCLIBRARY_API BOOL getIRProximitySensorArray(void *hSerial, uint8 *allIRProximityData);
FIREBIRDCLIBRARY_API BOOL getIRProximitySensor(void *hSerial,uint8 sensorNumber,uint8 *irProximityValue);
FIREBIRDCLIBRARY_API BOOL getlineSensorArray(void *hSerial,uint8 *allLineSensorData);
FIREBIRDCLIBRARY_API BOOL getlineSensor(void *hSerial,uint8 sensorNumber,uint8 *lineSensorValue);
FIREBIRDCLIBRARY_API BOOL getIRDistanceSensorArray(void *hSerial, uint8 *allIRDistanceData);
FIREBIRDCLIBRARY_API BOOL getIRDistanceSensor(void *hSerial,uint8 sensorNumber, uint8 *irdistanceValue);
FIREBIRDCLIBRARY_API BOOL getSonarSensorArray(void *hSerial,uint8 *allSonarData);
FIREBIRDCLIBRARY_API BOOL getSonar(void *hSerial, uint8 sensorNumber,uint8 *sonarValue);
FIREBIRDCLIBRARY_API BOOL getServopodSensor(void *hSerial, uint16 *servopodsensorvalue);
FIREBIRDCLIBRARY_API BOOL getBatteryVoltage(void *hSerial, uint8 *batteryVoltage);
FIREBIRDCLIBRARY_API BOOL getBatteryCurrent(void *hSerial,uint8 *batteryCurrent);
FIREBIRDCLIBRARY_API BOOL getBatteryTemprature(void *hSerial, uint8 *batteryTemprature);
FIREBIRDCLIBRARY_API BOOL getAccelerometerXYZ(void *hSerial, int16 *accelerometerXYZ);
FIREBIRDCLIBRARY_API BOOL getGyroXYZ(void *hSerial, int16 *GyroXYZ);
FIREBIRDCLIBRARY_API BOOL getMagnetometerXYZ(void *hSerial, int16 *magnetometerXYZ);
FIREBIRDCLIBRARY_API BOOL getAccelerometerX(void *hSerial, int16 *accelerationX);
FIREBIRDCLIBRARY_API BOOL getAccelerometerY(void *hSerial, int16 *accelerationY);
FIREBIRDCLIBRARY_API BOOL getAccelerometerZ(void *hSerial, int16 *accelerationZ);
FIREBIRDCLIBRARY_API BOOL getGyroX(void *hSerial, int16 *gyroX);
FIREBIRDCLIBRARY_API BOOL getGyroY(void *hSerial, int16 *gyroY);
FIREBIRDCLIBRARY_API BOOL getGyroZ(void *hSerial, int16 *gyroZ);
FIREBIRDCLIBRARY_API BOOL getMagnetometerX(void *hSerial,int16 *magnetometerX);
FIREBIRDCLIBRARY_API BOOL getMagnetometerY(void *hSerial,int16 *magnetometerY);
FIREBIRDCLIBRARY_API BOOL getMagnetometerZ(void *hSerial,int16 *magnetometerZ);
FIREBIRDCLIBRARY_API BOOL setLineSensorON(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setLineSensorOFF(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setIRProximitySensorON(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setIRProximitySensorOFF(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setIRDistanceSensorON(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setIRDistanceSensorOFF(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setSonarSensorON(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setSonarSensorOFF(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setServopodSensorON(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setServopodSensorOFF(void *hSerial);
FIREBIRDCLIBRARY_API BOOL getLineSensorStatus(void *hSerial, uint8 *status);
FIREBIRDCLIBRARY_API BOOL getIRProximitySensorStatus(void *hSerial, uint8 *status);
FIREBIRDCLIBRARY_API BOOL getIRDistanceStatus(void *hSerial, uint8 *status);
FIREBIRDCLIBRARY_API BOOL getSonarStatus(void *hSerial, uint8 *status);
FIREBIRDCLIBRARY_API BOOL getServopodSensorStatus(void *hSerial, uint8 *status);
FIREBIRDCLIBRARY_API BOOL setLeftMotorVelocity(void *hSerial, signed short int velocity);
FIREBIRDCLIBRARY_API BOOL getLeftMotorVelocity(void *hSerial, signed short int *leftMotorVelocity);
FIREBIRDCLIBRARY_API BOOL setRightMotorVelocity(void *hSerial, signed short int velocity);
FIREBIRDCLIBRARY_API BOOL getRightMotorVelocity(void *hSerial, signed short int *rightMotorVelocity);
FIREBIRDCLIBRARY_API BOOL forward(void *hSerial);
FIREBIRDCLIBRARY_API BOOL backward(void *hSerial);
FIREBIRDCLIBRARY_API BOOL right(void *hSerial);
FIREBIRDCLIBRARY_API BOOL left(void *hSerial);
FIREBIRDCLIBRARY_API BOOL stop(void *hSerial);
FIREBIRDCLIBRARY_API BOOL getRightMotorCount(void *hSerial, int32 *rightMotorCount);
FIREBIRDCLIBRARY_API BOOL getLeftMotorCount(void *hSerial, int32 *leftMotorCount);
FIREBIRDCLIBRARY_API BOOL resetMotorEncoderCount(void *hSerial);
FIREBIRDCLIBRARY_API BOOL setPosition(void *hSerial,uint32 leftposition, signed short int leftVelocity,uint32 rightposition,signed short int rightVelocity);
FIREBIRDCLIBRARY_API BOOL setAcceleration(void *hSerial, uint8 acceleration);
FIREBIRDCLIBRARY_API BOOL getAcceleration(void *hSerial, uint8 *acceleration);
FIREBIRDCLIBRARY_API BOOL setMode(void *hSerial, uint8 mode);
FIREBIRDCLIBRARY_API BOOL getMode(void *hSerial, uint8 *mode);
FIREBIRDCLIBRARY_API BOOL setSafety(void *hSerial, uint8 safety);
FIREBIRDCLIBRARY_API BOOL setLinearVelocity_meterspersec(void *hSerial, float meterPersec);
FIREBIRDCLIBRARY_API BOOL setLinearPosition(void *hSerial, float left_dist, float left_vel, float right_dist, float right_vel);
FIREBIRDCLIBRARY_API BOOL setRobotAngularPosition(void *hSerial, float radians, float radianspersec);
FIREBIRDCLIBRARY_API BOOL setVelocity_meterspersec(void *hSerial,float Leftmeterspersec, float rightmeterspersec);
FIREBIRDCLIBRARY_API BOOL setVelocity_radianspersec(void *hSerial,float leftradianspersec, float rightradianspersec);
FIREBIRDCLIBRARY_API BOOL lcdCursor(void *hSerial,uint8 row, uint8 column, uint8 *Error_status);
FIREBIRDCLIBRARY_API BOOL lcdHome(void *hSerial);
FIREBIRDCLIBRARY_API BOOL lcdClear(void *hSerial);
FIREBIRDCLIBRARY_API BOOL lcdPrint(void *hSerial, uint8 row, uint8 column, uint32 value,uint8 digits,uint8 *Error_status);
FIREBIRDCLIBRARY_API BOOL lcdWriteChar(void *hSerial, char letter);
FIREBIRDCLIBRARY_API BOOL lcdWriteString(void *hSerial, uint8 row, uint8 column,char *str,uint8 *Error_status);
FIREBIRDCLIBRARY_API BOOL setPanServo(void *hSerial, uint8 PANangle);
FIREBIRDCLIBRARY_API BOOL setTiltServo(void *hSerial, uint8 TILTtangle);
FIREBIRDCLIBRARY_API BOOL setAuxServo(void *hSerial, uint8 AUXangle);
FIREBIRDCLIBRARY_API BOOL getPanServo(void *hSerial, uint8 *PANdata);
FIREBIRDCLIBRARY_API BOOL getTiltServo(void *hSerial, uint8 *TILTdata);
FIREBIRDCLIBRARY_API BOOL getAuxServo(void *hSerial, uint8 *AUXdata);
FIREBIRDCLIBRARY_API BOOL getADC(void *hSerial, uint16 *AdcData);
FIREBIRDCLIBRARY_API BOOL setRoboticArmServo(void *hSerial, uint8 JointNO, uint8 Angle, uint8 Velocity);
FIREBIRDCLIBRARY_API BOOL setLeftMotorVelocity_meterspersec(void *hSerial, float velocity);
FIREBIRDCLIBRARY_API BOOL setRightMotorVelocity_meterspersec(void *hSerial, float velocity);
FIREBIRDCLIBRARY_API BOOL getLeftMotorVelocity_meterspersec(void *hSerial, float *leftMotorVelocity);
FIREBIRDCLIBRARY_API BOOL getRightMotorVelocity_meterspersec(void *hSerial, float *rightMotorVelocity);
FIREBIRDCLIBRARY_API BOOL setRobotAngularVelocityRadianpersec(void *hSerial, float radianpersec);
FIREBIRDCLIBRARY_API BOOL getRobotAngularVelocityRadianpersec(void *hSerial, float *radianspersec);
FIREBIRDCLIBRARY_API BOOL setLeftMotorVelocity_radianspersec(void *hSerial, float velocity);
FIREBIRDCLIBRARY_API BOOL setRightMotorVelocity_radianspersec(void *hSerial, float velocity);
FIREBIRDCLIBRARY_API BOOL getLeftMotorVelocity_radianspersec(void *hSerial, float *leftMotorVelocity);
FIREBIRDCLIBRARY_API BOOL getRightMotorVelocity_radianspersec(void *hSerial, float *rightMotorVelocity);
FIREBIRDCLIBRARY_API BOOL setRobotID(void *hSerial, uint8 R_id);
FIREBIRDCLIBRARY_API BOOL getRobotID(void *hSerial, uint8 *R_id);
FIREBIRDCLIBRARY_API BOOL getHardwareVersion(void *hSerial, uint8 *HW_version);
FIREBIRDCLIBRARY_API BOOL getSoftwareVersion(void *hSerial, uint8 *SW_version);
FIREBIRDCLIBRARY_API BOOL getADCPotentiometer(void *hSerial, uint16 *adc_Pot);
FIREBIRDCLIBRARY_API BOOL setGPIOPannelLED(void *hSerial, uint8 led_data);
FIREBIRDCLIBRARY_API BOOL getGPIOPannelLED(void *hSerial, uint8 *led_status);
FIREBIRDCLIBRARY_API BOOL getGPIOPannelswitches(void *hSerial, uint8 *switch_status);
FIREBIRDCLIBRARY_API BOOL setSafetyTimeout(void *hSerial, uint8 safety_timeout);
FIREBIRDCLIBRARY_API BOOL getSafetyTimeout(void *hSerial, uint8 *safety_timeout);
FIREBIRDCLIBRARY_API BOOL getErrorStatus(uint8 *error);
FIREBIRDCLIBRARY_API BOOL getLeftmotorCurrent(void *hSerial, float *Left_Amp);
FIREBIRDCLIBRARY_API BOOL getRightmotorCurrent(void *hSerial, float *Right_Amp);
FIREBIRDCLIBRARY_API BOOL getLeftmotorVoltage(void *hSerial, float *Left_V);
FIREBIRDCLIBRARY_API BOOL getRightmotorVoltage(void *hSerial, float *Right_V);
FIREBIRDCLIBRARY_API BOOL setWheelDiameter_mm(void *hSerial, float diameter);
FIREBIRDCLIBRARY_API BOOL getWheelDiameter_mm(void *hSerial, float *Diameter);
FIREBIRDCLIBRARY_API BOOL setRobotAxlelength_mm(void *hSerial, float axle_length);
FIREBIRDCLIBRARY_API BOOL getRobotAxlelength_mm(void *hSerial, float *Axle_Length);
FIREBIRDCLIBRARY_API BOOL setRobotMaxVelocity_meterspersec(void *hSerial, float max_velocity);
FIREBIRDCLIBRARY_API BOOL getRobotMaxVelocity_meterspersec(void *hSerial, float *Max_Velocity);
FIREBIRDCLIBRARY_API BOOL getEncoderTicksperRevolution(void *hSerial, uint16 *ticks_mm);










