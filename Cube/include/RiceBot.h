/*
 * RiceBot.h
 *
 *  Created on: Aug 29, 2014
 *      Author: Keiko
 */

#ifndef RICEBOT_H_
#define RICEBOT_H_

#include "main.h"

//Constants
#define MATH_PI 3.14159265358979323846

//Teams
#define RED -1
#define BLUE 1

//Control Style
#define CTTANKDRIVE 	0
#define CTARCADEDRIVE	1
#define CTCHEEZYDRIVE	2
#define CTMECANUMDRIVE	3
#define CTHDRIVE		4

//Defines how the analog joystick inputs translate to motor outputs (CT-)
int controlStyle;

//Drivetrain Styles
#define DTFOURWHEELS 	0
#define DTSIXWHEELS 	1
#define DTEIGHTWHEELS 	2
#define DTMECANUM 		3
#define DTHOLONOMIC 	4
#define DTDTHDRIVE 		5
#define DTSWERVE 		6

//Defines the drivetrain installed on the robot (DT-)
int driveTrainStyle;

//Autonomous Instructions
#define AUTODRIVEBASIC	0
#define AUTODRIVEGYRO 	1
#define AUTOTURNBASIC	2
#define AUTOTURNPID		3
#define AUTOARMPID		4
#define AUTOCOLLECTORS	5


//The basic motor struct
struct motorStruct {
	unsigned char port;
	int out;
	int reflected;
};

/*
 * The Motor type serves as a wrapper to keep track of all data for each motor on the robot.
 *
 * @param port The port on the Cortex which the motor is plugged into
 * @param out The power output to the motor, between -127 and 127
 * @param reflected If the output to the motor should be reversed. -1 or 1
 */
typedef struct motorStruct Motor;

//The basic pid struct
struct pidStruct {
	int running;
	int setPoint;
	int current;
	float error;
	float lastError;
	long integral;
	float derivative;
	float kP;
	float kI;
	float kD;
	int output;
//	bool done;
};

/*
 * The Pid type contains all data for any individual pid loop we may wish to run.
 *
 * @param setPoint The target value for the loop
 * @param *current A pointer to relevant sensor value (CANNOT BE AN ARRAY)
 * @param error The difference between setPoint and &current
 * @param lastError The previous error value, used for derivative calculations
 * @param integral A running sum of all previous errors
 * @param derivative The difference between lastError and error
 * @param kP The coefficient for the proportional term
 * @param kI The coefficient for the integral term
 * @param kD The coefficient for the derivative term
 * @param output The value to be set to the motors
 */
typedef struct pidStruct Pid;

struct RicencoderStruct {
	int left;
	int right;
	float ticksPerRev;
	int mult;
	int isIME;
};

/*
 * The Ricencoder contains data for a Left/Right encoder pair.
 *
 * @param left The current input value for the left encoder
 * @param right The current input value for the right encoder
 * @param ticksPerRev The number of ticks per revolution of the encoder
 * 						627.2 for the 393 IME in high torque mode (factory default)
 * 						392 for the 393 IME in high speed mode
 * 						360 for the Quadrature Encoder
 * @param mult A multiplier to use as compensation for gear ratio
 * @param isIME 1 if IME, 0 if quad encoder
 */
typedef struct RicencoderStruct Ricencoder;

Motor MOTDTFrontRight;
Motor MOTDTFrontMidRight;
Motor MOTDTMidRight;
Motor MOTDTBackRight;
Motor MOTDTFrontLeft;
Motor MOTDTFrontMidLeft;
Motor MOTDTMidLeft;
Motor MOTDTBackLeft;

Motor MOTARMFront;
Motor MOTARMRight;
Motor MOTARMLeft;
Motor MOTARMTopRight;
Motor MOTARMBottomRight;
Motor MOTARMTopLeft;
Motor MOTARMBottomLeft;

Motor MOTCOL;

#define IMEDTLEFT		2
#define IMEDTRIGHT 		3
#define IMEARMLEFT		0
#define IMEARMRIGHT		1

#define POTARMLeft 		1
#define POTARMRight 	2
#define POTARMFront		3

Encoder encDTLeft;
Encoder encDTRight;
Encoder encARMLeft;
Encoder encARMRight;

Ricencoder ENCDT;
Ricencoder ENCARM;

Gyro gyro;
//Value of [Left, Right] Encoders retrieved at last IOTask
//int encVals[2];
//Value of Gyro retrieved at last IOTask
int gyroVal;
//Value of armPot retrieved at last IOTask
int armPot[3];

Pid PidDTLeft;
Pid PidDTRight;
Pid PidARMLeft;
Pid PidARMRight;
Pid PidARMFront;

Motor *newMotor();

/*
 * A simple function to set all 3 fields of a Motor.
 *
 * @param m The motor struct to be initialized
 * @param port The port on the Cortex which the motor is plugged into
 * @param out The power output to the motor, between -127 and 127
 * @param reflected If the output to the motor should be reversed. -1 or 1
 */
void initMotor(Motor *m, unsigned char port, int out, int reflected);

Pid *newPid();

void initPid(Pid *p, float kP, float kI, float kD);

Ricencoder *newEncoder();

void initRicencoder(Ricencoder *r, float ticksPerRev, int mult, int isIME);

void riceBotInitializeIO();

void riceBotInitialize();

void getJoystickForDriveTrain();

void setDriveTrainMotors(int driveTrainStyle);

void autonomousTask(int instruction, int distance, int pow, long timeout);

void processPid(Pid *pidLoop, int current);

int speedRegulator(int speed);

void startIOTask(void *ignore);

void startPidTask(void *ignore);

int max(int a, int b);

int min(int a, int b);

#endif /* RICEBOT_H_ */
