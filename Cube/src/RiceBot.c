/*
 * Welcome to the RiceBot library!
 *
 * This library was written for use by the Rice University Vex U Robotics team.
 * All are welcome to use and modify the library, so long as due credit is given to the creator.
 * If you have questions/comments/suggestions, email me at Keiko.F.Kaplan@rice.edu
 *
 * This library was written to be used with the Purdue Robotic Operating System.
 *
 * Author: Keiko Kaplan
 */

#include "main.h"

//Allocates memory and initializes a Motor type.
Motor *newMotor() {
	Motor *m = malloc(sizeof(Motor));
	m->port = 0;
	m->out = 0;
	m->reflected = 1;
	return m;
}

/*
 * A simple function to set all 3 fields of a Motor.
 *
 * @param m The motor struct to be initialized
 * @param port The port on the Cortex which the motor is plugged into
 * @param out The power output to the motor, between -127 and 127
 * @param reflected If the output to the motor should be reversed. -1 or 1
 */
void initMotor(Motor *m, unsigned char port, int out, int reflected) {
	//	printf("initMotor ");
	m->port = port;
	m->out = out;
	m->reflected = reflected;
}

//Allocates memory and initializes a Pid type.
Pid *newPid() {
	Pid *p = malloc(sizeof(Pid));
	p->running = 0;
	p->setPoint = 0;
	p->current = 0;
	p->error = 0;
	p->lastError = 0;
	p->integral = 0;
	p->derivative = 0;
	p->kP = 0;
	p->kI = 0;
	p->kD = 0;
	p->output = 0;
	return p;
}

/*
 * A simple function to set the fields of a Pid type
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
void initPid(Pid *p, float kP, float kI, float kD) {
	p->running = 0;
	p->setPoint = 0;
	p->current = 0;
	p->error = 0;
	p->lastError = 0;
	p->integral = 0;
	p->derivative = 0;
	p->kP = kP;
	p->kI = kI;
	p->kD = kD;
	p->output = 0;
}

Ricencoder *newEncoder() {
	Ricencoder *r = malloc(sizeof(Ricencoder));
	r->left = 0;
	r->right = 0;
	r->ticksPerRev = 0;
	r->mult = 1;
	r->isIME = 0;
	return r;
}

/*
 * The Ricencoder contains data for a Left/Right encoder pair.
 *
 * @param ticksPerRev The number of ticks per revolution of the encoder
 * 						627.2 for the 393 IME in high torque mode (factory default)
 * 						392 for the 393 IME in high speed mode
 * 						360 for the Quadrature Encoder
 * @param mult A multiplier to use as compensation for gear ratio
 * @param isIME 1 if IME, 0 if quad encoder
 */
void initRicencoder(Ricencoder *r, float ticksPerRev, int mult, int isIME) {
	r->left = 0;
	r->right = 0;
	r->ticksPerRev = ticksPerRev;
	r->mult = mult;
	r->isIME = isIME;
}

void riceBotInitializeIO() {

}

/*
 * Call this from the default Initialize function.
 * After, be sure to reinitialize each motor you will be using on your robot.
 */
void riceBotInitialize() {

	Motor *MOTDTFrontRight = newMotor();
	Motor *MOTDTFrontMidRight = newMotor();
	Motor *MOTDTMidRight = newMotor();
	Motor *MOTDTBackRight = newMotor();
	Motor *MOTDTFrontLeft = newMotor();
	Motor *MOTDTFrontMidLeft = newMotor();
	Motor *MOTDTMidLeft = newMotor();
	Motor *MOTDTBackLeft = newMotor();

	Motor *MOTARMFront = newMotor();
	Motor *MOTARMRight = newMotor();
	Motor *MOTARMLeft = newMotor();
	Motor *MOTARMTopRight = newMotor();
	Motor *MOTARMBottomRight = newMotor();
	Motor *MOTARMTopLeft = newMotor();
	Motor *MOTARMBottomLeft = newMotor();

	Motor *MOTCOL = newMotor();

	printf("%d\n\r", imeInitializeAll());
	imeReset(IMEARMLEFT);
	imeReset(IMEARMRIGHT);
	encDTLeft = encoderInit(0, 0, false);
	encDTRight = encoderInit(0, 0, false);
	encARMLeft = encoderInit(0, 0, false);
	encARMRight = encoderInit(0, 0, false);

	Ricencoder *ENCDT = newEncoder();
	Ricencoder *ENCARM = newEncoder();
	//	gyro = gyroInit(0, 196);
	//	gyroVal = gyroGet(gyro);

	analogCalibrate(POTARMLeft);
	analogCalibrate(POTARMRight);
	analogCalibrate(POTARMFront);

	Pid *PidARMLeft = newPid();
	Pid *PidARMRight = newPid();
	Pid *PidARMFront = newPid();
}

/*
 * Checks joystick input and sets all Motor structs to appropriate output
 * @param controlStyle The format of the joystick input.
 * 			Can be:
 * 		 			TANKDRIVE
 * 	 	 			ARCADEDRIVE
 *	 	 			CHEEZYDRIVE
 *	 	 			MECANUMDRIVE
 *	 	 			HDRIVE
 */
void getJoystickForDriveTrain() {
	int x1 = joystickGetAnalog(1, 4);
	int y1 = joystickGetAnalog(1, 3);
	int x2 = joystickGetAnalog(1, 1);
	int y2 = joystickGetAnalog(1, 2);

	switch(controlStyle) {
	case CTTANKDRIVE:
		MOTDTFrontLeft.out = y1;
		MOTDTFrontMidLeft.out = y1;
		MOTDTMidLeft.out = y1;
		MOTDTBackLeft.out = y1;

		MOTDTFrontRight.out = y2;
		MOTDTFrontMidRight.out = y2;
		MOTDTMidRight.out = y2;
		MOTDTBackRight.out = y2;
		break;
	case CTARCADEDRIVE:
		MOTDTFrontLeft.out = (y1 + x1) / 2;
		MOTDTFrontMidLeft.out = (y1 + x1) / 2;
		MOTDTMidLeft.out = (y1 + x1) / 2;
		MOTDTBackLeft.out = (y1 + x1) / 2;

		MOTDTFrontRight.out = (y1 - x1) / 2;
		MOTDTFrontMidRight.out = (y1 - x1) / 2;
		MOTDTMidRight.out = (y1 - x1) / 2;
		MOTDTBackRight.out = (y1 - x1) / 2;
		break;
	case CTCHEEZYDRIVE:
		MOTDTFrontLeft.out = (y1 + x2) / 2;
		MOTDTFrontMidLeft.out = (y1 + x2) / 2;
		MOTDTMidLeft.out = (y1 + x2) / 2;
		MOTDTBackLeft.out = (y1 + x2) / 2;

		MOTDTFrontRight.out = (y1 - x2) / 2;
		MOTDTFrontMidRight.out = (y1 - x2) / 2;
		MOTDTMidRight.out = (y1 - x2) / 2;
		MOTDTBackRight.out = (y1 - x2) / 2;
		break;
	case CTMECANUMDRIVE:
		MOTDTFrontLeft.out = y1 + x2 + x1;
		MOTDTBackLeft.out = y1 + x2 - x1;

		MOTDTFrontRight.out = y1 - x2 - x1;
		MOTDTBackRight.out = y1 - x2 + x1;
		break;
	case CTHDRIVE:
	default:
		break;
	}
}

/* Final stage: sets all physical motors based on output set in Motor structs
 * Run in a task?
 * @param driveTrainStyle The configuration of the wheels on the robot.
 * 			Can be:
 * 					FOURWHEELS
 * 					SIXWHEELS
 * 					EIGHTWHEELS
 * 					MECANUM
 * 					HOLONOMIC
 * 					HDRIVE
 * 					SWERVE
 */
void setDriveTrainMotors(int driveTrainStyle) {
	switch(driveTrainStyle) {
	case DTFOURWHEELS:
		motorSet(MOTDTFrontLeft.port, MOTDTFrontLeft.out * MOTDTFrontLeft.reflected);
		motorSet(MOTDTBackLeft.port, MOTDTBackLeft.out * MOTDTBackLeft.reflected);

		motorSet(MOTDTFrontRight.port, MOTDTFrontRight.out * MOTDTFrontRight.reflected);
		motorSet(MOTDTBackRight.port, MOTDTBackRight.out * MOTDTBackRight.reflected);
		break;
	case DTSIXWHEELS:
		motorSet(MOTDTFrontLeft.port, MOTDTFrontLeft.out * MOTDTFrontLeft.reflected);
		motorSet(MOTDTMidLeft.port, MOTDTMidLeft.out * MOTDTMidLeft.reflected);
		motorSet(MOTDTBackLeft.port, MOTDTBackLeft.out * MOTDTBackLeft.reflected);

		motorSet(MOTDTFrontRight.port, MOTDTFrontRight.out * MOTDTFrontRight.reflected);
		motorSet(MOTDTMidRight.port, MOTDTMidRight.out * MOTDTMidRight.reflected);
		motorSet(MOTDTBackRight.port, MOTDTBackRight.out * MOTDTBackRight.reflected);
		break;
	case DTEIGHTWHEELS:
		motorSet(MOTDTFrontLeft.port, MOTDTFrontLeft.out * MOTDTFrontLeft.reflected);
		motorSet(MOTDTFrontMidLeft.port, MOTDTFrontMidLeft.out * MOTDTFrontMidLeft.reflected);
		motorSet(MOTDTMidLeft.port, MOTDTMidLeft.out * MOTDTMidLeft.reflected);
		motorSet(MOTDTBackLeft.port, MOTDTBackLeft.out * MOTDTBackLeft.reflected);

		motorSet(MOTDTFrontRight.port, MOTDTFrontRight.out * MOTDTFrontRight.reflected);
		motorSet(MOTDTFrontMidRight.port, MOTDTFrontMidRight.out * MOTDTFrontMidRight.reflected);
		motorSet(MOTDTMidRight.port, MOTDTMidRight.out * MOTDTMidRight.reflected);
		motorSet(MOTDTBackRight.port, MOTDTBackRight.out * MOTDTBackRight.reflected);
		break;
	default:
		break;
	}
}

void autonomousTask(int instruction, int distance, int pow, long timeout) {
	int target;
	long startTime = millis();

	int power[2];
	power[1] = (pow == NULL) ? 127 : pow;
	power[0] = power[1];

	switch(instruction) {
	case AUTODRIVEBASIC:
		target = ENCDT.ticksPerRev / (4 * MATH_PI) * distance;
		//		power = (pow == NULL) ? 127 : pow;
		int current[2] = {ENCDT.left, ENCDT.right};

		while(current[1] < target && millis() < startTime + timeout) {
			if(abs(current[1] - current[0]) > 50) {
				if(current[0] > current[1]) {
					power[0] = speedRegulator(power[0] - 2);
				} else if(current[0] < current[1]) {
					power[0] = speedRegulator(power[0] + 2);
				}
			}

			MOTDTFrontRight.out = power[1];
			MOTDTFrontMidRight.out = power[1];
			MOTDTMidRight.out = power[1];
			MOTDTBackRight.out = power[1];
			MOTDTFrontLeft.out = power[0];
			MOTDTFrontMidLeft.out = power[0];
			MOTDTMidLeft.out = power[0];
			MOTDTBackLeft.out = power[0];

			delay(20);
			current[0] = ENCDT.left;
			current[1] = ENCDT.right;
		}
		break;
	case AUTOTURNBASIC:
		target = distance;
		if(target < gyroVal) {		//Left Turn
			while(gyroVal > target && millis() < startTime + timeout) {
				MOTDTFrontRight.out = pow;
				MOTDTFrontMidRight.out = pow;
				MOTDTMidRight.out = pow;
				MOTDTBackRight.out = pow;
				MOTDTFrontLeft.out = -pow;
				MOTDTFrontMidLeft.out = -pow;
				MOTDTMidLeft.out = -pow;
				MOTDTBackLeft.out = -pow;
			}
		}
		else if(target > gyroVal) {	//Right Turn
			while(gyroVal < target && millis() < startTime + timeout) {
				MOTDTFrontRight.out = -pow;
				MOTDTFrontMidRight.out = -pow;
				MOTDTMidRight.out = -pow;
				MOTDTBackRight.out = -pow;
				MOTDTFrontLeft.out = pow;
				MOTDTFrontMidLeft.out = pow;
				MOTDTMidLeft.out = pow;
				MOTDTBackLeft.out = pow;
			}
		}
		break;
	case AUTODRIVEGYRO:

		break;
	}
}

void processPid(Pid *pidLoop, int current) {
	if (pidLoop->running) {
		pidLoop->current = current;
		//	printf("Current: %d\n\r", pidLoop->current);
		pidLoop->error = pidLoop->setPoint - pidLoop->current;
		pidLoop->integral += pidLoop->error;
		pidLoop->derivative = pidLoop->lastError - pidLoop->error;

		pidLoop->output = speedRegulator((pidLoop->error * pidLoop->kP) +
				(pidLoop->integral * pidLoop->kI) + (pidLoop->derivative * pidLoop->kD));

		pidLoop->lastError = pidLoop->error;
	}

}

int speedRegulator(int speed) {
	if(speed > 127) {
		return 127;
	} else if(speed < -127) {
		return -127;
	} else {
		return speed;
	}
}

int max(int a, int b) {
	if(a < b) {
		return b;
	}
	return a;
}

int min(int a, int b) {
	if(a > b) {
		return b;
	}
	return a;
}
