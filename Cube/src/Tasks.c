/*
 * MotorTask.c
 *
 *  Created on: Oct 27, 2014
 *      Author: Keiko
 */

#include "main.h"

void startIOTask(void *ignore) {
	while(1) {
//		printf("IOTask ");
		setDriveTrainMotors(DTFOURWHEELS);

		motorSet(MOTARMLeft.port, MOTARMLeft.out * MOTARMLeft.reflected);
		motorSet(MOTARMRight.port, MOTARMRight.out * MOTARMRight.reflected);
		motorSet(MOTARMTopLeft.port, MOTARMTopLeft.out * MOTARMTopLeft.reflected);
		motorSet(MOTARMTopRight.port, MOTARMTopRight.out * MOTARMTopRight.reflected);
		motorSet(MOTARMBottomLeft.port, MOTARMBottomLeft.out * MOTARMBottomLeft.reflected);
		motorSet(MOTARMBottomRight.port, MOTARMBottomRight.out * MOTARMBottomRight.reflected);

		motorSet(MOTCOL.port, MOTCOL.out * MOTCOL.reflected);

		armPot[0] = analogReadCalibrated(POTARMLeft);
		armPot[1] = -analogReadCalibrated(POTARMRight);
		armPot[2] = analogReadCalibrated(POTARMFront);

//		if(ENCDT.isIME) {
//			imeGet(IMEDTLEFT, &ENCDT.left);
//			imeGet(IMEDTRIGHT, &ENCDT.right);
//		} else {
//			ENCDT.left = encoderGet(encDTLeft);
//			ENCDT.right = encoderGet(encDTRight);
//		}
//
//		if(ENCARM.isIME) {
//			imeGet(IMEDTLEFT, &ENCARM.left);
//			imeGet(IMEDTRIGHT, &ENCARM.right);
//		} else {
//			ENCARM.left = encoderGet(encARMLeft);
//			ENCARM.right = encoderGet(encARMRight);
//		}
		gyroVal = gyroGet(gyro);

		delay(10);
	}
}

void startPidTask(void *ignore) {
	while(1) {
		//Manually add each pid loop here
		processPid(&PidARMLeft, (armPot[0]));
		processPid(&PidARMRight, (armPot[1]));
		processPid(&PidARMFront, (armPot[2]));
		if(PidARMLeft.running) {
			MOTARMBottomLeft.out = PidARMLeft.output;
		}
		if(PidARMRight.running) {
			MOTARMBottomRight.out = PidARMRight.output;
		}
		if(PidARMFront.running) {
			MOTARMTopLeft.out = PidARMFront.output;
			MOTARMBottomLeft.out = PidARMFront.output;
		}

		printf("SetPoint: %d/%d, Pot L/R: %d/%d, MotorOut %d/%d Running: %d/%d \n\r", PidARMLeft.setPoint,
				PidARMRight.setPoint, PidARMLeft.current, PidARMRight.current, MOTARMBottomLeft.out, MOTARMBottomRight.out,
				PidARMLeft.running, PidARMRight.running);

		delay(20);
	}
}
