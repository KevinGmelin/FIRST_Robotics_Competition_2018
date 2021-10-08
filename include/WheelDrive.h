/*
 * WheelDrive.h
 *
 *  Created on: Jan 13, 2018
 *      Author: Administrator
 */
#include "WPILib.h"
#ifndef WHEELDRIVE_H
#define WHEELDRIVE_H

class WheelDrive {
private:

	frc::PWMSpeedController *M_angleMotor, *M_speedMotor;
	frc::AnalogInput *M_turningEncoder;
	frc::PIDController *M_pidController;
	double M_encoderOffset;
	const double MAX_VOLTS= 4.90;
public:
	WheelDrive(PWMSpeedController *angleMotor, bool flipAngleMotor, PWMSpeedController *speedMotor, bool flipSpeedMotor, int encoderPin, double encoderOffset = 0);
	WheelDrive(int angleMotorPin, bool flipAngleMotor, int speedMotorPin, bool flipSpeedMotor, int encoderPin, double encoderOffset = 0);
	virtual ~WheelDrive();



	void Drive(double speed, double angle); //Will reverse motor direction instead of flipping 180
	void Disable();
	void DriveUnoptimized(double speed, double angle); //Necessary for driving with encoders
	bool OnTarget();
};

#endif /* WHEELDRIVE_H */
