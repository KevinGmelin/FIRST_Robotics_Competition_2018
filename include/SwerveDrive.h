/*
 * SwerveDrive.h
 *
 *  Created on: Jan 13, 2018
 *      Author: Administrator
 */
#ifndef SWERVEDRIVE_H
#define SWERVEDRIVE_H
#include "WheelDrive.h"
#include "WPILib.h"
class SwerveDrive {
public:
	const double length = 19.5;
	const double width = 25;
	double r = sqrt(length*length+width*width);
    double bRRotateAngle = 90 - atan2(-(width/r), -(length)/r)*180/M_PI;
    double bLRotateAngle = 90 - atan2((width/r), -(length/r))*180/M_PI;
    double fRRotateAngle = 90 - atan2(-(width/r), length/r)*180/M_PI;
    double fLRotateAngle = 90 - atan2(width/r, length/r)*180/M_PI;

private:
	WheelDrive *backRight, *backLeft, *frontRight, *frontLeft;
	Encoder *bREnc, *bLEnc, *fREnc, *fLEnc;
	double bRPIDSpeed = 0.3, bLPIDSpeed = 0.3, fRPIDSpeed = 0.3, fLPIDSpeed = 0.3;
	double gyroStraightReading = 0;

public:
	SwerveDrive(WheelDrive *bR, int bREncA, int bREncB, WheelDrive *bL, int bLEncA, int bLEncB,
			WheelDrive *fR, int fREncA, int fREncB, WheelDrive *fL, int fLEncA, int fLEncB);
	void Drive(double x1, double y1, double x2);
	void DriveUnoptimized(double x1, double y1, double x2);

	void ResetEncoders();
	void SetWheelsToAngles(double bRA, double bLA, double fRA, double fLA);
	void SetWheelsToRotateAngles();
	bool OnTarget();
	bool DriveStraightDistanceUncorrected(double distance, double angle);
	void SetPIDSpeeds(double bRS, double bLS, double fRS, double fLS);
	void PreparePID(double bRS, double bLS, double fRS, double fLS, double gyroReading);
	bool DriveStraightDistancePID(double distance, double angle, double gryoReading);
	bool RotateAnglePID(double angle);
	void PutEncoderDataToSmartDashboard();
	double GetFLEnc();
	void Disable();
	virtual ~SwerveDrive();
};

#endif /* SWERVEDRIVE_H */
