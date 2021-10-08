/*
 * SwerveDrive.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Administrator
 */

#include <SwerveDrive.h>
#include <math.h>
#include "Robot.h"
#include <iostream>
SwerveDrive::SwerveDrive(WheelDrive *bR, int bREncA, int bREncB, WheelDrive *bL, int bLEncA, int bLEncB,
		WheelDrive *fR, int fREncA, int fREncB, WheelDrive *fL, int fLEncA, int fLEncB)
{
	// TODO Auto-generated constructor stub
	backRight = bR;
	backLeft = bL;
	frontRight = fR;
	frontLeft = fL;
	bREnc = new Encoder(bREncA, bREncB, false, Encoder::EncodingType::k4X);
	bLEnc = new Encoder(bLEncA, bLEncB, false, Encoder::EncodingType::k4X);
	fREnc = new Encoder(fREncA, fREncB, false, Encoder::EncodingType::k4X);
	fLEnc = new Encoder(fLEncA, fLEncB, false, Encoder::EncodingType::k4X);

	bREnc->SetDistancePerPulse(EncoderDistancePerPulse::swerveCimCoder);
	bLEnc->SetDistancePerPulse(EncoderDistancePerPulse::swerveCimCoder);
	fREnc->SetDistancePerPulse(EncoderDistancePerPulse::swerveCimCoder);
	fLEnc->SetDistancePerPulse(EncoderDistancePerPulse::swerveCimCoder);


}

SwerveDrive::~SwerveDrive()
{
	// TODO Auto-generated destructor stub
	delete(backRight);
	delete(backLeft);
	delete(frontRight);
	delete(frontLeft);
	delete(bREnc);
	delete(bLEnc);
	delete(fREnc);
	delete(fLEnc);
}


void SwerveDrive::Drive(double x1, double y1, double x2)
{
	if((x1 == 0) && (y1 == 0) && (x2 == 0))
	{
		backRight->Disable();
		backLeft->Disable();
		frontRight->Disable();
		frontLeft->Disable();
		return;
	}

	double FLX = x1 + x2 * (length / r );
	double FLY = y1 + x2 * (width / r);
	double FRX = FLX;
	double FRY = y1 - x2 * (width / r);
	double BRX = x1 - x2 * (length / r);
	double BRY = FRY;
	double BLX = BRX;
	double BLY = FLY;

	double backRightSpeed = sqrt ((BRX * BRX) + (BRY * BRY));
	double backLeftSpeed = sqrt ((BLX * BLX) + (BLY * BLY));
	double frontRightSpeed = sqrt ((FRX * FRX) + (FRY * FRY));
	double frontLeftSpeed = sqrt ((FLX * FLX) + (FLY * FLY));

	double maxSpeed = backRightSpeed;
	if(backLeftSpeed > maxSpeed){maxSpeed = backLeftSpeed;}
	if(frontRightSpeed > maxSpeed){maxSpeed = frontRightSpeed;}
	if(frontLeftSpeed > maxSpeed){maxSpeed = frontLeftSpeed;}

	if(maxSpeed > 1)
	{
		backRightSpeed /= maxSpeed;
		backLeftSpeed /= maxSpeed;
		frontRightSpeed /= maxSpeed;
		frontLeftSpeed /= maxSpeed;
	}

    double backRightAngle = 90 - atan2(BRY, BRX)*180/M_PI;
    double backLeftAngle = 90 - atan2(BLY, BLX)*180/M_PI;
    double frontRightAngle = 90 - atan2(FRY, FRX)*180/M_PI;
    double frontLeftAngle = 90 - atan2(FLY, FLX)*180/M_PI;

    backRight->Drive(backRightSpeed, backRightAngle);
    backLeft->Drive(backLeftSpeed, backLeftAngle);
    frontRight->Drive(frontRightSpeed, frontRightAngle);
    frontLeft->Drive(frontLeftSpeed, frontLeftAngle);

}
void SwerveDrive::DriveUnoptimized(double x1, double y1, double x2)
{
	if((x1 == 0) && (y1 == 0) && (x2 == 0))
	{
		backRight->Disable();
		backLeft->Disable();
		frontRight->Disable();
		frontLeft->Disable();
		return;
	}

	double FLX = x1 + x2 * (length / r );
	double FLY = y1 + x2 * (width / r);
	double FRX = FLX;
	double FRY = y1 - x2 * (width / r);
	double BRX = x1 - x2 * (length / r);
	double BRY = FRY;
	double BLX = BRX;
	double BLY = FLY;

	double backRightSpeed = sqrt ((BRX * BRX) + (BRY * BRY));
	double backLeftSpeed = sqrt ((BLX * BLX) + (BLY * BLY));
	double frontRightSpeed = sqrt ((FRX * FRX) + (FRY * FRY));
	double frontLeftSpeed = sqrt ((FLX * FLX) + (FLY * FLY));

	double maxSpeed = backRightSpeed;
	if(backLeftSpeed > maxSpeed){maxSpeed = backLeftSpeed;}
	if(frontRightSpeed > maxSpeed){maxSpeed = frontRightSpeed;}
	if(frontLeftSpeed > maxSpeed){maxSpeed = frontLeftSpeed;}

	if(maxSpeed > 1)
	{
		backRightSpeed /= maxSpeed;
		backLeftSpeed /= maxSpeed;
		frontRightSpeed /= maxSpeed;
		frontLeftSpeed /= maxSpeed;
	}

    double backRightAngle = 90 - atan2(BRY, BRX)*180/M_PI;
    double backLeftAngle = 90 - atan2(BLY, BLX)*180/M_PI;
    double frontRightAngle = 90 - atan2(FRY, FRX)*180/M_PI;
    double frontLeftAngle = 90 - atan2(FLY, FLX)*180/M_PI;

    backRight->DriveUnoptimized(backRightSpeed, backRightAngle);
    backLeft->DriveUnoptimized(backLeftSpeed, backLeftAngle);
    frontRight->DriveUnoptimized(frontRightSpeed, frontRightAngle);
    frontLeft->DriveUnoptimized(frontLeftSpeed, frontLeftAngle);

}


void SwerveDrive::ResetEncoders()
{
		bREnc->Reset();
		bLEnc->Reset();
		fREnc->Reset();
		fLEnc->Reset();
}

void SwerveDrive::SetWheelsToAngles(double bRA, double bLA, double fRA, double fLA)
{
	backRight->DriveUnoptimized(0, bRA);
	backLeft->DriveUnoptimized(0, bLA);
	frontRight->DriveUnoptimized(0, fRA);
	frontLeft->DriveUnoptimized(0, fLA);

}

void SwerveDrive::SetWheelsToRotateAngles()
{
	backRight->DriveUnoptimized(0, bRRotateAngle);
	backLeft->DriveUnoptimized(0, bLRotateAngle);
	frontRight->DriveUnoptimized(0, fRRotateAngle);
	frontLeft->DriveUnoptimized(0, fLRotateAngle);
}
bool SwerveDrive::OnTarget()
{
	bool bROn = backRight->OnTarget();
	bool bLOn = backLeft->OnTarget();
	bool fROn = frontRight->OnTarget();
	bool fLOn = frontLeft->OnTarget();

	return (bROn && bLOn && fROn && fLOn);

}



bool SwerveDrive::DriveStraightDistanceUncorrected(double distance, double angle)
{
	if(bREnc->GetDistance() < distance)
	{
		backRight->Drive(Speeds::bRStraight, angle);
		backLeft->Drive(Speeds::bLStraight, angle);
		frontRight->Drive(Speeds::fRStraight, angle);
		frontLeft->Drive(Speeds::fLStraight, angle);
		return false;
	}
	else
	{
		backRight->Disable();
		backLeft->Disable();
		frontRight->Disable();
		frontLeft->Disable();
		return true;
	}



}

void SwerveDrive::SetPIDSpeeds(double bRS, double bLS, double fRS, double fLS)
{
	bRPIDSpeed = bRS;
	bLPIDSpeed = bLS;
	fRPIDSpeed = fRS;
	fLPIDSpeed = fLS;
}
void SwerveDrive::PreparePID(double bRS, double bLS, double fRS, double fLS, double gyroReading)
{
	bRPIDSpeed = bRS;
	bLPIDSpeed = bLS;
	fRPIDSpeed = fRS;
	fLPIDSpeed = fLS;
	SwerveDrive::gyroStraightReading = gyroReading;
	bREnc->Reset();
	bLEnc->Reset();
	fREnc->Reset();
	fLEnc->Reset();

}
bool SwerveDrive::DriveStraightDistancePID(double distance, double angle, double gyroReading)
{
	if(fLEnc->GetDistance() < distance)
	{
		bRPIDSpeed += (fREnc->GetRate() - bREnc->GetRate())*kPDriveStraight;
		bLPIDSpeed += (fLEnc->GetRate() - bLEnc->GetRate())*kPDriveStraight;
		double error = gyroReading - gyroStraightReading;
		if(abs(error) > 0.05)
		{
			fRPIDSpeed += error*(-0.0004);
		}
		backRight->DriveUnoptimized(bRPIDSpeed, angle);
		backLeft->DriveUnoptimized(bLPIDSpeed, angle);
		frontRight->DriveUnoptimized(fRPIDSpeed, angle);
		frontLeft->DriveUnoptimized(fLPIDSpeed, angle);
		return false;
	}
	else
	{
		return true;
	}
}

bool SwerveDrive::RotateAnglePID(double angle)
{
	double distance = r*M_PI*angle/360;

	if(fLEnc->GetDistance() < distance)
	{
		bRPIDSpeed += (fLEnc->GetRate() - bREnc->GetRate())*kPDriveStraight;
		bLPIDSpeed += (fLEnc->GetRate() - bLEnc->GetRate())*kPDriveStraight;
		fRPIDSpeed += (fLEnc->GetRate() - fREnc->GetRate())*kPDriveStraight;
		backRight->DriveUnoptimized(bRPIDSpeed, bRRotateAngle);
		backLeft->DriveUnoptimized(bLPIDSpeed, bLRotateAngle);
		frontRight->DriveUnoptimized(fRPIDSpeed, fRRotateAngle);
		frontLeft->DriveUnoptimized(fLPIDSpeed, fLRotateAngle);
		return false;
	}
	else
	{
		return true;
	}
}
void SwerveDrive::PutEncoderDataToSmartDashboard()
{
	SmartDashboard::PutNumber("BackRightEncDistance", bREnc->GetDistance());
	SmartDashboard::PutNumber("BackLeftEncDistance", bLEnc->GetDistance());
	SmartDashboard::PutNumber("frontRightEncDistance", fREnc->GetDistance());
	SmartDashboard::PutNumber("frontLeftEncDistance", fLEnc->GetDistance());
	SmartDashboard::PutNumber("BackRightEncRate", bREnc->GetRate());
	SmartDashboard::PutNumber("BackLeftEncRate", bLEnc->GetRate());
	SmartDashboard::PutNumber("frontRightEncRate", fREnc->GetRate());
	SmartDashboard::PutNumber("frontLeftEncRate", fLEnc->GetRate());
	SmartDashboard::PutNumber("blSpeed", bLPIDSpeed);
	SmartDashboard::PutNumber("brSpeed", bRPIDSpeed);
	SmartDashboard::PutNumber("flSpeed", fLPIDSpeed);
	SmartDashboard::PutNumber("fRSpeed", fRPIDSpeed);


}
double SwerveDrive::GetFLEnc()
{
	return (fLEnc->GetDistance());
}
void SwerveDrive::Disable()
{
	backRight->Disable();
	backLeft->Disable();
	frontRight->Disable();
	frontLeft->Disable();
}
