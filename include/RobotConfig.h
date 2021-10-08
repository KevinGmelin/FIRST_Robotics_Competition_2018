#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H
#include "WPILib.h"

namespace EncoderOffset
{
 	const double backRightOff = 2.68433;
 	const double backLeftOff = 3.29956;
 	const double frontRightOff = 0.891113;
 	const double frontLeftOff = 0.648193;
}

namespace Pin
{
	const int bRTurn = 2;
	const int bRMove = 3;
	const int bLTurn = 0;
	const int bLMove = 1;
	const int fRTurn = 8;
	const int fRMove = 9;
	const int fLTurn = 5;
	const int fLMove = 4;

	const int rotateIntake = 9;
	const int leftIntake = 10;
	const int rightIntake = 8;
	const int lift1 = 13;
	const int lift2 = 12;
	const int secondStage = 11;

	const int bRAnalogEncoder = 2;
	const int bLAnalogEncoder= 1;
	const int fRAnalogEncoder = 3;
	const int fLAnalogEncoder = 0;


	const int limitLiftTop = 21;
	const int limitLiftBottom = 22;//10
	const int secondaryStageBottom = 17;
	const int secondaryStageTop = 8;

	const int rotateIntakePot = 6;

	const int bREncA = 1;
	const int bREncB = 0;
	const int bLEncA = 2;
	const int bLEncB = 3;
	const int fREncA = 9;
	const int fREncB = 8;
	const int fLEncA = 5;
	const int fLEncB = 6;
	const int liftEncA = 14;
	const int liftEncB = 15;
	const int secondaryStageEncA = 18;
	const int secondaryStageEncB = 19;

};

namespace Reverse
{
	//all turning motors will go counterclockwise
	const bool bRTurn = false;
	const bool bRMove = false;
	const bool bLTurn = false;
	const bool bLMove = false;
	const bool fRTurn = true;
	const bool fRMove = false;
	const bool fLTurn = false;
	const bool fLMove = true;

	const bool rotateIntake = false;
	const bool leftIntake = true;
	const bool rightIntake = false;
	const bool lift1 = false;
	const bool lift2 = false;
	const bool secondStage = false;


	const bool bREnc = false;
	const bool bLEnc = false;
	const bool fREnc = false;
	const bool fLEnc = false;
	const bool liftEnc = false;
}

namespace EncoderDistancePerPulse
{
	const int cimCoderCountsPerRev = 20;
	const double swerveDriveGearRatio = 6.67;
	const double swerveWheelDiameter = 4;
	const double swerveCimCoder = (swerveWheelDiameter*M_PI)/(cimCoderCountsPerRev*swerveDriveGearRatio);
	const double lift = 1;
	const double secondaryStage = 1;
}

namespace Speeds
{
	const double intakeIn = -0.5;
	const double intakeOutSlow = 0.2;
	const double intakeOutFast = 1;
	const double rotateIntakeUp = -0.3;
	const double rotateIntakeDown = 0.2;
	const double liftUp = -0.95;

	const double liftDown = 0.1;
	const double secondaryStageUp = -0.1;
	const double secondaryStageDown = 0.1;

	const double bLStraight = 0.33; //about a rate of 27 to 29 inches per second
	const double bRStraight = 0.32;
	const double fLStraight = 0.32;
	const double fRStraight = 0.27;
}

struct ButtonStickPair
{
	int stick;
	int button;
};
namespace Buttons
{
	const ButtonStickPair rotateIntakeUp = {0,1};
	const ButtonStickPair rotateIntakeDown = {0,2};
	const ButtonStickPair rotateIntakeToShoot = {0,3};

	const ButtonStickPair intakeIn = {1, 1};
	const ButtonStickPair intakeOutFast = {1, 2};
	const ButtonStickPair driveFast = {1,3};
	const ButtonStickPair intakeClockwise = {1,4};
	const ButtonStickPair intakeCounterClockwise = {1,5};
	const ButtonStickPair liftUp = {2,3};
	const ButtonStickPair liftDown = {2,2};
	const ButtonStickPair secondaryStageUp = {2, 4};
	const ButtonStickPair secondaryStageDown = {2,5};
	const ButtonStickPair intakeOutSlow = {2, 1};
	const ButtonStickPair rotateIntakeUp2 = {2, 9};
	const ButtonStickPair rotateIntakeDown2 = {2, 8};


}

const double kPDriveStraight = 0.008;
const double intakeRotatedDown = 0.98;
const double intakeRotatedUp = 0.84;
const double intakeRotatedToShoot= 0.89;
const double liftFullyUp = 1465;
const double secondaryStageUp = 396000;
#endif /* ROBOTCONFIG_H */
