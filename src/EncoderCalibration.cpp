#include <iostream>
#include <string>

#include <Robot.h>
#include <Drive/DifferentialDrive.h>
#include <Joystick.h>
#include <SampleRobot.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <Spark.h>
#include <Timer.h>
#include "WPILib.h"
#include "math.h"
using namespace frc;


class Robot : public frc::SampleRobot {
	AnalogInput bRTurnE, bLTurnE, fRTurnE, fLTurnE;
	Encoder bRMoveE, bLMoveE, fRMoveE, fLMoveE, liftEnc;
	DigitalInput liftBottom, liftTop, secondaryStageBottom, secondaryStageTop;
	AnalogInput rotateIntakePot;
	Joystick stick;
public:
	Robot():
	bRTurnE(Pin::bRAnalogEncoder),
	bLTurnE(Pin::bLAnalogEncoder),
	fRTurnE(Pin::fRAnalogEncoder),
	fLTurnE(Pin::fLAnalogEncoder),
	bRMoveE(Pin::bREncA,Pin::bREncB),
	bLMoveE(Pin::bLEncA,Pin::bLEncB),
	fRMoveE(Pin::fREncA,Pin::fREncB),
	fLMoveE(Pin::fLEncA,Pin::fLEncB),
	liftEnc(Pin::liftEncA, Pin::liftEncB),
	liftBottom(Pin::limitLiftBottom),
	liftTop(Pin::limitLiftTop),
	secondaryStageBottom(Pin::secondaryStageBottom),
	secondaryStageTop(Pin::secondaryStageTop),
	rotateIntakePot(Pin::rotateIntakePot),
	stick(0)
	{
		bRMoveE.SetDistancePerPulse(EncoderDistancePerPulse::bR);
		bLMoveE.SetDistancePerPulse(EncoderDistancePerPulse::bL);
		fRMoveE.SetDistancePerPulse(EncoderDistancePerPulse::fR);
		fLMoveE.SetDistancePerPulse(EncoderDistancePerPulse::fL);
		liftEnc.SetDistancePerPulse(EncoderDistancePerPulse::lift);
	}

	void RobotInit() {

	}

	void Autonomous() {

	}

	void OperatorControl() override {

		while (IsOperatorControl() && IsEnabled())
		{
			if(stick.GetRawButton(1)){
				std::cout << "BackRightTurn: " << bRTurnE.GetVoltage() << std::endl;
				std::cout << "BackLeftTurn: " << bLTurnE.GetVoltage() << std::endl;
				std::cout << "FrontRightTurn: " << fRTurnE.GetVoltage() << std::endl;
				std::cout << "FrontLeftTurn: " << fLTurnE.GetVoltage() << std::endl;
				std::cout << "BackRightMovDistance: " << bRMoveE.GetDistance() << std::endl;
				std::cout << "BackLeftMovDistance: " << bLMoveE.GetDistance() << std::endl;
				std::cout << "frontRightMovDistance: " << fRMoveE.GetDistance() << std::endl;
				std::cout << "frontLeftMovDistance: " << fLMoveE.GetDistance() << std::endl;
				std::cout << "BackRightMovRate: " << bRMoveE.GetRate() << std::endl;
				std::cout << "BackLeftMovRate: " << bLMoveE.GetRate() << std::endl;
				std::cout << "frontRightMovRate: " << fRMoveE.GetRate() << std::endl;
				std::cout << "frontLeftMovRate: " << fLMoveE.GetRate() << std::endl;
				std::cout << "liftEncDistance: " << liftEnc.GetDistance() << std::endl;
				std::cout << "liftEncRate: " << liftEnc.GetRate() << std::endl;
				std::cout << "liftBottom: " << liftBottom.Get() << std::endl;
				std::cout << "liftTop: " << liftTop.Get() << std::endl;
				std::cout << "secondaryStageBottom: " << secondaryStageBottom.Get() << std::endl;
				std::cout << "secondaryStageTop: " << secondaryStageTop.Get() << std::endl;
				std::cout << "rotateIntakePot" << rotateIntakePot.GetVoltage() << std::endl;

				frc::Wait(1);

			}


		}
	}

	/*
	 * Runs during test mode
	 */
	void Test() override {}

private:

};

START_ROBOT_CLASS(Robot)
