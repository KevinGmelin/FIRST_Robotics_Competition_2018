#include "Robot.h"
#include <iostream>
#include <string>
#include "WPILib.h"
#include "SwerveDrive.h"
#include "WheelDrive.h"
#include <math.h>
#include <vector>
#include "ctre/Phoenix.h"
#include <pthread.h>
#include <math.h>
#include <ctime>
#include <algorithm> //std::sort
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace frc;
using namespace cv;

static void VisionThread()
{
	// Get the USB camera from CameraServer
	cs::UsbCamera camera = CameraServer::GetInstance()->StartAutomaticCapture();
	// Set the resolution
	int frameWidth = 640, frameHeight = 480;
	camera.SetResolution(frameWidth, frameHeight);

	// Get a CvSink. This will capture Mats from the Camera
	cs::CvSink cvSink = CameraServer::GetInstance()->GetVideo();
	// Setup a CvSource. This will send images back to the Dashboard
	cs::CvSource outputStream = CameraServer::GetInstance()->PutVideo("Camera Footage", frameWidth, frameHeight);

	// Mats are very memory expensive. Lets reuse this Mat.
	cv::Mat mat;

	//cv::VideoWriter video(fileName.str(), VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, cv::Size(frameWidth, frameHeight), true);
	while (true)
	{
		// Tell the CvSink to grab a frame from the camera and
		// put it
		// in the source mat.  If there is an error notify the
		// output.
		if (cvSink.GrabFrame(mat) == 0) {
			// Send the output the error.
			outputStream.NotifyError(cvSink.GetError());
			// skip the rest of the current iteration
			continue;
		}
		outputStream.PutFrame(mat);

	}
}

class Robot : public frc::IterativeRobot
{
private:
	WheelDrive backRight, backLeft, frontRight, frontLeft;
	SwerveDrive swerveDrive;
	Spark rotateIntake, leftIntake,rightIntake;
	Talon lift1, lift2;
	Spark secondaryStage;
	DigitalInput limitLiftTop, limitLiftBottom, secondaryStageTop, secondaryStageBottom;
	AnalogInput rotatePot;
	PIDController rotateIntakePIDController;
	Encoder liftEnc, secondaryStageEnc;
	PIDController lift1PIDController, lift2PIDController, secondaryStagePIDController;
	PigeonIMU pidgey;
	double autoRotationRate{0.07}, autoStartingAngle{0};
	std::vector<Joystick*> stick;
	int autoStep = 1;
	bool liftIsCalibrated = false, secondaryStageIsCalibrated = false;
	frc::SendableChooser<std::string> modeChooser, sideChooser;
	const std::string kAutoNameDefault = "Nothing";
	const std::string kAutoNameLeftTimed = "LeftTimed";
	const std::string kAutoNamePIDDist = "PIDDist";
	const std::string kAutoNamePIDDist2 = "PIDDist2";

	const std::string kAutoNameTurnWheels = "TurnWheels";
	const std::string kAutoNamePIDRotate = "PIDRot";
	const std::string kAutoNameVision = "Vision";
	const std::string kAutoNameCompound = "Compoound";
	const std::string left = "Left";
	const std::string right = "Right";
	const std::string middle = "Middle";
	const std::string kAutoModeSwitch = "Switch";
	std::string modeSelected, sideSelected;
	std::string gameData;
	DriverStation& ds = DriverStation::GetInstance();

	frc::Timer time;
	bool visionThreadTerminated = false;
public:
	Robot():
		backRight(Pin::bRTurn, Reverse::bRTurn, Pin::bRMove, Reverse::bRMove, Pin::bRAnalogEncoder, EncoderOffset::backRightOff),
		backLeft(Pin::bLTurn, Reverse::bLTurn, Pin::bLMove, Reverse::bLMove, Pin::bLAnalogEncoder, EncoderOffset::backLeftOff),
		frontRight(Pin::fRTurn, Reverse::fRTurn, Pin::fRMove, Reverse::fRMove, Pin::fRAnalogEncoder, EncoderOffset::frontRightOff),
		frontLeft(Pin::fLTurn, Reverse::fLTurn, Pin::fLMove, Reverse::fLMove, Pin::fLAnalogEncoder, EncoderOffset::frontLeftOff),
		swerveDrive(&backRight, Pin::bREncA, Pin::bREncB, &backLeft, Pin::bLEncA, Pin::bLEncB, &frontRight, Pin::fREncA, Pin::fREncB, &frontLeft, Pin::fLEncA, Pin::fLEncB),
		rotateIntake(Pin::rotateIntake),
		leftIntake(Pin::rightIntake),
		rightIntake(Pin::leftIntake),
		lift1(Pin::lift1),
		lift2(Pin::lift2),
		secondaryStage(Pin::secondStage),
		limitLiftTop(Pin::limitLiftTop),
		limitLiftBottom(Pin::limitLiftBottom),
		secondaryStageTop(Pin::secondaryStageTop),
		secondaryStageBottom(Pin::secondaryStageBottom),
		rotatePot(Pin::rotateIntakePot),
		rotateIntakePIDController(12, 0, 0, &rotatePot, &rotateIntake),
		liftEnc(Pin::liftEncA, Pin::liftEncB),
		secondaryStageEnc(Pin::secondaryStageEncA, Pin::secondaryStageEncB),
		lift1PIDController(-0.01, 0, 0, &liftEnc, &lift1),
		lift2PIDController(-0.01, 0, 0, &liftEnc, &lift2),
		secondaryStagePIDController(-0.00001, 0, 0, &secondaryStageEnc, &secondaryStage),
		pidgey(1)

	{
		stick.push_back(new Joystick(0));
		stick.push_back(new Joystick(1));
		stick.push_back(new Joystick(2));
		rotateIntake.SetInverted(Reverse::rotateIntake);
		leftIntake.SetInverted(Reverse::leftIntake);
		rightIntake.SetInverted(Pin::rightIntake);
		lift1.SetInverted(Reverse::lift1);
		lift2.SetInverted(Reverse::lift2);
		secondaryStage.SetInverted(Reverse::secondStage);

		rotateIntakePIDController.SetInputRange(0.8, 1);
		rotateIntakePIDController.SetOutputRange(-0.5, 0.5);
		rotateIntakePIDController.SetAbsoluteTolerance(0.01);
		lift1PIDController.SetInputRange(0, 1500);
		lift1PIDController.SetOutputRange(-1, 1);
		lift1PIDController.SetPercentTolerance(5);
		lift2PIDController.SetInputRange(0, 1500);
		lift2PIDController.SetOutputRange(-1, 1);
		lift2PIDController.SetPercentTolerance(5);
		secondaryStagePIDController.SetInputRange(0, 400000);
		secondaryStagePIDController.SetOutputRange(-1, 1);
		secondaryStagePIDController.SetPercentTolerance(5);

		liftEnc.SetDistancePerPulse(EncoderDistancePerPulse::lift);
		secondaryStageEnc.SetDistancePerPulse(EncoderDistancePerPulse::secondaryStage);

	}

	~Robot()
	{
		for (Joystick* object : stick)
		    delete object;
		stick.clear();
	}

	void RobotInit()
	{
		sideChooser.AddDefault(left, left);
		sideChooser.AddObject(right, right);

		frc::SmartDashboard::PutData("Auto sides", &sideChooser);

		time.Start();
		std::thread visionThread(VisionThread);
		visionThread.detach();
	}

	void AutonomousInit() override {
		modeSelected = kAutoModeSwitch;
		sideSelected = sideChooser.GetSelected();

		std::cout << "Auto selected: " << modeSelected << std::endl;
		autoStep = 1;
		time.Reset();
		time.Start();
		autoStartingAngle = pidgey.GetFusedHeading();
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
		std::cout << "Switch is on " << gameData[0] << std::endl;

	}

	void AutonomousPeriodic()
	{
		double autoAngle = pidgey.GetFusedHeading();
		if(modeSelected == kAutoModeSwitch)
		{

			switch(autoStep)
			{
				case 1:
					time.Reset();
					std::cout << ("Starting PIDDist") << std::endl;
					autoStep++;
					break;
				case 2:
					if(sideSelected == left)
						swerveDrive.SetWheelsToAngles(-90, -90, -90, -90);
					else if(sideSelected == right)
						swerveDrive.SetWheelsToAngles(90,90,90,90);
					if(time.Get() > 0.5)
					{
						swerveDrive.ResetEncoders();
						autoStartingAngle = pidgey.GetFusedHeading();
						std::cout << "Starting Angle: " << autoStartingAngle << std::endl;
						autoStep++;
					}
					break;
				case 3:
					swerveDrive.PutEncoderDataToSmartDashboard();
					SmartDashboard:: PutNumber("Angle", autoAngle);
					if(swerveDrive.GetFLEnc() < 138)
					{
						if(sideSelected == right)
							swerveDrive.DriveUnoptimized(0.4, 0, (pidgey.GetFusedHeading() - autoStartingAngle)/75);
						else
							swerveDrive.DriveUnoptimized(-0.4, 0, (pidgey.GetFusedHeading() - autoStartingAngle)/75);

					}
					else
					{
						autoStep++;
						swerveDrive.Disable();
						gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
						time.Reset();
					}
					break;
				case 4:
					swerveDrive.SetWheelsToAngles(0, 0, 0, 0);

					if(time.Get()>0.3)
					{
						swerveDrive.ResetEncoders();
						if((gameData[0] == 'L' && sideSelected == right) || (gameData[0] == 'R' && sideSelected == left) )
						{
							autoStep = -1;
						}
						else
						{
							autoStep++;
						}
					}
					break;
				case 5:
					if(swerveDrive.GetFLEnc() < 15)
					{
						if(sideSelected == left)
							swerveDrive.DriveUnoptimized(0, 0.4, (pidgey.GetFusedHeading() - autoStartingAngle)/75);
						else if(sideSelected == right)
							swerveDrive.DriveUnoptimized(0, 0.4, (pidgey.GetFusedHeading() - autoStartingAngle)/75);
					}
					else
					{
						autoStep++;
						swerveDrive.Disable();
						time.Reset();
					}
					break;
				case 6:
					rotateIntakePIDController.SetSetpoint(intakeRotatedToShoot);
					rotateIntakePIDController.SetEnabled(true);
					if(time.Get() > 0.5)
					{
						autoStep++;
						time.Reset();
					}
					break;
				case 7:
					leftIntake.Set(Speeds::intakeOutFast);
					rightIntake.Set(Speeds::intakeOutFast);
					if(time.Get()>1)
					{
						autoStep++;
					}
					break;
				default:
					swerveDrive.Disable();
					leftIntake.Set(0);
					rightIntake.Set(0);
			}
		}

		Wait(0.02);
	}

	void TeleopPeriodic() override
	{
		SmartDashboard::PutNumber("LeftStickX",stick[0]->GetX());
		SmartDashboard::PutNumber("LeftStickY",stick[0]->GetY());
		SmartDashboard::PutNumber("RightStickX",stick[1]->GetX());
		SmartDashboard::PutNumber("Fused Heading", pidgey.GetFusedHeading());
		double ypr[3];
		pidgey.GetYawPitchRoll(ypr);

		SmartDashboard::PutNumber("Yaw", ypr[0]);

		// The motors will be updated every 5ms
		if((fabs(stick[0]->GetMagnitude()) > 0.05) || (fabs(stick[1]->GetX()) > 0.05))
		{
			if(stick[Buttons::driveFast.stick]->GetRawButton(Buttons::driveFast.button))
			{
				swerveDrive.Drive(stick[0]->GetX(), -stick[0]->GetY(),stick[1]->GetX());
			}
			else
			{
				swerveDrive.Drive(0.5*stick[0]->GetX(), -0.5*stick[0]->GetY(),0.5*stick[1]->GetX());
			}
		}
		else
		{
			swerveDrive.Drive(0,0,0);
		}


		if(stick[Buttons::intakeIn.stick]->GetRawButton(Buttons::intakeIn.button))
		{
			leftIntake.Set(Speeds::intakeIn);
			rightIntake.Set(Speeds::intakeIn);
		}
		else if(stick[Buttons::intakeOutSlow.stick]->GetRawButton(Buttons::intakeOutSlow.button))
		{
			leftIntake.Set(Speeds::intakeOutSlow);
			rightIntake.Set(Speeds::intakeOutSlow);
		}
		else if(stick[Buttons::intakeOutFast.stick]->GetRawButton(Buttons::intakeOutFast.button))
		{
			leftIntake.Set(Speeds::intakeOutFast);
			rightIntake.Set(Speeds::intakeOutFast);
		}
		else if(stick[Buttons::intakeClockwise.stick]->GetRawButton(Buttons::intakeClockwise.button))
		{
			leftIntake.Set(-Speeds::intakeIn);
			rightIntake.Set(Speeds::intakeIn);
		}
		else if(stick[Buttons::intakeCounterClockwise.stick]->GetRawButton(Buttons::intakeCounterClockwise.button))
		{
			leftIntake.Set(Speeds::intakeIn);
			rightIntake.Set(-Speeds::intakeIn);
		}
		else
		{
			leftIntake.Set(0);
			rightIntake.Set(0);
		}

		if(stick[Buttons::rotateIntakeUp.stick]->GetRawButton(Buttons::rotateIntakeUp.button))
		{
			rotateIntakePIDController.SetSetpoint(intakeRotatedUp);
			rotateIntakePIDController.SetEnabled(true);
		}
		else if(stick[Buttons::rotateIntakeToShoot.stick]->GetRawButton(Buttons::rotateIntakeToShoot.button))
		{
			rotateIntakePIDController.SetSetpoint(intakeRotatedToShoot);
			rotateIntakePIDController.SetEnabled(true);
		}
		else if(stick[Buttons::rotateIntakeDown.stick]->GetRawButton(Buttons::rotateIntakeDown.button))
		{
			rotateIntakePIDController.SetSetpoint(intakeRotatedDown);
			rotateIntakePIDController.SetEnabled(true);
		}
		else if(stick[Buttons::rotateIntakeUp2.stick]->GetRawButton(Buttons::rotateIntakeUp2.button) && (rotatePot.GetVoltage() > intakeRotatedUp))
		{
			rotateIntakePIDController.SetEnabled(false);
			rotateIntake.Set(Speeds::rotateIntakeUp);
		}
		else if(stick[Buttons::rotateIntakeDown2.stick]->GetRawButton(Buttons::rotateIntakeDown2.button) && (rotatePot.GetVoltage() < intakeRotatedDown))
		{
			rotateIntakePIDController.SetEnabled(false);
			rotateIntake.Set(Speeds::rotateIntakeDown);
		}
		else
		{
			if(!rotateIntakePIDController.IsEnabled())
			{
				rotateIntake.Set(0);
			}
		}

		if(stick[Buttons::liftUp.stick]->GetRawButton(Buttons::liftUp.button) && limitLiftTop.Get() && liftIsCalibrated) //lift up
		{
			lift1PIDController.SetSetpoint(liftFullyUp);
			lift2PIDController.SetSetpoint(liftFullyUp);

			lift1PIDController.SetEnabled(true);
			lift2PIDController.SetEnabled(true);

		}
		else if(stick[Buttons::liftDown.stick]->GetRawButton(Buttons::liftDown.button) && limitLiftBottom.Get() && liftIsCalibrated)
		{
			lift1PIDController.SetSetpoint(25);
			lift2PIDController.SetSetpoint(25);

			lift1PIDController.SetEnabled(true);
			lift2PIDController.SetEnabled(true);
		}
		else if(stick[Buttons::liftDown.stick]->GetRawButton(Buttons::liftDown.button) && limitLiftBottom.Get())
		{
			lift1PIDController.SetEnabled(false);
			lift2PIDController.SetEnabled(false);
			lift1.Set(Speeds::liftDown);
			lift2.Set(Speeds::liftDown);
		}
		else if(stick[Buttons::overrideLiftUp.stick]->GetRawButton(Buttons::overrideLiftUp.button))
		{
			lift1PIDController.SetEnabled(false);
			lift2PIDController.SetEnabled(false);
			lift1.Set(Speeds::liftUp);
			lift2.Set(Speeds::liftUp);
		}
		else if(stick[Buttons::overrideLiftDown.stick]->GetRawButton(Buttons::overrideLiftDown.button))
		{
			lift1PIDController.SetEnabled(false);
			lift2PIDController.SetEnabled(false);
			lift1.Set(Speeds::liftDown);
			lift2.Set(Speeds::liftDown);
		}
		else if(!limitLiftBottom.Get())
		{
			lift1PIDController.SetEnabled(false);
			lift2PIDController.SetEnabled(false);
			lift1.Set(0);
			lift2.Set(0);
			liftEnc.Reset();
			liftIsCalibrated = true;
		}
		else
		{
			lift1PIDController.SetEnabled(false);
			lift2PIDController.SetEnabled(false);
			lift1.Set(0);
			lift2.Set(0);
		}

		if(stick[Buttons::secondaryStageUp.stick]->GetRawButton(Buttons::secondaryStageUp.button) && secondaryStageIsCalibrated && secondaryStageTop.Get()/* && secondaryStageTop.Get()*/)
		{
			secondaryStagePIDController.SetSetpoint(secondaryStageUp-5000);
			secondaryStagePIDController.SetEnabled(true);

		}
		else if(stick[Buttons::secondaryStageDown.stick]->GetRawButton(Buttons::secondaryStageDown.button) && secondaryStageIsCalibrated && secondaryStageBottom.Get()/* && secondaryStageBottom.Get()*/)
		{
			secondaryStagePIDController.SetSetpoint(-500);
			secondaryStagePIDController.SetEnabled(true);
		}
		else if(stick[Buttons::secondaryStageDown.stick]->GetRawButton(Buttons::secondaryStageDown.button) && secondaryStageBottom.Get())
		{
			secondaryStage.Set(Speeds::secondaryStageDown);

		}
		else if(stick[Buttons::overrideSecondaryStageUp.stick]->GetRawButton(Buttons::overrideSecondaryStageUp.button))
		{
			secondaryStagePIDController.SetEnabled(false);
			secondaryStagePIDController.SetEnabled(false);
			secondaryStage.Set(Speeds::secondaryStageUp);
		}
		else if(stick[Buttons::overrideSecondaryStageDown.stick]->GetRawButton(Buttons::overrideSecondaryStageDown.button))
		{
			secondaryStagePIDController.SetEnabled(false);
			secondaryStagePIDController.SetEnabled(false);
			secondaryStage.Set(Speeds::secondaryStageDown);
		}
		else if(!secondaryStageBottom.Get())
		{
			secondaryStage.Set(0);
			secondaryStageIsCalibrated = true;
			secondaryStageEnc.Reset();
		}
		else
		{
			secondaryStagePIDController.SetEnabled(false);
			secondaryStage.Set(0);
		}

		if(!visionThreadTerminated && (ds.GetMatchTime() < 3))
		{

		}

		frc::Wait(0.005);
	}

	void DisabledPeriodic() override
	{
		if(stick[0]->GetRawButton(1))
		{
			std::cout << "RotatePot: " << rotatePot.GetVoltage() << std::endl;
			std::cout << "LiftTop: " << limitLiftTop.Get() << std::endl;
			std::cout << "LiftBottom: " << limitLiftBottom.Get() << std::endl;
			std::cout << "SecondaryStageTop: " << secondaryStageTop.Get() << std::endl;
			std::cout << "SecondaryStageBottom: " << secondaryStageBottom.Get() << std::endl;
			std::cout << "LiftEncoder: " << liftEnc.GetDistance() << std::endl;
			std::cout << "SecondaryStageEncoder: " << secondaryStageEnc.GetDistance() << std::endl;
			std::cout << std::endl;
			Wait(1);
		}
	}

};

START_ROBOT_CLASS(Robot)
