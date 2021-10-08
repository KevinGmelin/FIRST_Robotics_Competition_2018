/*
 * WheelDrive.cpp
 *
 *  Created on: Jan 13, 2018
 *      Author: Administrator
 */

#include <WheelDrive.h>
#include <iostream>
#include "math.h"
WheelDrive::WheelDrive(PWMSpeedController *angleMotor, bool flipAngleMotor, PWMSpeedController *speedMotor, bool flipSpeedMotor, int encoderPin, double encoderOffset)
{

	M_angleMotor = angleMotor;
	M_speedMotor = speedMotor;
	M_angleMotor->SetInverted(flipAngleMotor);
	M_speedMotor->SetInverted(flipSpeedMotor);
	M_turningEncoder = new AnalogInput(encoderPin);
	M_pidController = new frc::PIDController(1, 0, 0, M_turningEncoder, M_angleMotor);
	M_pidController->SetInputRange(0, 4.9);
	M_pidController->SetOutputRange(-0.75,0.75);
	M_pidController->SetContinuous();
	M_pidController->SetAbsoluteTolerance(0.2);
	M_encoderOffset = encoderOffset;

}

WheelDrive::WheelDrive(int angleMotorPin, bool flipAngleMotor, int speedMotorPin, bool flipSpeedMotor, int encoderPin, double encoderOffset)
{
	M_angleMotor = new Spark(angleMotorPin);
	M_speedMotor = new Spark(speedMotorPin);
	M_angleMotor->SetInverted(flipAngleMotor);
	M_speedMotor->SetInverted(flipSpeedMotor);
	M_turningEncoder = new AnalogInput(encoderPin);
	M_pidController = new frc::PIDController(1, 0, 0, M_turningEncoder, M_angleMotor);
	M_pidController->SetInputRange(0, 4.9);
	M_pidController->SetOutputRange(-0.75,0.75);
	M_pidController->SetContinuous();
	M_pidController->SetAbsoluteTolerance(0.2);
	M_encoderOffset = encoderOffset;
}

WheelDrive::~WheelDrive()
{
	// TODO Auto-generated destructor stub
	delete(M_angleMotor);
	delete(M_speedMotor);
	delete(M_turningEncoder);
	delete(M_pidController);
}

void WheelDrive::Drive(double speed, double angle) //angle measured positive in clockwise direction from foward vector
{
	double currentAngle = -(M_turningEncoder->GetVoltage()- M_encoderOffset) * 360 / MAX_VOLTS; //measures angle positive in clockwise from foward vector
	while(angle <= -180)
	{
		angle += 360;
	}
	while(angle> 180)
	{
		angle -= 360;
	}
	while(currentAngle <= -180)
	{
		currentAngle += 360;
	}
	while(currentAngle > 180)
	{
		currentAngle -= 360;
	}
	if( (abs(angle-currentAngle) > 90) && (abs(angle-currentAngle) < 270) )
	{
		speed = -speed;
		angle += 180;
	}

	M_speedMotor->Set(speed);
    //double setpoint = (180-angle) * MAX_VOLTS/360 + M_encoderOffset; // Optimization offset can be calculated here.
    double setpoint = (-angle) * MAX_VOLTS/360 + M_encoderOffset; // Optimization offset can be calculated here.
    while(setpoint < 0)
    {
        setpoint = MAX_VOLTS + setpoint;
    }
    while(setpoint > MAX_VOLTS)
    {
        setpoint = setpoint - MAX_VOLTS;
    }
    M_pidController->SetEnabled(true);
    M_pidController->SetSetpoint(setpoint);

}

void WheelDrive::DriveUnoptimized(double speed, double angle)
{
	M_speedMotor->Set(speed);
    //double setpoint = (180-angle) * MAX_VOLTS/360 + M_encoderOffset; // Optimization offset can be calculated here.
    double setpoint = (-angle) * MAX_VOLTS/360 + M_encoderOffset; // Optimization offset can be calculated here.
    while(setpoint < 0)
    {
        setpoint = MAX_VOLTS + setpoint;
    }
    while(setpoint > MAX_VOLTS)
    {
        setpoint = setpoint - MAX_VOLTS;
    }
    M_pidController->SetEnabled(true);
    M_pidController->SetSetpoint(setpoint);
}
void WheelDrive::Disable()
{
	M_speedMotor->Set(0);
	M_pidController->SetEnabled(false);
}


bool WheelDrive::OnTarget()
{
	return (M_pidController->OnTarget());
}


//void WheelDrive::Drive
