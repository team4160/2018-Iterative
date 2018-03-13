/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include <iostream>
#include <string>

class Robot : public frc::IterativeRobot {
public:
	constexpr int kTimeoutMs = 10;
	constexpr int kEncoderUnit = 4096;

	//Setting up the TalonSRX's config
	constexpr double driveRampTime = 0.25;
	constexpr int driveCurrentLimit = 30;
	constexpr int driveMaxCurrent = 38;
	constexpr int driveMaxTime = 100;

	constexpr double clawRampTime = 0.25;
	constexpr int clawCurrentLimit = 20;
	constexpr int clawMaxCurrent = 25;
	constexpr int clawMaxTime = 100;

	constexpr double elevatorRampTime = 0.25;
	constexpr int elevatorCurrentLimit = 30;
	constexpr int elevatorMaxCurrent = 35;
	constexpr int elevatorMaxTime = 100;

	constexpr int clawForwardLimit = kEncoderUnit * 5; //5 rotations TODO test the top limit
	constexpr int clawReverseLimit = kEncoderUnit * 0; //TODO test bottom limit TODO if time then map elevator to claw so claw doesn't get smashed

	unsigned int armCount;
	unsigned int elevatorCount;
	unsigned int driveState;

	//Creating the TalonSRXs and sensors
	Joystick *Joystick1, *Joystick2;
	WPI_TalonSRX *DBLeft, *DBLeft2;
	WPI_TalonSRX *DBRight, *DBRight2;
	WPI_TalonSRX *Claw, *ClawLeft, *ClawRight;
	WPI_TalonSRX *Elevator1, *Elevator2, *Elevator3;
	CANifier *clawSenor;
	ADXRS450_Gyro *gyro;
	BuiltInAccelerometer *accel;
	DifferentialDrive *drive;

	//Setting up some functions
	void MotorBuilder(WPI_TalonSRX *srx, bool brake, bool inverted, double RampTime, int CurrentLimit, int MaxCurrent, int MaxTime);
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "No Auto";
	std::string m_autoSelected;
};
