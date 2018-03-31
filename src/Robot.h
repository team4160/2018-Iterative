/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once
#include "Constants.h"
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include <iostream>
#include <string>

class Robot : public frc::IterativeRobot {
public:
	static constexpr int kTimeoutMs = 0; //change this to 0 if you don't want verification
	static constexpr int kEncoderUnit = 4096;
	static constexpr int kClawEncoderKnownHigh = 4096; //TODO find low position
	static constexpr int kElevatorEncoderKnownLow = 0; //TODO find high position
	static constexpr int kAutopausetime = 10;
	static constexpr float turnSensitivity = 0.6;

	//Setting up the TalonSRX's config
	static constexpr double driveRampTime = 0.25;
	static constexpr int driveCurrentLimit = 33;
	static constexpr int driveMaxCurrent = 39;
	static constexpr int driveMaxTime = 500;

	static constexpr double clawRampTime = 0.25;
	static constexpr int clawCurrentLimit = 10;
	static constexpr int clawMaxCurrent = 20; //claw shouldn't need a lot of current
	static constexpr int clawMaxTime = 100;

	static constexpr double elevatorRampTime = 0.25;
	static constexpr int elevatorCurrentLimit = 33;
	static constexpr int elevatorMaxCurrent = 39;
	static constexpr int elevatorMaxTime = 500;

	static constexpr int clawForwardLimit = kEncoderUnit * 5; //eg. 5 rotations TODO test the top soft limit
	static constexpr int clawReverseLimit = kEncoderUnit * 0; //TODO test bottom soft limit

	//Creating the TalonSRXs and sensors
	Joystick *Joystick1, *Joystick2;
	WPI_TalonSRX *DBLeft, *DBLeft2;
	WPI_TalonSRX *DBRight, *DBRight2;
	WPI_TalonSRX *Claw, *ClawLeft, *ClawRight;
	WPI_TalonSRX *Elevator1, *Elevator2, *Elevator3;
	DoubleSolenoid *ElevatorSolenoid;
	CANifier *ClawSensor;
	ADXRS450_Gyro *gyro;
	BuiltInAccelerometer *accel;
	PowerDistributionPanel *PDP;
	Timer *mytimer;

	//Setting up some functions
	void MotorBuilder(WPI_TalonSRX *srx, bool brake, bool inverted, double RampTime, int CurrentLimit, int MaxCurrent, int MaxTime);
	void FindLimits();
	void RGB(double R, double G, double B, CANifier *can);
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void DisabledInit() override;
	void DisabledPeriodic() override;
	void RobotInit() override;
	void RobotPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestInit() override;
	void TestPeriodic() override;

private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "No Auto";
	std::string m_autoSelected;

	std::string gameData;

	unsigned int driveState;

	double left;
	double right;
	double turn;
	double driveSpeed;

	bool isHomed = false;
};
