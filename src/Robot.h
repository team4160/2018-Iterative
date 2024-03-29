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
#include <cmath>
#include <map>

class Robot: public frc::IterativeRobot {
public:
	static constexpr int kTimeoutMs = 0; //change this to 0 if you don't want verification
	static constexpr int kEncoderUnit = 4096;
	static constexpr int kClawEncoderKnownHigh = -1000; //TODO find low position
	static constexpr int kElevatorEncoderKnownLow = 0; //TODO find high position
	static constexpr int kElevatorEncoderMiddle = 16000;
	static constexpr int kElevatorEncoderHigh = 36000;
	static constexpr double kAutopausetime = 10;
	static constexpr double turnSensitivity = 0.6;
	static constexpr double elevatorSenseCurrent = 5.0;
	static constexpr double clawSenseCurrent = 5.0;

	//Setting up the TalonSRX's config
	static constexpr double driveRampTime = 0.2;
	static constexpr int driveCurrentLimit = 0;
	static constexpr int driveMaxCurrent = 0;
	static constexpr int driveMaxTime = 0;

	static constexpr double clawRampTime = 0;
	static constexpr int clawCurrentLimit = 20;
	static constexpr int clawMaxCurrent = 40; //claw shouldn't need a lot of current
	static constexpr int clawMaxTime = 100;

	static constexpr double elevatorRampTime = 0.3;
	static constexpr int elevatorCurrentLimit = 5;
	static constexpr int elevatorMaxCurrent = 10;
	static constexpr int elevatorMaxTime = 100;

	static constexpr int clawForwardLimit = -150; //eg. 5 rotations TODO test the top soft limit
	static constexpr int clawReverseLimit = -850; //TODO test bottom soft limit

	//Creating the TalonSRXs and sensors
	Joystick *Joystick1, *Joystick2;
	WPI_TalonSRX *DBLeft, *DBLeft2;
	WPI_TalonSRX *DBRight, *DBRight2;
	WPI_TalonSRX *Claw, *Claw2, *ClawLeft, *ClawRight;
	WPI_TalonSRX *Elevator1, *Elevator2, *Elevator3;
//	DoubleSolenoid *ElevatorSolenoid;
	CANifier *ClawSensor;
	ADXRS450_Gyro *gyro;
	BuiltInAccelerometer *accel;
	PowerDistributionPanel *PDP;
	Timer *mytimer;

	//Setting up some functions
	void MotorBuilder(WPI_TalonSRX *srx, bool brake, bool inverted, double RampTime, int CurrentLimit, int MaxCurrent, int MaxTime);
//	void RGB(double R, double G, double B, CANifier *can);
//	void SetAllianceColor();
//	void homeElevator();
//	void homeClaw();
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
	int Testing();

private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameLeft = "Left";
	const std::string kAutoNameRight = "Right";
	const std::string kAutoNameCenter = "Center";
	std::string m_autoSelected;
	std::string gameData;

	unsigned int driveState;

	double left;
	double right;
	double turn;
	double driveSpeed;
	double clawLeftSpeed;
	double clawRightSpeed;
	double ClawSpeed;
	double ClawHold;
	double ElevatorSpeed;

	bool isClawHomed = false;
	bool isElevatorHomed = false;
	bool flagElevatorDown = false;
	bool flagElevatorMid = false;
	bool flagElevatorHigh = false;
	bool flagClawUp = false;
};
