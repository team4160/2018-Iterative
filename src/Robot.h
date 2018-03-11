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
#include <SmartDashboard/SmartDashboard.h>
#include <string>
#include <IterativeRobot.h>
#include <SmartDashboard/SendableChooser.h>

class Robot: public frc::IterativeRobot {
public:
	Joystick *Joystick1, *Joystick2;
	WPI_TalonSRX *DBLeft, *DBLeft2;
	WPI_TalonSRX *DBRight, *DBRight2;
	WPI_TalonSRX *Claw, *ClawLeft, *ClawRight;
	WPI_TalonSRX *Elevator1,*Elevator2,*Elevator3;
	void MotorBuilder(WPI_TalonSRX *srx, bool brake, bool inverted,
			double RampTime, int CurrentLimit, int MaxCurrent, int MaxTime);
	void RobotInit() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

private:
	frc::SendableChooser<std::string> m_chooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	std::string m_autoSelected;
};
