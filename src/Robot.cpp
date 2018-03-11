/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::MotorBuilder(WPI_TalonSRX *srx, bool brake, bool inverted,
		double RampTime = 0, int CurrentLimit = 0, int MaxCurrent = 0,
		int MaxTime = 0) {
	srx->SetInverted(inverted);
	if (RampTime != 0) {
		srx->ConfigOpenloopRamp(RampTime, 0);
	}
	if (CurrentLimit != 0) {
		srx->ConfigContinuousCurrentLimit(CurrentLimit, 0);
		srx->ConfigPeakCurrentLimit(MaxCurrent, 0);
		srx->ConfigPeakCurrentDuration(MaxTime, 0);
		srx->EnableCurrentLimit(true);
	}
}

void Robot::RobotInit() {
	m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

	//Setting the Controllers
	Joystick1 = new Joystick(0);
	Joystick2 = new Joystick(1);

	DBLeft = new WPI_TalonSRX(1);
	DBLeft2 = new WPI_TalonSRX(2);
	DBRight = new WPI_TalonSRX(3);
	DBRight2 = new WPI_TalonSRX(4);
	Claw = new WPI_TalonSRX(5);
	ClawLeft = new WPI_TalonSRX(6);
	ClawRight = new WPI_TalonSRX(7);
	Elevator1 = new WPI_TalonSRX(8);
	Elevator2 = new WPI_TalonSRX(9);
	Elevator3 = new WPI_TalonSRX(10);

	//follow
	DBLeft2->Set(ControlMode::Follower, DBLeft->GetDeviceID());
	DBRight2->Set(ControlMode::Follower, DBRight->GetDeviceID());
	Elevator2->Set(ControlMode::Follower, Elevator1->GetDeviceID());
	Elevator3->Set(ControlMode::Follower, Elevator1->GetDeviceID());

	//Setting up the TalonSRXs
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

	//Motor Builder(&Motor,brake,invert,Ramp,limit,maxlimit,maxtime)
	MotorBuilder(DBLeft, true, true, driveRampTime, driveCurrentLimit,
			driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBLeft2, true, true, driveRampTime, driveCurrentLimit,
			driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight, true, false, driveRampTime, driveCurrentLimit,
			driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight2, true, false, driveRampTime, driveCurrentLimit,
			driveMaxCurrent, driveMaxTime);
	MotorBuilder(Claw, true, false, clawRampTime, clawCurrentLimit,
			clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawLeft, true, false, clawRampTime, clawCurrentLimit,
			clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawRight, true, true, clawRampTime, clawCurrentLimit,
			clawMaxCurrent, clawMaxTime);
	MotorBuilder(Elevator1, true, false, elevatorRampTime, elevatorCurrentLimit,
			elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator2, true, false, elevatorRampTime, elevatorCurrentLimit,
			elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator3, true, false, elevatorRampTime, elevatorCurrentLimit,
			elevatorMaxCurrent, elevatorMaxTime);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDas	hboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
	m_autoSelected = m_chooser.GetSelected();
	// m_autoSelected = SmartDashboard::GetString(
	// 		"Auto Selector", kAutoNameDefault);
	std::cout << "Auto selected: " << m_autoSelected << std::endl;

	if (m_autoSelected == kAutoNameCustom) {
		// Custom Auto goes here
	} else {
		// Default Auto goes here
		Wait(7);
		DBLeft->Set(.5);
		DBRight->Set(.5);
		Wait(3.5);
		DBLeft->Set(0);
		DBRight->Set(0);
	}
}

void Robot::AutonomousPeriodic() {
	if (m_autoSelected == kAutoNameCustom) {
		// Custom Auto goes here
	} else {
		// Default Auto goes here
	}
}

void Robot::TeleopInit() {
}

void Robot::TeleopPeriodic() {
	double left = Joystick1->GetRawAxis(1) * -1;
	double right = Joystick1->GetRawAxis(5) * -1;
	DBLeft->Set(left);
	DBRight->Set(right);
	/*Elevator1->Set(Joystick2->GetRawAxis(1) * -1);
	if (Joystick2->GetRawButton(3))
		Claw->Set(.25);
	else if (Joystick2->GetRawButton(2))
		Claw->Set(-.25);
	else
		Claw->Set(.07);*/
	Claw->Set(Joystick2->GetRawAxis(1) * -1);
	if (Joystick2->GetRawButton(6)) {
		ClawLeft->Set(1);
		ClawRight->Set(1);
	} else if (Joystick2->GetRawButton(7)) {
		ClawLeft->Set(-1);
		ClawRight->Set(-1);
	} else {
		ClawLeft->Set(0);
		ClawRight->Set(0);
	}
}

void Robot::TestPeriodic() {
}

START_ROBOT_CLASS(Robot)
