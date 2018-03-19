/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

void Robot::MotorBuilder(WPI_TalonSRX *srx, bool brake = true, bool inverted = false, double RampTime = 0, int CurrentLimit = 20,
		int MaxCurrent = 20, int MaxTime = 100) {
	srx->SetInverted(inverted);
	srx->ConfigOpenloopRamp(RampTime, kTimeoutMs);
	srx->ConfigContinuousCurrentLimit(CurrentLimit, kTimeoutMs);
	srx->ConfigPeakCurrentLimit(MaxCurrent, kTimeoutMs);
	srx->ConfigPeakCurrentDuration(MaxTime, kTimeoutMs);
	if (CurrentLimit > 0) {
		srx->EnableCurrentLimit(true);
	}
}

void Robot::FindLimits() { //convert from while to if loops so it doesn't stop robot
	//find Claw up position
	Claw->Set(ControlMode::PercentOutput, 0.04);
	while (!ClawSensor->GetGeneralInput(ClawSensor->LIMF)) { //TODO not sure if switches are NormallyOpen
		if (IsAutonomous()) {
			if (time->Get() >= kAutopausetime) {
				goto clawEmergencyAutoBreak;
			}
		}
		Wait(0.005);
	}
	Claw->SetSelectedSensorPosition(kClawEncoderKnownHigh,/*REMOTE*/0,/*TimeOut*/0);
	clawEmergencyAutoBreak:

	Claw->Set(ControlMode::Position, 0);	//move claw down

	//find Elevator down position
	Elevator1->Set(ControlMode::PercentOutput, -0.04);
	while (!Elevator1->GetSensorCollection().IsRevLimitSwitchClosed()) {
		if (IsAutonomous()) {
			if (time->Get() >= kAutopausetime) {
				goto elevatorEmergencyAutoBreak;
			}
		}
		Wait(0.005);
	}
	Elevator1->SetSelectedSensorPosition(kElevatorEncoderKnownLow,/*REMOTE*/0,/*TimeOut*/0);
	elevatorEmergencyAutoBreak:

	Elevator1->Set(ControlMode::Position, 0);	//move claw down
}

void Robot::RGB(double R, double G, double B, CANifier *can){//It is GRB
	can->SetLEDOutput(/*percent*/G,CANifier::LEDChannelA);
	can->SetLEDOutput(/*percent*/R,CANifier::LEDChannelB);
	can->SetLEDOutput(/*percent*/B,CANifier::LEDChannelC);
}

void Robot::RobotInit() {
	m_chooser.AddDefault(kAutoNameDefault, kAutoNameDefault);
	m_chooser.AddObject(kAutoNameCustom, kAutoNameCustom);
	frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
	driveState = 0;	//set to tank

	//Setting the Controllers
	Joystick1 = new Joystick(0);
	Joystick2 = new Joystick(1);

	gyro = new ADXRS450_Gyro(SPI::kOnboardCS0);
	PDP = new PowerDistributionPanel(0);
	accel = new BuiltInAccelerometer();
	time = new Timer();

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
	ElevatorSolenoid = new DoubleSolenoid(/*PCM Number*/0, /*forward Channel*/0,/*reverse Channel*/1);

	ClawSensor = new CANifier(21);

	//follow
	DBLeft2->Set(ControlMode::Follower, DBLeft->GetDeviceID());
	DBRight2->Set(ControlMode::Follower, DBRight->GetDeviceID());
	Elevator2->Set(ControlMode::Follower, Elevator1->GetDeviceID());
	Elevator3->Set(ControlMode::Follower, Elevator1->GetDeviceID());

	//Motor Builder(&Motor,brake,invert,Ramp,limit,maxlimit,maxtime)
	MotorBuilder(DBLeft, /*brake*/true,/*invert*/true, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBLeft2, /*brake*/true,/*invert*/true, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight, /*brake*/true,/*invert*/false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight2, /*brake*/true,/*invert*/false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(Claw, /*brake*/true,/*invert*/false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawLeft, /*brake*/true,/*invert*/false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawRight, /*brake*/true,/*invert*/true, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(Elevator1, /*brake*/true,/*invert*/false, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator2, /*brake*/true,/*invert*/false, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator3, /*brake*/true,/*invert*/false, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);

	//Add CANifier encoder
	ClawSensor->ConfigVelocityMeasurementPeriod(CANifierVelocityMeasPeriod::Period_100Ms, kTimeoutMs);
	ClawSensor->ConfigVelocityMeasurementWindow(64, kTimeoutMs);
	ClawSensor->SetStatusFramePeriod(CANifierStatusFrame::CANifierStatusFrame_Status_2_General, /*refresh rate*/10, kTimeoutMs); /* speed up quadrature DIO */

	//attach CANifier to Claw motor
	Claw->ConfigRemoteFeedbackFilter(ClawSensor->GetDeviceNumber(), RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature,/*REMOTE*/
	0, kTimeoutMs);
	Claw->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off,/*REMOTE*/1, kTimeoutMs); //turn off second sensor for claw
	Claw->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0,/*PID_PRIMARY*/0, kTimeoutMs);
	//Claw->SetSensorPhase(true); //Sensor Invert? TODO
	Claw->ConfigForwardLimitSwitchSource(RemoteLimitSwitchSource_RemoteCANifier, LimitSwitchNormal_NormallyOpen,
			ClawSensor->GetDeviceNumber(), 0);

	//TODO Claw PID See 10.1 set P=1 I=10+ maybe don't override but use website
	//Claw->Config_kP(/*slot*/0, 1, kTimeoutMs);
	//Claw->Config_kI(/*slot*/0, 10, kTimeoutMs);

	//TODO create encoder limits
	/*Claw->ConfigForwardSoftLimitThreshold(clawForwardLimit, kTimeoutMs);
	 Claw->ConfigForwardSoftLimitEnable(true, kTimeoutMs);
	 Claw->ConfigReverseSoftLimitThreshold(clawReverseLimit, kTimeoutMs);
	 Claw->ConfigReverseSoftLimitEnable(true, kTimeoutMs);*/

	//elevator sensors
	Elevator1->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off,/*REMOTE*/0, kTimeoutMs);
	Elevator1->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off,/*REMOTE*/1, kTimeoutMs);
	Elevator1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTimeoutMs);
	Elevator1->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);
	//Elevator1->Config_kP(/*slot*/0, 0.5, kTimeoutMs);
	//Elevator1->Config_kI(/*slot*/0, 0.2, kTimeoutMs);

	drive = new DifferentialDrive(*DBLeft, *DBRight);
	ElevatorSolenoid->Set(DoubleSolenoid::Value::kOff);
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
	time->Reset(); //don't know if this is needed (maybe not)
	time->Start();

	//FindLimits(); TODO uncomment when limit switches are installed //TODO convert so it is in AutonomousPeriodic
	while (time->Get() >= kAutopausetime)
		; //pause until kAutopausetime seconds has passed since timer started

	m_autoSelected = m_chooser.GetSelected();	//Java SmartDashboard
	//m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault); //LabVIEW Dashboard
	std::cout << "Auto selected: " << m_autoSelected << std::endl;

	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if(frc::DriverStation::GetInstance().GetAlliance()==DriverStation::kRed){
		RGB(0,0,0,ClawSensor);
	}else if(frc::DriverStation::GetInstance().GetAlliance()==DriverStation::kBlue){
		RGB(0,50,0,ClawSensor);
	}
	/*
	 if (gameData.length > 0) {
	 if (gameData[0] == 'L') {
	 //Put left auto code here
	 } else {
	 //Put right auto code here
	 }
	 }
	 */
	if (m_autoSelected == kAutoNameCustom) {
		// No Auto goes here
	} else {
		// Default Auto goes here
		// Wait(7); unneeded because of while loop above
		DBLeft->Set(.5);
		DBRight->Set(.5);
		Wait(3.5);
		DBLeft->Set(0);
		DBRight->Set(0);
	} //TODO figure out how to turn 90 degree in auto with time and encoder
}

void Robot::AutonomousPeriodic() {
	if (m_autoSelected == kAutoNameCustom) {
		// No Auto goes here
	} else {
		// Default Auto goes here
	}
}

void Robot::TeleopInit() {
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

}

void Robot::TeleopPeriodic() {
	if (Joystick1->GetRawButtonPressed(PS4::Share)) {
		++driveState %= 3;	//increment and reset to 0 if 3
		switch (driveState) {
		case 0:
			std::cout << "Tank Drive" << std::endl;
			break;
		case 1:
			std::cout << "Arcade Drive" << std::endl;
			break;
		case 2:
			std::cout << "Curvature Drive" << std::endl;
		}
	}
	switch (driveState) {
	case 0:
		left = Joystick1->GetRawAxis(PS4::PSLeftStickDown) * -1;
		right = Joystick1->GetRawAxis(PS4::PSRightStickDown) * -1;
		drive->TankDrive(left, right, /*Squared Inputs*/true);
		break;
	case 1:
		left = Joystick1->GetRawAxis(PS4::PSRightStickDown) * -1;
		right = Joystick1->GetRawAxis(PS4::PSLeftStickRight);
		drive->ArcadeDrive(left, right, /*Squared Inputs*/true);
		break;
	case 2:
		left = Joystick1->GetRawAxis(PS4::PSRightStickDown) * -1;
		right = Joystick1->GetRawAxis(PS4::PSLeftStickRight);
		drive->CurvatureDrive(left, right,/*quick turn*/Joystick1->GetRawButtonPressed(PS4::R3));
	}
	/*
	 //TODO elevator buttons set levels
	 if (Joystick1->GetRawButtonPressed(PS4::Square)){
	 Elevator1->Set(ControlMode::Position, 0);
	 }
	 //TODO claw buttons set levels
	 if (Joystick1->GetPOV()==180){
	 Claw->Set(ControlMode::Position, down);
	 }else if(Joystick1->GetPOV()==0){
	 Claw->Set(ControlMode::Position, up);
	 }
	 */
	//Claw intakes
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

	frc::SmartDashboard::PutNumber("Gyroscope", gyro->GetAngle());
	frc::SmartDashboard::PutNumber("POV", Joystick1->GetPOV());
	frc::SmartDashboard::PutNumber("Elevator", Elevator1->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("Elevator Pulse", Elevator1->GetSensorCollection().GetPulseWidthPosition());
	frc::SmartDashboard::PutNumber("Elevator Quad", Elevator1->GetSensorCollection().GetQuadraturePosition());
	frc::SmartDashboard::PutNumber("Claw", Claw->GetSelectedSensorPosition(0));
	frc::SmartDashboard::PutNumber("Claw Pulse", Claw->GetSensorCollection().GetPulseWidthPosition());
	frc::SmartDashboard::PutNumber("Claw Quad", Claw->GetSensorCollection().GetQuadraturePosition());
	frc::SmartDashboard::PutNumber("Claw Forward Limit", ClawSensor->GetGeneralInput(ClawSensor->LIMF));
	frc::SmartDashboard::PutNumber("Elevator Reverse Limit", Elevator1->GetSensorCollection().IsRevLimitSwitchClosed());

	//just for testing
	Claw->Set(ControlMode::PercentOutput, Joystick2->GetRawAxis(Attack::Up) * -1);
	Elevator1->Set(ControlMode::PercentOutput, Joystick2->GetRawAxis(Attack::Throttle));
}

void Robot::DisabledInit() {
	RGB(25,25,25,ClawSensor);
}

void Robot::DisabledPeriodic() {
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	gyro->Calibrate();	//keep on Calibrating gyro while disabled so it is always Calibrated when needed
	Wait(1);
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

void Robot::RobotPeriodic() {
}

START_ROBOT_CLASS(Robot)
