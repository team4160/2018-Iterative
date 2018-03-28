/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* Rules
 * 1: If you require sensors then have a method to timeout if sensor has failed
 * 2: Avoid while loops (stops the robot from processing anything else and motors will continue spinning)
 * 3: Do not feature creep
 * 4: Test your changes when possible
 * 5: If using the same code over and over then make a function
 */

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
	Timer temptimer;
	temptimer.Start();
	isHomed = true;
	Claw->Set(ControlMode::PercentOutput, 0.04);	//move up by 4%
	while (!ClawSensor->GetGeneralInput(ClawSensor->LIMF)) { //TODO not sure if switches are NormallyOpen
		if (IsAutonomous()) {
			if (mytimer->Get() >= kAutopausetime) {
				isHomed = false;
				goto clawEmergencyBreak;
			}
		}
		if (temptimer.Get() > 5/*seconds*/) { //TODO find good timeout time
			isHomed = false;
			goto clawEmergencyBreak;
		}
		Wait(0.005);
	}
	Claw->SetSelectedSensorPosition(kClawEncoderKnownHigh, /*REMOTE*/0, /*TimeOut*/0);
	clawEmergencyBreak:	//if broken out then don't set or move (will need to move manually)

//find Elevator down position
	Elevator1->Set(ControlMode::PercentOutput, -0.04);	//move down by 4%
	while (!Elevator1->GetSensorCollection().IsRevLimitSwitchClosed()) {
		if (IsAutonomous()) {
			if (mytimer->Get() >= kAutopausetime) {
				isHomed = false;
				goto elevatorEmergencyBreak;
			}
		}
		if (temptimer.Get() > 5/*seconds*/) {	//TODO find good timeout time
			isHomed = false;
			goto elevatorEmergencyBreak;
		}
		Wait(0.005);
	}
	Elevator1->SetSelectedSensorPosition(kElevatorEncoderKnownLow, /*REMOTE*/0, /*TimeOut*/0);
	Elevator1->Set(ControlMode::Position, 0);	//move claw down
	elevatorEmergencyBreak: ;
}

void Robot::RGB(double R, double G, double B, CANifier *can) {	//Normally It is GRB
	can->SetLEDOutput(/*percent*/G, CANifier::LEDChannelA);
	can->SetLEDOutput(/*percent*/R, CANifier::LEDChannelB);
	can->SetLEDOutput(/*percent*/B, CANifier::LEDChannelC);
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
	mytimer = new Timer();

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
	ElevatorSolenoid = new DoubleSolenoid(/*PCM Number*/0, /*forward Channel*/0, /*reverse Channel*/1);

	ClawSensor = new CANifier(21);

//follow
	DBLeft2->Set(ControlMode::Follower, DBLeft->GetDeviceID());
	DBRight2->Set(ControlMode::Follower, DBRight->GetDeviceID());
	Elevator2->Set(ControlMode::Follower, Elevator1->GetDeviceID());
	Elevator3->Set(ControlMode::Follower, Elevator1->GetDeviceID());

//Motor Builder(&Motor,brake,invert,Ramp,limit,maxlimit,maxtime)
	MotorBuilder(DBLeft, /*brake*/true, /*invert*/false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBLeft2, /*brake*/true, /*invert*/false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime); //DriveBase already negates right side
	MotorBuilder(DBRight, /*brake*/true, /*invert*/false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight2, /*brake*/true, /*invert*/false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(Claw, /*brake*/true, /*invert*/false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawLeft, /*brake*/true, /*invert*/false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawRight, /*brake*/true, /*invert*/true, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(Elevator1, /*brake*/true, /*invert*/false, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator2, /*brake*/true, /*invert*/false, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator3, /*brake*/true, /*invert*/false, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);

//Add CANifier encoder
	ClawSensor->ConfigVelocityMeasurementPeriod(CANifierVelocityMeasPeriod::Period_100Ms, kTimeoutMs);
	ClawSensor->ConfigVelocityMeasurementWindow(64, kTimeoutMs);
	ClawSensor->SetStatusFramePeriod(CANifierStatusFrame::CANifierStatusFrame_Status_2_General, /*refresh rate*/10, kTimeoutMs); /* speed up quadrature DIO */

//attach CANifier to Claw motor
	Claw->ConfigRemoteFeedbackFilter(ClawSensor->GetDeviceNumber(), RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature, /*REMOTE*/
	0, kTimeoutMs);
	Claw->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off, /*REMOTE*/1, kTimeoutMs); //turn off second sensor for claw
	Claw->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, /*PID_PRIMARY*/0, kTimeoutMs);
//Claw->SetSensorPhase(true); //TODO Sensor Invert?
	Claw->ConfigForwardLimitSwitchSource(RemoteLimitSwitchSource_RemoteCANifier, LimitSwitchNormal_NormallyOpen,
			ClawSensor->GetDeviceNumber(), 0);

//TODO Claw PID See 10.1 set P=1 I=10+ maybe don't override but use website for now
//Claw->Config_kP(/*slot*/0, 1, kTimeoutMs);
//Claw->Config_kI(/*slot*/0, 10, kTimeoutMs);

//TODO create soft encoder limits when you found positions
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
	drive->SetDeadband(0.03);
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
	mytimer->Reset(); //If we start auto a second time
	mytimer->Start();

	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
	if (frc::DriverStation::GetInstance().GetAlliance() == DriverStation::kRed)
		RGB(50, 0, 0, ClawSensor);
	else if (frc::DriverStation::GetInstance().GetAlliance() == DriverStation::kBlue)
		RGB(0, 0, 50, ClawSensor);
	else
		RGB(0, 50, 0, ClawSensor);

//FindLimits(); TODO uncomment when limit switches are installed //TODO convert so it is in AutonomousPeriodic

	while (mytimer->Get() >= kAutopausetime)
		; //pause until kAutopausetime seconds has passed since timer started

	m_autoSelected = m_chooser.GetSelected();	//Java SmartDashboard
//m_autoSelected = SmartDashboard::GetString("Auto Selector", kAutoNameDefault); //LabVIEW Dashboard
	std::cout << "Auto selected: " << m_autoSelected << std::endl;
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
		// No Auto
	} else {
		// Default Auto
		// Wait(7); unneeded because of while loop above
		drive->TankDrive(0.5, 0.5, false);
		Wait(3.5);
		drive->TankDrive(0, 0, false);
	} //TODO figure out how to turn 90 degree in auto with time(timeout) and (encoder or gyro)
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

	if (isHomed) {
		Claw->Set(ControlMode::Position, 0);	//move claw down if homed
		Elevator1->Set(ControlMode::Position, 0);	//move claw down if homed
	} else {
		Claw->SetSelectedSensorPosition(kClawEncoderKnownHigh, /*REMOTE*/0, /*TimeOut*/0);
		Claw->Set(ControlMode::Position, 0);
		Elevator1->Set(ControlMode::Position, 0);
		Elevator1->SetSelectedSensorPosition(kElevatorEncoderKnownLow, /*REMOTE*/0, /*TimeOut*/0);
	}
}

void Robot::TeleopPeriodic() {
	if (Joystick1->GetRawButtonPressed(PS4::Options)) {
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
		right = Joystick1->GetRawAxis(PS4::PSRightStickDown) * -1;
		left = Joystick1->GetRawAxis(PS4::PSLeftStickRight);
		drive->ArcadeDrive(right, left, /*Squared Inputs*/true);
		break;
	case 2:
		right = Joystick1->GetRawAxis(PS4::PSRightStickDown) * -1;
		left = Joystick1->GetRawAxis(PS4::PSLeftStickRight);
		drive->CurvatureDrive(right, left,/*quick turn*/Joystick1->GetRawButtonPressed(PS4::R3));
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

//elevator
	//TODO

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

//just for testing TODO delete after testing and Dashboard code above
	Claw->Set(ControlMode::PercentOutput, Joystick2->GetRawAxis(Attack::Down) * -1);
	Elevator1->Set(ControlMode::PercentOutput, Joystick2->GetRawAxis(Attack::ReverseThrottle) * -1);
	if (Joystick1->GetRawButtonPressed(PS4::Pad)) {
		FindLimits();
	}
}

void Robot::DisabledInit() {
	RGB(25, 25, 25, ClawSensor);
	mytimer->Stop();
	mytimer->Reset();
}

void Robot::DisabledPeriodic() {
	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage(); //always get latest game data
	gyro->Calibrate(); //keep on Calibrating gyro while disabled so it is always Calibrated when needed
	Wait(1);
}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

void Robot::RobotPeriodic() {
}

START_ROBOT_CLASS(Robot)
