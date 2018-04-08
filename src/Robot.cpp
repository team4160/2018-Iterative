/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* Rules
 * 1: If you require sensors then have a method to timeout if sensor has failed
 * 2: Avoid while loops (stops the robot from processing anything else)
 * 3: If using the same code over and over then make a function
 * 4: Test your changes when possible
 * 5: Do not feature creep
 */

#include "Robot.h"

void Robot::MotorBuilder(WPI_TalonSRX *srx, bool brake = true, bool inverted = false, double RampTime = 0, int CurrentLimit = 20, int MaxCurrent = 20, int MaxTime = 100) {
	srx->SetInverted(inverted);
	srx->ConfigOpenloopRamp(RampTime, kTimeoutMs);
	srx->ConfigContinuousCurrentLimit(CurrentLimit, kTimeoutMs);
	srx->ConfigPeakCurrentLimit(MaxCurrent, kTimeoutMs);
	srx->ConfigPeakCurrentDuration(MaxTime, kTimeoutMs);
	if (CurrentLimit > 0) {
		srx->EnableCurrentLimit(true);
	} else {
		srx->EnableCurrentLimit(false);
	}
}

//void Robot::RGB(double R, double G, double B, CANifier *can) {	//Normally It is GRB
//	can->SetLEDOutput(/*percent*/G, CANifier::LEDChannelA);
//	can->SetLEDOutput(/*percent*/R, CANifier::LEDChannelB);
//	can->SetLEDOutput(/*percent*/B, CANifier::LEDChannelC);
//}
//
//void Robot::SetAllianceColor() {
//	if (frc::DriverStation::GetInstance().GetAlliance() == DriverStation::kRed)
//		RGB(50, 0, 0, ClawSensor);
//	else if (frc::DriverStation::GetInstance().GetAlliance() == DriverStation::kBlue)
//		RGB(0, 0, 50, ClawSensor);
//	else
//		RGB(0, 50, 0, ClawSensor);
//}

//void Robot::homeElevator() {
//	if ((PDP->GetCurrent(kPDP::Elevator1) > elevatorSenseCurrent) | (PDP->GetCurrent(kPDP::Elevator2) > elevatorSenseCurrent)
//			| (PDP->GetCurrent(kPDP::Elevator3) > elevatorSenseCurrent)) {
//		isElevatorHomed = true;	//for auto
//		flagElevatorDown = false;	//for teleop
//		Elevator1->SetSelectedSensorPosition(kElevatorEncoderKnownLow, /*REMOTE*/0, /*TimeOut*/0);
//		Elevator1->Set(ControlMode::Position, kElevatorEncoderKnownLow);	//hold position
//	}
//}
//
//void Robot::homeClaw() {
//	if (PDP->GetCurrent(kPDP::Claw) > clawSenseCurrent) {
//		isClawHomed = true;	//for auto
//		flagClawUp = false;	//for teleop
////		Claw->Set(ControlMode::PercentOutput, 0);
//		Claw->SetSelectedSensorPosition(kClawEncoderKnownHigh, /*REMOTE*/0, /*TimeOut*/0);
////		ClawSensor->SetQuadraturePosition(kClawEncoderKnownHigh, 0);
////		Claw->Set(ControlMode::Position, kClawEncoderKnownHigh);
//		Claw->Set(ControlMode::PercentOutput, 0);
//	}
//}

void Robot::RobotInit() {
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
	Claw2 = new WPI_TalonSRX(11);
//	ElevatorSolenoid = new DoubleSolenoid(/*PCM Number*/0, /*forward Channel*/0, /*reverse Channel*/1);

	ClawSensor = new CANifier(21);

//set followers
	DBLeft2->Set(ControlMode::Follower, DBLeft->GetDeviceID());
	DBRight2->Set(ControlMode::Follower, DBRight->GetDeviceID());
	Elevator2->Set(ControlMode::Follower, Elevator1->GetDeviceID());
	Elevator3->Set(ControlMode::Follower, Elevator1->GetDeviceID());
	Claw2->Set(ControlMode::Follower, Claw->GetDeviceID());

//Motor Builder(&Motor,brake,invert,Ramp,limit,maxlimit,maxtime)
	MotorBuilder(DBLeft, /*brake*/true, /*invert*/false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBLeft2, /*brake*/true, /*invert*/false, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight, /*brake*/true, /*invert*/true, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(DBRight2, /*brake*/true, /*invert*/true, driveRampTime, driveCurrentLimit, driveMaxCurrent, driveMaxTime);
	MotorBuilder(Claw, /*brake*/true, /*invert*/false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(Claw2, /*brake*/true, /*invert*/false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawLeft, /*brake*/true, /*invert*/false, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(ClawRight, /*brake*/true, /*invert*/true, clawRampTime, clawCurrentLimit, clawMaxCurrent, clawMaxTime);
	MotorBuilder(Elevator1, /*brake*/true, /*invert*/true, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator2, /*brake*/true, /*invert*/true, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);
	MotorBuilder(Elevator3, /*brake*/true, /*invert*/true, elevatorRampTime, elevatorCurrentLimit, elevatorMaxCurrent, elevatorMaxTime);

//	DBLeft->SetSensorPhase(true);
//	DBRight->SetSensorPhase(true);

//Add CANifier encoder
	ClawSensor->ConfigVelocityMeasurementPeriod(CANifierVelocityMeasPeriod::Period_100Ms, kTimeoutMs);
	ClawSensor->ConfigVelocityMeasurementWindow(64, kTimeoutMs);
	ClawSensor->SetStatusFramePeriod(CANifierStatusFrame::CANifierStatusFrame_Status_2_General, /*refresh rate*/10, kTimeoutMs); /* speed up quadrature DIO */

//attach CANifier to Claw motor
	Claw->ConfigRemoteFeedbackFilter(ClawSensor->GetDeviceNumber(), RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature, /*REMOTE*/
	0, kTimeoutMs);
	Claw->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off, /*REMOTE*/1, kTimeoutMs); //turn off second sensor for claw
	Claw->ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0, /*PID_PRIMARY*/0, kTimeoutMs);
	Claw->SetSensorPhase(true); //TODO Sensor Invert?
//	Claw->ConfigReverseLimitSwitchSource(RemoteLimitSwitchSource_RemoteCANifier, LimitSwitchNormal_NormallyOpen, ClawSensor->GetDeviceNumber(), 0);

//TODO Claw PID See 10.1 set P=1 I=10+ maybe don't override but use website for now
	Claw->Config_kP(/*slot*/0, 1, kTimeoutMs);
//	Claw->Config_kD(/*slot*/0, 5, kTimeoutMs);

//TODO create soft encoder limits when you found positions
//	Claw->ConfigForwardSoftLimitThreshold(clawForwardLimit, kTimeoutMs);
//	Claw->ConfigForwardSoftLimitEnable(true, kTimeoutMs);
//	Claw->ConfigReverseSoftLimitThreshold(clawReverseLimit, kTimeoutMs);
//	Claw->ConfigReverseSoftLimitEnable(true, kTimeoutMs);

//elevator sensors
	Elevator1->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off,/*REMOTE*/0, kTimeoutMs);
	Elevator1->ConfigRemoteFeedbackFilter(0x00, RemoteSensorSource::RemoteSensorSource_Off,/*REMOTE*/1, kTimeoutMs);
	Elevator1->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTimeoutMs);
//	Elevator1->ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 0);

//	Elevator1->Config_kP(/*slot*/0, 0.5, kTimeoutMs);
//	Elevator1->Config_kD(/*slot*/0, 5, kTimeoutMs);

//	ElevatorSolenoid->Set(DoubleSolenoid::Value::kOff);

//	CameraServer::GetInstance()->SetSize(CameraServer::kSize320x240);
	CameraServer::GetInstance()->StartAutomaticCapture();
	Elevator1->SetSelectedSensorPosition(0, /*REMOTE*/0, /*TimeOut*/0);
//	Elevator1->SetSensorPhase(true);
	Elevator1->ConfigForwardSoftLimitThreshold(-600, 0);
	Elevator1->ConfigForwardSoftLimitEnable(true, 0);
	Elevator1->ConfigReverseSoftLimitThreshold(-31000, kTimeoutMs);
	Elevator1->ConfigReverseSoftLimitEnable(true, kTimeoutMs);
//	Elevator1->ConfigForwardSoftLimitEnable(false, 0);
//	Elevator1->ConfigReverseSoftLimitEnable(false, 0);

	gyro->Calibrate(); //takes around 5 seconds to execute and must not move
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
	mytimer->Reset();
	mytimer->Start();
	gyro->Reset();

//	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

//	SetAllianceColor();

	/*
	 if (gameData.length > 0) {
	 if (gameData[0] == 'L') {
	 //Put left auto code here
	 } else {
	 //Put right auto code here
	 }
	 }
	 */

//TODO figure out how to turn 90 degree in auto with time(timeout) and (gyro or encoder)
//	Claw->Set(ControlMode::PercentOutput, 0.05);	//move up by 5%
//	Elevator1->Set(ControlMode::PercentOutput, -0.05);	//move down by 5%
}

void Robot::AutonomousPeriodic() {
//	if (!isElevatorHomed) {
//		homeElevator();
//	}
//	if (!isClawHomed) {
//		homeClaw();
//	}
	if ( (mytimer->Get() < 3)) {
		DBLeft->Set(0.6);
		DBRight->Set(0.6);
	} else {
		DBLeft->Set(0);
		DBRight->Set(0);
	}
}

void Robot::TeleopInit() {
//	gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();
//	SetAllianceColor();
}

void Robot::TeleopPeriodic() {
	if (Joystick1->GetRawButtonPressed(XB1::View)) {
		++driveState %= 3;	//increment and reset to 0 if 3
		switch (driveState) {
		case 0:	//Tank
			std::cout << "Tank Drive" << std::endl;
			frc::SmartDashboard::PutString("Drive Mode", "Tank Drive");
			break;
		case 1:	//Arcade
			std::cout << "Arcade Drive" << std::endl;
			frc::SmartDashboard::PutString("Drive Mode", "Arcade Drive");
		}
	}
	switch (driveState) {
	case 0:	//Tank
		left = Joystick1->GetRawAxis(XB1::XBLeftStickDown);
		right = Joystick1->GetRawAxis(XB1::XBRightStickDown);
		DBLeft->Set(left);
		DBRight->Set(right);
		break;
	case 1:	//Arcade
		left = (Joystick1->GetRawAxis(XB1::XBRightStickRight));
		driveSpeed = (Joystick1->GetRawAxis(XB1::XBLeftStickDown));

		turn = ((turnSensitivity * left * left * left) + (1 - turnSensitivity) * left);
		DBLeft->Set(driveSpeed - turn);
		DBRight->Set(driveSpeed + turn);
	}

//Claw intakes

	clawRightSpeed = clawLeftSpeed = ((Joystick2->GetRawAxis(PS4::L2In) + 1) / -2);
	clawLeftSpeed += ((Joystick2->GetRawAxis(PS4::R2In) + 1) / 2);
	clawRightSpeed += ((Joystick2->GetRawAxis(PS4::R2In) + 1) / 2);
	if (Joystick2->GetRawButton(PS4::L1))
		clawLeftSpeed = -.25;
	if (Joystick2->GetRawButton(PS4::R1))
		clawRightSpeed = -.25;
	ClawLeft->Set(clawLeftSpeed);
	ClawRight->Set(clawRightSpeed);

//	if (Joystick2->GetRawButtonPressed(2)) {	//elevator low
//		Elevator1->Set(ControlMode::PercentOutput, -0.35);
//		flagElevatorDown = true;
//	}
//	if (Joystick2->GetRawButtonPressed(4)) {	//elevator mid
//		Elevator1->Set(ControlMode::Position, kElevatorEncoderMiddle);
//	}
//	if (Joystick2->GetRawButtonPressed(3)) {	//elevator high
//		Elevator1->Set(ControlMode::Position, kElevatorEncoderHigh);
//	}
//	if (Joystick2->GetRawButton(PS4::Cross)) {	//claw down
//		Claw->Set(ControlMode::Position, -200);
//	}
//	if (Joystick2->GetRawButton(PS4::Triangle)) {	//claw up
//		Claw->Set(ControlMode::PercentOutput, -0.4);
//		flagClawUp = true;
//	}
//	if (flagElevatorDown) {	//sense elevator down with current
//		homeElevator();
//	}
//	if (flagClawUp) {	//sense claw up with current
//		homeClaw();
//	}

//	frc::SmartDashboard::PutNumber("Gyroscope", gyro->GetAngle());
//	frc::SmartDashboard::PutNumber("POV", Joystick2->GetPOV());
//	frc::SmartDashboard::PutNumber("Drive Left", DBLeft->GetSelectedSensorPosition(0));
//	frc::SmartDashboard::PutNumber("Drive Left Pulse", DBLeft->GetSensorCollection().GetPulseWidthPosition());
//	frc::SmartDashboard::PutNumber("Drive Left Quad", DBLeft->GetSensorCollection().GetQuadraturePosition());
//	frc::SmartDashboard::PutNumber("Drive Right", DBRight->GetSelectedSensorPosition(0));
//	frc::SmartDashboard::PutNumber("Drive Right Pulse", DBRight->GetSensorCollection().GetPulseWidthPosition());
//	frc::SmartDashboard::PutNumber("Drive Right Quad", DBRight->GetSensorCollection().GetQuadraturePosition());
//	frc::SmartDashboard::PutNumber("Elevator", Elevator1->GetSelectedSensorPosition(0));
//	frc::SmartDashboard::PutNumber("Elevator Pulse", Elevator1->GetSensorCollection().GetPulseWidthPosition());
//	frc::SmartDashboard::PutNumber("Elevator Quad", Elevator1->GetSensorCollection().GetQuadraturePosition());
//	frc::SmartDashboard::PutNumber("Elevator Reverse Limit", Elevator1->GetSensorCollection().IsRevLimitSwitchClosed());
	frc::SmartDashboard::PutNumber("Claw", Claw->GetSelectedSensorPosition(0));
//	frc::SmartDashboard::PutNumber("Claw Pulse", Claw->GetSensorCollection().GetPulseWidthPosition());
//	frc::SmartDashboard::PutNumber("Claw Quad", Claw->GetSensorCollection().GetQuadraturePosition());
//	frc::SmartDashboard::PutNumber("Claw Forward Limit", ClawSensor->GetGeneralInput(ClawSensor->LIMF));
//
//	frc::SmartDashboard::PutNumber("DBLeft", PDP->GetCurrent(kPDP::DBLeft));
//	frc::SmartDashboard::PutNumber("DBLeft2", PDP->GetCurrent(kPDP::DBLeft2));
//	frc::SmartDashboard::PutNumber("DBRight", PDP->GetCurrent(kPDP::DBRight));
//	frc::SmartDashboard::PutNumber("DBRight2", PDP->GetCurrent(kPDP::DBRight2));
	frc::SmartDashboard::PutNumber("Elevator1", PDP->GetCurrent(kPDP::Elevator1));
	frc::SmartDashboard::PutNumber("Elevator2", PDP->GetCurrent(kPDP::Elevator2));
	frc::SmartDashboard::PutNumber("Elevator3", PDP->GetCurrent(kPDP::Elevator3));
	frc::SmartDashboard::PutNumber("Claw current", PDP->GetCurrent(kPDP::Claw));
	frc::SmartDashboard::PutNumber("ClawLeft current", PDP->GetCurrent(kPDP::ClawLeft));
	frc::SmartDashboard::PutNumber("ClawRight current", PDP->GetCurrent(kPDP::ClawRight));

//	just for testing TODO delete after testing and Dashboard code above
//	Claw->Set(ControlMode::PercentOutput, Joystick2->GetRawAxis(Attack::Down) * -1);
//	if(Joystick2->GetRawButton(1))Elevator1->Set(ControlMode::PercentOutput, Joystick2->GetRawAxis(Attack::ReverseThrottle) * -1);
//	else Elevator1->Set(ControlMode::PercentOutput, 0);
	if(Joystick2->GetRawButton(PS4::R3)) Elevator1->Set(ControlMode::PercentOutput, Joystick2->GetRawAxis(PS4::PSLeftStickDown));
	else Elevator1->Set(ControlMode::PercentOutput, Joystick2->GetRawAxis(PS4::PSLeftStickDown) / 2);
	ClawSpeed= Joystick2->GetRawAxis(PS4::PSRightStickDown);
//	if((ClawSpeed < -.04)|(ClawSpeed > 0.04)){
		Claw->Set(ControlMode::PercentOutput, ClawSpeed/2);
//		ClawHold=Claw->GetSelectedSensorPosition(0);
//	}else{
//		Claw->Set(ControlMode::Position, ClawHold);
//	}
}

void Robot::DisabledInit() {
//	RGB(25, 25, 25, ClawSensor);
	mytimer->Stop();
	mytimer->Reset();
//	Elevator1->SetSelectedSensorPosition(kElevatorEncoderKnownLow, /*REMOTE*/0, /*TimeOut*/0);
}

void Robot::DisabledPeriodic() {

}

void Robot::TestInit() {
}

void Robot::TestPeriodic() {
}

void Robot::RobotPeriodic() {
	frc::SmartDashboard::PutNumber("Elevator", Elevator1->GetSelectedSensorPosition(0));
}

START_ROBOT_CLASS(Robot)
