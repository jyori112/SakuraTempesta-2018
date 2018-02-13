/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {

	int kXboxDrivePort = 0;
	int kXboxOpePort = 1;

	XboxController xbox_ope;
	XboxController xbox_drive;
	Arm arm;
	Drive drive;
	Lift lift;
	AutonomousChooser autonomousChooser;
	//PowerDistributionPanel pdp;

	String gameData;
	int location;

	@Override
	public void robotInit() {

		xbox_drive = new XboxController(kXboxDrivePort);
		xbox_ope = new XboxController(kXboxOpePort);
		drive = new Drive(xbox_drive);
		arm = new Arm(xbox_ope);
		lift = new Lift(xbox_ope);

		CameraServer.getInstance().startAutomaticCapture(); //カメラ起動
		CameraServer.getInstance().getVideo();

		gameData = DriverStation.getInstance().getGameSpecificMessage();
		location = DriverStation.getInstance().getLocation();

		autonomousChooser = new AutonomousChooser(gameData, location, xbox_drive, xbox_ope, drive, lift, arm);

		//pdp = new PowerDistributionPanel();

	}

	@Override
	public void disabledInit() {

	}
	@Override
	public void disabledPeriodic() {
		autonomousChooser.chooseAutonomusMode();
		SmartDashboard.putBoolean("IsAutonomousModeChosen?", autonomousChooser.isAutonomousModeChosen);
		SmartDashboard.putNumber("SelectedAutonomousMode", autonomousChooser.autonomousChooser);
		SmartDashboard.putBoolean("IsAutonomousDone?", autonomousChooser.isAutonomousDone);
	}

	@Override
	public void autonomousInit() {

	}


	@Override
	public void autonomousPeriodic() {
		autonomousChooser.autonomousPeriodic();
		SmartDashboard.putBoolean("speedPID", drive.driveSpeed_pidController.isEnabled());
		SmartDashboard.putBoolean("rotatePID", drive.driveRotation_pidController.isEnabled());
		SmartDashboard.putBoolean("locationOK?", drive.driveSpeed_pidController.onTarget());
		SmartDashboard.putBoolean("angleOK?", drive.driveRotation_pidController.onTarget());
		SmartDashboard.putNumber("Gyro", drive.gyro.getAngle());
		SmartDashboard.putNumber("Right Encoder", drive.driveRightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", drive.driveLeftEncoder.getDistance());
	}


	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {
		drive.teleopPeriodic();
		lift.teleopPeriodic();
		arm.teleopPeriodic();

		/*
		SmartDashboard.putNumber("PDP Supply Voltage", pdp.getVoltage());
		SmartDashboard.putNumber("PDP Temperature", pdp.getTemperature());
		SmartDashboard.putNumber("PDP Total Current", pdp.getTotalCurrent());
		SmartDashboard.putNumber("PDP Total Energy", pdp.getTotalEnergy());
		SmartDashboard.putNumber("PDP Total Power", pdp.getTotalPower());
		SmartDashboard.putNumber("Right Arm Current", pdp.getCurrent(4));
		SmartDashboard.putNumber("Left Arm Current", pdp.getCurrent(1));
		SmartDashboard.putNumber("Right1 Drive Current", pdp.getCurrent(14));
		SmartDashboard.putNumber("Right2 Drive Current", pdp.getCurrent(13));
		SmartDashboard.putNumber("Left1 Drive Current", pdp.getCurrent(12));
		SmartDashboard.putNumber("Left2 Drive Current", pdp.getCurrent(0));
		SmartDashboard.putNumber("Lift Current", pdp.getCurrent(15));
		*/

		SmartDashboard.putNumber("Lift Encoder", lift.liftEncoder.getDistance());
		SmartDashboard.putNumber("Estimated arm height", lift.liftEncoder.getArmsHeight());
		SmartDashboard.putNumber("Gyro", drive.gyro.getAngle());
		SmartDashboard.putNumber("Right Encoder", drive.driveRightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", drive.driveLeftEncoder.getDistance());

	}

	@Override
	public void testPeriodic() {

	}
}
