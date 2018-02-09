/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6909.robot;

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

	String gameData;
	int location;

	@Override
	public void robotInit() {

		xbox_drive = new XboxController(kXboxDrivePort);
		xbox_ope = new XboxController(kXboxOpePort);
		drive = new Drive(xbox_drive);
		arm = new Arm(xbox_ope);
		lift = new Lift(xbox_ope);

		gameData = DriverStation.getInstance().getGameSpecificMessage();
		location = DriverStation.getInstance().getLocation();

		autonomousChooser = new AutonomousChooser(gameData, location, drive, lift, arm);

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

		SmartDashboard.putNumber("Gyro", drive.gyro.getAngle());
		SmartDashboard.putNumber("Right Encoder", drive.driveRightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", drive.driveLeftEncoder.getDistance());
	}

	@Override
	public void testPeriodic() {

	}
}
