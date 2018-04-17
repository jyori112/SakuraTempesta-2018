package org.usfirst.frc.team69.robot;

/*
 * ロボットはXboxController一つで十分に操作できる。
 */

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;


public class Robot extends IterativeRobot {

	int kXboxDrivePort = 0;
	int kXboxOpePort = 1;

	XboxController xbox_ope; //現在未使用
	XboxController xbox_drive; //このコントローラのみで操作するようにしている

	Arm arm;
	Lift lift;
	Drive_BASE drive_base;


	String gameData;
	int location;

	Drive_PID drive_pid;
	AutonomousChooser_PID autonomousChooser_pid;
	AutonomousChooser_TIME autonomousChooser_time;

	final String DriveAutonomousMode = "SinglEncoder"; // "DualEncoder" or "SinglEncoder" or "TIME"

	@Override
	public void robotInit() {

		xbox_drive = new XboxController(kXboxDrivePort);
		arm = new Arm(xbox_drive);
		lift = new Lift(xbox_drive);
		drive_base = new Drive_BASE(xbox_drive, lift.liftEncoder);

		if (DriveAutonomousMode == "TIME") {
			autonomousChooser_time = new AutonomousChooser_TIME(xbox_drive, drive_base, lift, arm);
		}else {
			drive_pid = new Drive_PID(DriveAutonomousMode, drive_base);
			autonomousChooser_pid = new AutonomousChooser_PID(xbox_drive, drive_pid, lift, arm);
		}

		CameraServer.getInstance().startAutomaticCapture(); //カメラ起動
		CameraServer.getInstance().getVideo();

	}

	@Override
	public void disabledInit() {

	}
	@Override
	public void disabledPeriodic() {

	}

	@Override
	public void autonomousInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (DriveAutonomousMode == "TIME") {
			autonomousChooser_time.init(gameData);
		}else {
			drive_pid = new Drive_PID(DriveAutonomousMode, drive_base);
			autonomousChooser_pid.init(gameData);
		}
	}


	@Override
	public void autonomousPeriodic() {
		if (DriveAutonomousMode == "TIME") {
			autonomousChooser_time.periodic();
		}else {
			drive_pid = new Drive_PID(DriveAutonomousMode, drive_base);
			autonomousChooser_pid.periodic();
		}
	}


	@Override
	public void teleopInit() {
		drive_base.teleopInit();
		lift.teleopInit();
		arm.teleopInit();

	}

	@Override
	public void teleopPeriodic() {
		drive_base.teleopPeriodic();
		lift.teleopPeriodic();
		arm.teleopPeriodic();
	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {

	}
}

