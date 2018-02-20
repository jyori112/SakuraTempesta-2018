package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends IterativeRobot {

	int kXboxDrivePort = 0;
	int kXboxOpePort = 1;

	XboxController xbox_ope; //現在未使用
	XboxController xbox_drive; //このコントローラのみで操作するようにしている

	Arm arm;
	Lift lift;
	Drive drive;

	AutonomousChooser autonomousChooser;

	String gameData;
	int location;

	@Override
	public void robotInit() {

		xbox_drive = new XboxController(kXboxDrivePort);
		arm = new Arm(xbox_drive);
		lift = new Lift(xbox_drive);
		drive = new Drive(xbox_drive, lift.liftEncoder);
		//drive2 = new Drive_withRightLeftPID(xbox_drive, lift.liftEncoder);

		CameraServer.getInstance().startAutomaticCapture(); //カメラ起動
		CameraServer.getInstance().getVideo();

		gameData = DriverStation.getInstance().getGameSpecificMessage();
		location = DriverStation.getInstance().getLocation();

		autonomousChooser = new AutonomousChooser(gameData, location, xbox_drive, xbox_drive, drive, lift, arm);

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
		autonomousChooser.autonomousInit();
	}


	@Override
	public void autonomousPeriodic() {
		autonomousChooser.autonomousPeriodic();
		SmartDashboard.putNumber("right encoder", drive.driveRightEncoder.getDistance());
		SmartDashboard.putNumber("left encoder", drive.driveLeftEncoder.getDistance());
		SmartDashboard.putNumber("gyro", drive.gyro.getAngle());
		SmartDashboard.putString("current DrivePID Mode", drive.getDrivePIDMode());
		SmartDashboard.putNumber("current phase", autonomousChooser.phase);
	}


	@Override
	public void teleopInit() {
		drive.teleopInit();
		lift.teleopInit();
		arm.teleopInit();
	}

	@Override
	public void teleopPeriodic() {
		drive.teleopPeriodic();
		lift.teleopPeriodic();
		arm.teleopPeriodic();

	}

	@Override
	public void testInit() {
		drive.setDrivePIDMode("stop");
	}

	@Override
	public void testPeriodic() {
		drive.runStraightPID(10000);
	}
}

