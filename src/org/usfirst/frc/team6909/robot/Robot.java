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

		CameraServer.getInstance().startAutomaticCapture(); //カメラ起動
		CameraServer.getInstance().getVideo();

		autonomousChooser = new AutonomousChooser(xbox_drive, xbox_drive, drive, lift, arm);

	}

	@Override
	public void disabledInit() {

	}
	@Override
	public void disabledPeriodic() {

		autonomousChooser.chooseAutonomusMode();
		SmartDashboard.putBoolean("locationChosen?", autonomousChooser.locationChosen);
		SmartDashboard.putNumber("location", autonomousChooser.location);
		SmartDashboard.putBoolean("patternChosen?", autonomousChooser.patternChosen);
		SmartDashboard.putNumber("pattern", autonomousChooser.autonomousChooser);
		SmartDashboard.putBoolean("IsAutonomousDone?", autonomousChooser.isAutonomousDone);

	}

	@Override
	public void autonomousInit() {
		autonomousChooser.autonomousInit();
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		autonomousChooser.gameData = gameData;
	}


	@Override
	public void autonomousPeriodic() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		autonomousChooser.gameData = gameData;

		autonomousChooser.autonomousPeriodic();
		SmartDashboard.putNumber("rotate R", drive.rotateR_output);
		SmartDashboard.putNumber("rotate L", drive.rotateR_output);
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
		SmartDashboard.putNumber("gyro", drive.gyro.getAngle());
		SmartDashboard.putNumber("R encoder", drive.driveRightEncoder.getDistance());
		SmartDashboard.putNumber("L encoder", drive.driveLeftEncoder.getDistance());
	}

	@Override
	public void testInit() {
		lift.lift_pidController.setOutputRange(Lift.kOutputResistingGravity, 0.7);
		drive.driveRightEncoder.reset();
		drive.driveLeftEncoder.reset();
		drive.gyro.reset();
		lift.liftEncoder.reset();
	}

	@Override
	public void testPeriodic() {
		SmartDashboard.putNumber("phase", autonomousChooser.phase);
		switch(autonomousChooser.phase) {
		case 0:
			autonomousChooser.Start();
			break;
		case 1:
			autonomousChooser.DriveRotate(90, 0.3);
			break;
		case 2:
			autonomousChooser.ArmShoot(1.0, 0.3);
			break;
		case 3:
		    autonomousChooser.End();
			break;
		}
	}
}

