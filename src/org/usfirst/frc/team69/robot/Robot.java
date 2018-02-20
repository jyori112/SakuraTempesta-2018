/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team69.robot;

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

	Drive drive;
	//Drive_withRightLeftPID drive2;

	Lift lift;

	AutonomousChooser autonomousChooser;
	//AutonomousChooser_withRightLeftPID autonomousChooser2;

	//PowerDistributionPanel pdp;

	String gameData;
	int location;

	@Override
	public void robotInit() {

		xbox_drive = new XboxController(kXboxDrivePort);
		//xbox_ope = new XboxController(kXboxOpePort);
		arm = new Arm(xbox_drive);
		lift = new Lift(xbox_drive);
		drive = new Drive(xbox_drive, lift.liftEncoder);
		//drive2 = new Drive_withRightLeftPID(xbox_drive, lift.liftEncoder);

		CameraServer.getInstance().startAutomaticCapture(); //カメラ起動
		CameraServer.getInstance().getVideo();

		gameData = DriverStation.getInstance().getGameSpecificMessage();
		location = DriverStation.getInstance().getLocation();

		autonomousChooser = new AutonomousChooser(gameData, location, xbox_drive, xbox_drive, drive, lift, arm);
		//autonomousChooser2 = new AutonomousChooser_withRightLeftPID(gameData, location, xbox_drive, xbox_drive, drive2, lift, arm);
		//pdp = new PowerDistributionPanel();

	}

	@Override
	public void disabledInit() {

	}
	@Override
	public void disabledPeriodic() {
		/*
		autonomousChooser2.chooseAutonomusMode();
		SmartDashboard.putBoolean("IsAutonomousModeChosen?", autonomousChooser2.isAutonomousModeChosen);
		SmartDashboard.putNumber("SelectedAutonomousMode", autonomousChooser2.autonomousChooser);
		SmartDashboard.putBoolean("IsAutonomousDone?", autonomousChooser2.isAutonomousDone);
		*/

		autonomousChooser.chooseAutonomusMode();
		SmartDashboard.putBoolean("IsAutonomousModeChosen?", autonomousChooser.isAutonomousModeChosen);
		SmartDashboard.putNumber("SelectedAutonomousMode", autonomousChooser.autonomousChooser);
		SmartDashboard.putBoolean("IsAutonomousDone?", autonomousChooser.isAutonomousDone);

	}

	@Override
	public void autonomousInit() {
		//autonomousChooser2.autonomousInit();
		autonomousChooser.autonomousInit();
	}


	@Override
	public void autonomousPeriodic() {
		//autonomousChooser2.autonomousPeriodic();
		//SmartDashboard.putNumber("Current phase", autonomousChooser2.phase);
		//SmartDashboard.putNumber("Lift Encoder", lift.liftEncoder.getDistance());
		//SmartDashboard.putNumber("Estimated arm height", lift.liftEncoder.getArmsHeight());

		/*
		SmartDashboard.putNumber("pid instance counter", drive2.pid_cnt);
		SmartDashboard.putBoolean("PIDExist", drive2.pidExist);
		if (drive2.pidExist == true) {
			SmartDashboard.putBoolean("RightPID enabled", drive2.rightPID.isEnabled());
			SmartDashboard.putNumber("RightPID setpoint", drive2.rightPID.getSetpoint());
			SmartDashboard.putNumber("RightPID Current output", drive2.rightPID.get());
			SmartDashboard.putBoolean("LeftPID enabled", drive2.leftPID.isEnabled());
			SmartDashboard.putNumber("LeftPID setpoint", drive2.leftPID.getSetpoint());
			SmartDashboard.putNumber("LeftPID Current output", drive2.leftPID.get());

			SmartDashboard.putBoolean("pidOnTarget?", drive2.rightPID.onTarget() && drive2.leftPID.onTarget());
		}
		//SmartDashboard.putNumber("Gyro", drive2.gyro.getAngle());
		SmartDashboard.putNumber("Right Encoder", drive2.driveRightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", drive2.driveLeftEncoder.getDistance());
		*/
		autonomousChooser.autonomousPeriodic();
		SmartDashboard.putNumber("straightPID output", drive.straightPID.get());
		SmartDashboard.putNumber("rotatePID output", drive.rotatePID.get());

		SmartDashboard.putNumber("Current phase", autonomousChooser.phase);

		SmartDashboard.putNumber("Lift Encoder", lift.liftEncoder.getDistance());
		SmartDashboard.putNumber("Estimated arm height", lift.liftEncoder.getArmsHeight());
		SmartDashboard.putNumber("lift setpoint", lift.lift_pidController.getSetpoint());

		SmartDashboard.putBoolean("straightPID", drive.straightPID.isEnabled());
		SmartDashboard.putNumber("Right Encoder", drive.driveRightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", drive.driveLeftEncoder.getDistance());
		SmartDashboard.putBoolean("locationOK?", drive.straightPID.onTarget());
		SmartDashboard.putNumber("straight setpoint", drive.straightPID.getSetpoint());

		SmartDashboard.putBoolean("rotatePID", drive.rotatePID.isEnabled());
		SmartDashboard.putBoolean("angleOK?", drive.rotatePID.onTarget());
		SmartDashboard.putNumber("Gyro", drive.gyro.getAngle());
		SmartDashboard.putNumber("rotate setpoint", drive.rotatePID.getSetpoint());

	}


	@Override
	public void teleopInit() {
		drive.teleopInit();
		//drive2.teleopInit();
		lift.teleopInit();
		arm.teleopInit();
	}

	@Override
	public void teleopPeriodic() {
		drive.teleopPeriodic();
		//drive2.teleopPeriodic();
		lift.teleopPeriodic();
		arm.teleopPeriodic();

		SmartDashboard.putNumber("POV", xbox_drive.getPOV());


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
		/*
		SmartDashboard.putNumber("Current Lift PID Setpoint", lift.lift_pidController.getSetpoint());
		SmartDashboard.putBoolean("Lift PID onTarget?", lift.lift_pidController.onTarget());
		SmartDashboard.putNumber("Lift Motor Output", lift.lift.get()); //kOutputResisitingGravity測定用
		SmartDashboard.putNumber("Lift Encoder", lift.liftEncoder.getDistance());
		SmartDashboard.putNumber("Estimated arm height", lift.liftEncoder.getArmsHeight());
		SmartDashboard.putNumber("Gyro", drive2.gyro.getAngle());
		SmartDashboard.putNumber("Right Encoder", drive2.driveRightEncoder.getDistance());
		SmartDashboard.putNumber("Left Encoder", drive2.driveLeftEncoder.getDistance());
		*/


		SmartDashboard.putNumber("Current Lift PID Setpoint", lift.lift_pidController.getSetpoint());
		SmartDashboard.putBoolean("Lift PID onTarget?", lift.lift_pidController.onTarget());
		SmartDashboard.putNumber("Lift Motor Output", lift.lift.get()); //kOutputResisitingGravity測定用
		SmartDashboard.putNumber("Lift Encoder", lift.liftEncoder.getDistance());
		SmartDashboard.putNumber("Estimated arm height", lift.liftEncoder.getArmsHeight());
		SmartDashboard.putBoolean("lift at bottom", lift.liftBottomSwitch.get());
		SmartDashboard.putBoolean("lift at highest", lift.liftHighestSwitch.get());
		//SmartDashboard.putNumber("Gyro", drive.gyro.getAngle());
		//SmartDashboard.putNumber("Right Encoder", drive.driveRightEncoder.getDistance());
		//SmartDashboard.putNumber("Left Encoder", drive.driveLeftEncoder.getDistance());

	}

	@Override
	public void testInit() {
		drive.straightPID.disable();
		drive.rotatePID.disable();
		lift.lift_pidController.disable();
		lift.lift_pidController.setOutputRange(Lift.kOutputResistingGravity, 0.7);
		drive.driveRightEncoder.reset();
		drive.driveLeftEncoder.reset();
		drive.gyro.reset();
		lift.liftEncoder.reset();
	}

	@Override
	public void testPeriodic() {
		int setpoint = 6000;

		if( Math.abs((drive.driveRightEncoder.getDistance() + drive.driveLeftEncoder.getDistance()) / 2 - setpoint) < 100){
			drive.my_arcade_drive.arcadeDrive(0.0, 0.0);
		}else {
			if(drive.gyro.getAngle() < -10) {
				drive.my_arcade_drive.arcadeDrive(0.0, drive.gyro.getAngle() * Drive.kRotate_P);
			}else if(drive.gyro.getAngle() > 10) {
				drive.my_arcade_drive.arcadeDrive(0.0, drive.gyro.getAngle() * Drive.kRotate_P);
			}else {
				drive.my_arcade_drive.arcadeDrive(- ((drive.driveRightEncoder.getDistance() + drive.driveLeftEncoder.getDistance()) / 2 - setpoint) * Drive.kStraight_P , 0.0);
			}
		}


		if (xbox_drive.getAButton()) {
			drive.rotatePID.setPID(0.03, 0.00, 0.03); //pだけ0.027はアリ
			drive.runRotationPID(90);
		}else if(xbox_drive.getBButton()) {
			drive.rotatePID.setPID(0.03, 0.00, 0.04);
			drive.runRotationPID(180);
		}
	}
}
