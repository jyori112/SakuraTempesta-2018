package org.usfirst.frc.team1.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive {
	//Port
	private static final int kLeftFrontPort = 0;
	private static final int kLeftRearPort = 1;
	private static final int kRightFrontPort = 2;
	private static final int kRightRearPort = 3;
	//エンコーダー関連
	private static final int kDriveLeftEncoderChannelAPort = 6;
	private static final int kDriveLeftEncoderChannelBPort = 7;
	private static final int kDriveRightEncoderChannelAPort = 8;
	private static final int kDriveRightEncoderChannelBPort = 9;
	private static final double kDriveEncoderMMPerPulse = 77 * Math.PI;
	private Encoder DriveLeftEncoder;
	private Encoder DriveRightEncoder;
	//モーター
	private Spark leftFront;
	private Spark leftRear;
	private Spark rightFront;
	private Spark rightRear;
	private SpeedControllerGroup leftMotors;
	private SpeedControllerGroup rightMotors;
	//ドライブ本体
	private DifferentialDrive my_arcade_drive;
	//不感帯
	private static final double kNoReact = 0.1;
	//操作するコントローラ
	private XboxController xbox_drive;

	//Init
	Drive(XboxController xbox_drive) {

		this.xbox_drive = xbox_drive;
		leftFront = new Spark(kLeftFrontPort);
		leftRear = new Spark(kLeftRearPort);
		rightFront = new Spark(kRightFrontPort);
		rightRear = new Spark(kRightRearPort);
		leftMotors = new SpeedControllerGroup(leftFront, leftRear);
		rightMotors = new SpeedControllerGroup(rightFront, rightRear);
		my_arcade_drive = new DifferentialDrive(leftMotors, rightMotors);
		DriveRightEncoder = new Encoder(kDriveLeftEncoderChannelAPort, kDriveLeftEncoderChannelBPort);
		DriveRightEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);
		DriveLeftEncoder = new Encoder(kDriveRightEncoderChannelAPort, kDriveRightEncoderChannelBPort);
		DriveLeftEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);

	}

	void teleop_control() {

		if ((Math.abs(xbox_drive.getY(Hand.kLeft)) < kNoReact)
				&& (Math.abs(xbox_drive.getX(Hand.kRight)) < kNoReact)) {
			my_arcade_drive.arcadeDrive(0.0, 0.0); // Stay
		} else if ((Math.abs(xbox_drive.getY(Hand.kLeft)) > kNoReact)
				&& (Math.abs(xbox_drive.getX(Hand.kRight)) < kNoReact)) {
			my_arcade_drive.arcadeDrive(-xbox_drive.getY(Hand.kLeft), 0.0); // Drive forward/backward
		} else if ((Math.abs(xbox_drive.getY(Hand.kLeft)) < kNoReact)
				&& (Math.abs(xbox_drive.getX(Hand.kRight)) > kNoReact)) {
			my_arcade_drive.arcadeDrive(0.0, xbox_drive.getX(Hand.kRight)); // Turn right/left
		} else {
			my_arcade_drive.arcadeDrive(-xbox_drive.getY(Hand.kLeft), xbox_drive.getX(Hand.kRight)); // Free drive
		}

	}
}