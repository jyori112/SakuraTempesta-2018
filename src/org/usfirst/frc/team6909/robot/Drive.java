package org.usfirst.frc.team6909.robot;

/* ToDo
 *
 * 今のところなし
 *
 *
 */

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
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
	private static final double kDriveEncoderMMPerPulse = 7.7 * Math.PI / 10.71;
	public Encoder driveLeftEncoder;
 	public Encoder driveRightEncoder;
	//Gyro関連
	public ADXRS450_Gyro gyro;
	//PID
 	public PIDController driveSpeed_pidController;
	public PIDController driveRotation_pidController;
	static final double DriveSpeedTolerance = 1.0;
	static final double DriveRotationTolerane = 1.0;
	static final double kDriveSpeed_P = 0.5; //調整中
	static final double kDriveSpeed_I = 0.00;
	static final double kDriveSpeed_D = 0.00;
	static final double kDriveRotation_P = 0.5;
	static final double kDriveRotation_I = 0.00;
	static final double kDriveRotation_D = 0.00;
	//モーター
	private Spark leftFront;
	private Spark leftRear;
	private Spark rightFront;
	private Spark rightRear;
	public SpeedControllerGroup leftMotors;
	public SpeedControllerGroup rightMotors;
	//ドライブ本体
	public DifferentialDrive my_arcade_drive;
	//不感帯
	static final double kNoReact = 0.1;

	//操作するコントローラ
	public XboxController xbox_drive;
	//左Y軸の値を格納
	private double xfb;
	//右X軸の値を格納
	private double xlr;

	Drive(XboxController xbox_drive) {
		this.xbox_drive = xbox_drive;
		leftFront = new Spark(kLeftFrontPort);
		leftRear = new Spark(kLeftRearPort);
		rightFront = new Spark(kRightFrontPort);
		rightRear = new Spark(kRightRearPort);
		leftMotors = new SpeedControllerGroup(leftFront, leftRear);
		rightMotors = new SpeedControllerGroup(rightFront, rightRear);
		my_arcade_drive = new DifferentialDrive(leftMotors, rightMotors);

		driveRightEncoder = new Encoder(kDriveLeftEncoderChannelAPort, kDriveLeftEncoderChannelBPort);
		driveRightEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);
		driveLeftEncoder = new Encoder(kDriveRightEncoderChannelAPort, kDriveRightEncoderChannelBPort, true);
		driveLeftEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);
		driveRightEncoder.reset();
		driveLeftEncoder.reset();

		driveSpeed_pidController = new PIDController(kDriveSpeed_P, kDriveSpeed_I, kDriveSpeed_D, driveRightEncoder, new DriveSpeedPIDOutput(this.my_arcade_drive));
		driveSpeed_pidController.setEnabled(false);
		driveSpeed_pidController.setPercentTolerance(DriveSpeedTolerance);



		gyro = new  ADXRS450_Gyro();
		gyro.reset(); //Init時にreset **変更の可能性あり**

		driveRotation_pidController = new PIDController(kDriveRotation_P, kDriveRotation_I, kDriveRotation_D, gyro, new DriveRotationPIDOutput(this.my_arcade_drive));
		driveRotation_pidController.setEnabled(false);
		driveRotation_pidController.setPercentTolerance(DriveRotationTolerane);

	}

	void runRotationPID(double setpoint) {
		driveRotation_pidController.setInputRange(0, setpoint);
		driveRotation_pidController.setSetpoint(setpoint);
		driveRotation_pidController.enable();
	}

	void stopRotationPID() {
		driveRotation_pidController.disable();
	}

	void runSpeedPID(double setpoint) {
		driveSpeed_pidController.setInputRange(0, setpoint);
		driveSpeed_pidController.setSetpoint(setpoint);
		driveSpeed_pidController.enable();
	}

	void stopSpeedPID() {
		driveSpeed_pidController.disable();
	}

	void handControl() {
		xfb = xbox_drive.getY(Hand.kLeft);
		xlr = xbox_drive.getX(Hand.kRight);

		my_arcade_drive.arcadeDrive(Util.outputCalc(kNoReact, -xfb), Util.outputCalc(kNoReact, xlr));
		/* 入力に等しい出力が欲しいならこちら。
		my_arcade_drive.arcadeDrive(xfb, xlr);
		*/
	}

	void teleopPeriodic() {
		handControl();
	}
}

class DriveRotationPIDOutput implements PIDOutput {

	DifferentialDrive my_arcade_drive;

	DriveRotationPIDOutput(DifferentialDrive my_arcade_drive) {
		this.my_arcade_drive = my_arcade_drive;
	}

	@Override
	public void pidWrite(double output) {
		//停止してその場で回転する
		my_arcade_drive.arcadeDrive(0.0, output);
	}

}

class DriveSpeedPIDOutput implements PIDOutput {

	DifferentialDrive my_arcade_drive;

	DriveSpeedPIDOutput(DifferentialDrive my_arcade_drive) {
		this.my_arcade_drive = my_arcade_drive;
	}

	@Override
	public void pidWrite(double output) {
		//直進
		my_arcade_drive.arcadeDrive(output, 0.0);
	}

}
