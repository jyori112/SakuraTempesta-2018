package org.usfirst.frc.team69.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive_withRightLeftPID {
	//Port
	private static final int kLeftFrontPort = 0;
	private static final int kLeftRearPort = 1;
	private static final int kRightFrontPort = 2;
	private static final int kRightRearPort = 3;
	//エンコーダー関連
	private static final int kDriveRightEncoderChannelAPort = 6;
	private static final int kDriveRightEncoderChannelBPort = 7;
	private static final int kDriveLeftEncoderChannelAPort = 8;
	private static final int kDriveLeftEncoderChannelBPort = 9;
	private static final double kDriveEncoderMMPerPulse = 7.7 * Math.PI / 10.71;
	public Encoder driveLeftEncoder;
	public Encoder driveRightEncoder;
	//Gyro関連
	public ADXRS450_Gyro gyro;

	//PID
	PIDController rightPID;
	PIDController leftPID;
	public boolean pidExist;
	public int pid_cnt;

	static final double kStraight_P = 0.0;
	static final double kStraight_I = 0.0;
	static final double kStraight_D = 0.0;

	static final double kRotate_P = 0.0;
	static final double kRotate_I = 0.0;
	static final double kRotate_D = 0.0;

	//モーター
	public Spark leftFront;
	public Spark leftRear;
	public Spark rightFront;
	public Spark rightRear;
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
	//参考にするリフトのエンコーダー
	EncoderWithNewFuncs liftEncoder;

	Drive_withRightLeftPID(XboxController xbox_drive, EncoderWithNewFuncs liftEncoder) {
		this.xbox_drive = xbox_drive;
		this.liftEncoder = liftEncoder;
		leftFront = new Spark(kLeftFrontPort);
		leftRear = new Spark(kLeftRearPort);
		rightFront = new Spark(kRightFrontPort);
		rightRear = new Spark(kRightRearPort);
		leftMotors = new SpeedControllerGroup(leftFront, leftRear);
		rightMotors = new SpeedControllerGroup(rightFront, rightRear);
		my_arcade_drive = new DifferentialDrive(leftMotors, rightMotors);

		driveRightEncoder = new Encoder(kDriveLeftEncoderChannelAPort, kDriveLeftEncoderChannelBPort, true);
		driveRightEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);
		driveLeftEncoder = new Encoder(kDriveRightEncoderChannelAPort, kDriveRightEncoderChannelBPort, false);
		driveLeftEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);
		driveRightEncoder.reset();
		driveLeftEncoder.reset();

		gyro = new  ADXRS450_Gyro();
		gyro.reset();

		pidExist = false;

		/*
		rightPID = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveRightEncoder, new RightPIDWrite(this.rightMotors));
		leftPID = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveLeftEncoder, new LeftPIDWrite(this.leftMotors));
		rightPID.setEnabled(false);
		leftPID.setEnabled(false);
		rightPID.setAbsoluteTolerance(100);
		leftPID.setAbsoluteTolerance(100);
		rightPID.setOutputRange(-0.5, 0.5);
		leftPID.setOutputRange(-0.5, 0.5);
		 */
	}

	void runRotatePID(double setpoint) {
		rightPID = new PIDController(kRotate_P, kRotate_I, kRotate_D, gyro, new RightPIDWrite(this.rightMotors));
		leftPID = new PIDController(kRotate_P, kRotate_I, kRotate_D, gyro, new LeftPIDWrite(this.leftMotors));

		pid_cnt++;
		pidExist = true;

		rightPID.setAbsoluteTolerance(100);
		leftPID.setAbsoluteTolerance(100);
		rightPID.setOutputRange(-0.5, 0.5);
		leftPID.setOutputRange(-0.5, 0.5);

		rightPID.setSetpoint(setpoint);
		leftPID.setSetpoint(setpoint);
		rightPID.enable();
		leftPID.enable();

	}

	void stopRotatePID() {
		rightPID.free();
		leftPID.free();

		pid_cnt--;
		pidExist = false;
	}

	void runStraightPID(double setpoint) {
		rightPID = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveRightEncoder, new RightPIDWrite(this.rightMotors));
		leftPID = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveLeftEncoder, new LeftPIDWrite(this.leftMotors));

		pid_cnt++;
		pidExist = true;

		rightPID.setAbsoluteTolerance(100);
		leftPID.setAbsoluteTolerance(100);
		rightPID.setOutputRange(-0.7, 0.7);
		leftPID.setOutputRange(-0.7, 0.7);

		rightPID.setSetpoint(setpoint);
		leftPID.setSetpoint(setpoint);
		rightPID.enable();
		leftPID.enable();
	}

	void stopStraightPID() {
		rightPID.free();
		leftPID.free();

		pid_cnt--;
		pidExist = false;
	}

	void handControl() {
		xfb = xbox_drive.getY(Hand.kLeft);
		xlr = xbox_drive.getX(Hand.kRight);
		//yukkuri = xbox_drive.getTriggerAxis(Hand.kRight);

		if (liftEncoder.getArmsHeight() >= 200) {
			if (xfb >= 0.6) {
				my_arcade_drive.arcadeDrive(-0.6, Util.outputCalc(kNoReact, xlr));
			}else if(xfb <= -0.6) {
				my_arcade_drive.arcadeDrive(0.6, Util.outputCalc(kNoReact, xlr));
			}else {
				my_arcade_drive.arcadeDrive(Util.outputCalc(kNoReact, -xfb), Util.outputCalc(kNoReact, xlr));
			}
		}else{
			my_arcade_drive.arcadeDrive(Util.outputCalc(kNoReact, -xfb), Util.outputCalc(kNoReact, xlr));
			/* 入力に等しい出力が欲しいならこちら。
				my_arcade_drive.arcadeDrive(xfb, xlr);
			 */
		}
	}

	void teleopInit() {
		driveRightEncoder.reset();
		driveLeftEncoder.reset();
	}

	void teleopPeriodic() {
		handControl();
	}
}

class RightPIDWrite implements PIDOutput{

	SpeedControllerGroup rightMotors;

	RightPIDWrite(SpeedControllerGroup rightMotors){
		this.rightMotors = rightMotors;
	}

	@Override
	public void pidWrite(double output) {
		rightMotors.set(output );
	}

}

class LeftPIDWrite implements PIDOutput{

	SpeedControllerGroup leftMotors;

	LeftPIDWrite(SpeedControllerGroup leftMotors){
		this.leftMotors = leftMotors;
	}

	@Override
	public void pidWrite(double output) {
		leftMotors.set( - output);
	}

}
