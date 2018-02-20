package org.usfirst.frc.team6909.robot;

/* ToDo
 *・最低動作出力の確認 → 出力の下限がそこになる新しい関数を用意?
 *stopMotorの試行
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

	public PIDController straightL;
	public double straightL_output;

	public PIDController straightR;
	public double straightR_output;
	public PIDController rotateL;
	public double rotateL_output;
	public PIDController rotateR;
	public double rotateR_output;
	private String drivePIDMode = "stop";

	static final double kStraight_P = 0.03; //調整中
	static final double kStraight_I = 0.00;
	static final double kStraight_D = 0.03; //0.03
	static final double kRotate_P = 0.03; //調整中
	static final double kRotate_I = 0.002;
	static final double kRotate_D = 0.04;
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
	//参考にするリフトのエンコーダー
	EncoderWithNewFuncs liftEncoder;
	//Modyfier
	public final double rightModifier = 1.0;
	public final double leftModifier = 0.94; //0.94

	Drive(XboxController xbox_drive, EncoderWithNewFuncs liftEncoder) {
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

		gyro = new ADXRS450_Gyro();
		gyro.reset();

		straightR = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveRightEncoder, new StraightROutput(this));
		straightL = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveLeftEncoder, new StraightLOutput(this));
		rotateR = new PIDController(kRotate_P, kRotate_I,kRotate_D, gyro, new RotateROutput(this));
		rotateL = new PIDController(kRotate_P, kRotate_I, kRotate_D, gyro, new RotateLOutput(this));

		straightR.setAbsoluteTolerance(50);
		straightL.setAbsoluteTolerance(50);
		rotateR.setAbsoluteTolerance(3.0);
		rotateL.setAbsoluteTolerance(3.0);
		straightR.setOutputRange(-0.7, 0.7);
		straightL.setOutputRange(-0.7, 0.7);
		rotateR.setOutputRange(-0.7, 0.7);
		rotateL.setOutputRange(-0.7, 0.7);
		straightR.disable();
		straightL.disable();
		rotateR.disable();
		rotateL.disable();

	}

	public void setDrivePIDMode(String drivePIDMode) {
		this.drivePIDMode = drivePIDMode;
	}

	public String getDrivePIDMode() {
		return this.drivePIDMode;
	}

	public double getStraightPIDSetpoint() {
		return straightR.getSetpoint(); //RとLは常に同じsetpoint
	}

	public double getRotatePIDSetpoint() {
		return rotateR.getSetpoint(); //RとLは常に同じsetpoint
	}

	public boolean getStraightPIDOnTarget() {
		return straightR.onTarget() && straightL.onTarget();
	}

	public boolean getRotatePIDOnTarget() {
		return rotateL.onTarget() && rotateR.onTarget();
	}

	void excuteOutput() {
		if(drivePIDMode == "stop") {
			leftMotors.set(0.0);
			rightMotors.set(0.0);
		}else if(drivePIDMode == "straight") {
			leftMotors.set(straightL_output);
			rightMotors.set(straightR_output);
		}else if(drivePIDMode == "rotate") {
			leftMotors.set(rotateL_output);
			rightMotors.set(rotateR_output);
		}
	}

	void runRotatePID(double setpoint) {
		if (drivePIDMode != "rotate") {
			drivePIDMode = "rotate";
		}else {
			rotateR.setSetpoint(setpoint);
			rotateL.setSetpoint(setpoint);
			rotateR.enable();
			rotateL.enable();

			excuteOutput();
		}
	}

	void runStraightPID(double setpoint) {
		double goal = setpoint;
		double add = setpoint / 1000;
		double currentSetpoint = add;
		
		if (drivePIDMode != "straight") {
			drivePIDMode = "straight";
			driveRightEncoder.reset(); //encoderをリセット
			driveLeftEncoder.reset();
		}else {
			if(liftEncoder.getArmsHeight() > 200) {
				straightR.setOutputRange(-0.5, 0.5);
				straightL.setOutputRange(-0.5, 0.5);
				straightR.setSetpoint(currentSetpoint);
				straightL.setSetpoint(currentSetpoint);
				straightR.enable();
				straightL.enable();

				excuteOutput();
			}else {
				straightR.setOutputRange(-0.1, 0.1);
				straightR.setOutputRange(-0.1, 0.1);
				straightR.setSetpoint(currentSetpoint);
				straightL.setSetpoint(currentSetpoint);
				straightR.enable();
				straightL.enable();

				currentSetpoint += add;

				excuteOutput();
			}
		}
	}

	void stopDrive() {
		if (drivePIDMode != "stop") {
			drivePIDMode = "stop";
		}else {
			excuteOutput();
		}
	}

	void handControl() {
		xfb = xbox_drive.getY(Hand.kLeft);
		xlr = xbox_drive.getX(Hand.kRight);
		//yukkuri = xbox_drive.getTriggerAxis(Hand.kRight);

		if (liftEncoder.getArmsHeight() >= 200) {
			if (xfb >= 0.6) {
				my_arcade_drive.arcadeDrive(-0.6, Util.outputCalc(kNoReact, xlr));
			} else if (xfb <= -0.6) {
				my_arcade_drive.arcadeDrive(0.6, Util.outputCalc(kNoReact, xlr));
			} else {
				my_arcade_drive.arcadeDrive(Util.outputCalc(kNoReact, -xfb), Util.outputCalc(kNoReact, xlr));
			}
		} else {
			my_arcade_drive.arcadeDrive(Util.outputCalc(kNoReact, -xfb) * 0.7, Util.outputCalc(kNoReact, xlr) * 0.7);
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

class StraightROutput implements PIDOutput {
	Drive drive;

	StraightROutput(Drive drive) {
		this.drive = drive;
	}

	@Override
	public void pidWrite(double output) {
		drive.straightR_output = - drive.rightModifier * output;
	}

}

class StraightLOutput implements PIDOutput {
	Drive drive;

	StraightLOutput(Drive drive) {
		this.drive = drive;
	}

	@Override
	public void pidWrite(double output) {
		drive.straightL_output = drive.leftModifier * output;
	}

}

class RotateROutput implements PIDOutput {
	Drive drive;

	RotateROutput(Drive drive) {
		this.drive = drive;
	}

	@Override
	public void pidWrite(double output) {
		drive.rotateR_output = drive.rightModifier * output; //反転させる マイナスのマイナス
	}

}

class RotateLOutput implements PIDOutput {
	Drive drive;

	RotateLOutput(Drive drive) {
		this.drive = drive;
	}

	@Override
	public void pidWrite(double output) {
		drive.rotateL_output = drive.leftModifier * output;
	}

}

