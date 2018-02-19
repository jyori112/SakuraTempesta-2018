package org.usfirst.frc.team6909.robot;

/* ToDo
 *・最低動作出力の確認 → 出力の下限がそこになる新しい関数を用意?
 *
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
	public PIDController driveRightMotor_pidController1;
	public PIDController driveLeftMotor_pidController1;
	public PIDController driveRightMotor_pidController2;
	public PIDController driveLeftMotor_pidController2;
	//public PIDController driveSpeed_pidController;
	//public DriveSpeedPIDOutput driveSpeed_pidWrite;
	public PIDController driveRotation_pidController;
	public DriveRotationPIDOutput driveRotation_pidWrite;
	static final double DriveSpeedTolerance = 1.0; //未使用
	static final double DriveRotationTolerane = 1.0; //未使用
	static final double kDriveSpeed_P = 0.04; //調整中
	static final double kDriveSpeed_I = 0.00;
	static final double kDriveSpeed_D = 0.00;
	static final double kDriveSpeed_P2 = 0.01; //調整中
	static final double kDriveSpeed_I2 = 0.00;
	static final double kDriveSpeed_D2 = 0.00;
	static final double kDriveRotation_P = 0.03; //調整中
	static final double kDriveRotation_I = 0.00;
	static final double kDriveRotation_D = 0.02;
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

		driveRightMotor_pidController1 = new PIDController(kDriveSpeed_P, kDriveSpeed_I, kDriveSpeed_D,
				driveRightEncoder, new DriveRightMotorPIDOutput(this.rightMotors));
		driveLeftMotor_pidController1 = new PIDController(kDriveSpeed_P, kDriveSpeed_I, kDriveSpeed_D, driveLeftEncoder,
				new DriveLeftMotorPIDOutput(this.leftMotors));
		driveRightMotor_pidController1.setEnabled(false);
		driveLeftMotor_pidController1.setEnabled(false);
		driveRightMotor_pidController1.setAbsoluteTolerance(100);
		driveLeftMotor_pidController1.setAbsoluteTolerance(100);
		driveRightMotor_pidController1.setOutputRange(-0.6, 0.6);
		driveLeftMotor_pidController1.setOutputRange(-0.6, 0.6);
		driveRightMotor_pidController2 = new PIDController(kDriveSpeed_P2, kDriveSpeed_I2, kDriveSpeed_D2,
				driveRightEncoder, new DriveRightMotorPIDOutput(this.rightMotors));
		driveLeftMotor_pidController2 = new PIDController(kDriveSpeed_P2, kDriveSpeed_I2, kDriveSpeed_D2,
				driveLeftEncoder,
				new DriveLeftMotorPIDOutput(this.leftMotors));
		driveRightMotor_pidController2.setEnabled(false);
		driveLeftMotor_pidController2.setEnabled(false);
		driveRightMotor_pidController2.setAbsoluteTolerance(50);
		driveLeftMotor_pidController2.setAbsoluteTolerance(50);
		driveRightMotor_pidController2.setOutputRange(-0.4, 0.4);
		driveLeftMotor_pidController2.setOutputRange(-0.4, 0.4);
		/*
		driveSpeed_pidWrite = new DriveSpeedPIDOutput(my_arcade_drive);
		driveSpeed_pidController = new PIDController(kDriveSpeed_P, kDriveSpeed_I, kDriveSpeed_D, driveRightEncoder, driveSpeed_pidWrite);
		driveSpeed_pidController.setEnabled(false);
		driveSpeed_pidController.setAbsoluteTolerance(50);
		driveSpeed_pidController.setInputRange(0, 9000);
		*/

		gyro = new ADXRS450_Gyro();
		gyro.reset();

		driveRotation_pidWrite = new DriveRotationPIDOutput(my_arcade_drive);
		driveRotation_pidController = new PIDController(kDriveRotation_P, kDriveRotation_I, kDriveRotation_D, gyro,
				driveRotation_pidWrite);
		driveRotation_pidController.setEnabled(false);
		driveRotation_pidController.setAbsoluteTolerance(5.0); //ちょい厳しいかも *要確認*

		//driveSpeed_pidWrite.setReferencePIDController(driveRotation_pidController); //RotationのPIDもenableの時はそのoutputで回転もする
		//driveRotation_pidWrite.setReferencePIDController(driveSpeed_pidController); //SpeedのPIDもenabledの時はそのoutputで前後進もする **こちらは使わないほうがいいかも
	}

	void runRotationPID(double setpoint) {
		driveRotation_pidController.setSetpoint(setpoint);
		driveRotation_pidController.enable();
	}

	void stopRotationPID() {
		driveRotation_pidController.disable();
		driveRotation_pidController.free();
		my_arcade_drive.arcadeDrive(0.0, 0.0);
	}

	void runSpeedPID1(double setpoint) {
		driveRightMotor_pidController1.setSetpoint(setpoint);
		driveLeftMotor_pidController1.setSetpoint(setpoint);

		driveRightMotor_pidController1.enable();
		driveLeftMotor_pidController1.enable();

		/*
		driveSpeed_pidController.setSetpoint(setpoint);
		driveSpeed_pidController.enable();
		*/
	}

	void runSpeedPID2(double setpoint) {
		driveRightMotor_pidController2.setSetpoint(setpoint);
		driveLeftMotor_pidController2.setSetpoint(setpoint);

		driveRightMotor_pidController2.enable();
		driveLeftMotor_pidController2.enable();

		/*
		driveSpeed_pidController.setSetpoint(setpoint);
		driveSpeed_pidController.enable();
		*/
	}

	void stopSpeedPID1() {
		driveRightMotor_pidController1.disable();
		driveLeftMotor_pidController1.disable();
		driveRightMotor_pidController1.free();
		driveLeftMotor_pidController1.free();
		leftMotors.set(0.0);
		rightMotors.set(0.0);
	}

	void stopSpeedPID2() {
		driveRightMotor_pidController2.disable();
		driveLeftMotor_pidController2.disable();
		driveRightMotor_pidController2.free();
		driveLeftMotor_pidController2.free();
		leftMotors.set(0.0);
		rightMotors.set(0.0);
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

class DriveRightMotorPIDOutput implements PIDOutput {
	SpeedControllerGroup rightMotors;

	DriveRightMotorPIDOutput(SpeedControllerGroup rightMotors) {
		this.rightMotors = rightMotors;
	}

	@Override
	public void pidWrite(double output) {
		rightMotors.set(-0.94 * output);
	}

}

class DriveLeftMotorPIDOutput implements PIDOutput {
	SpeedControllerGroup leftMotors;

	DriveLeftMotorPIDOutput(SpeedControllerGroup leftMotors) {
		this.leftMotors = leftMotors;
	}

	@Override
	public void pidWrite(double output) {
		leftMotors.set(0.98 * output);
	}

}

class DriveRotationPIDOutput implements PIDOutput {
	//PIDController driveSpeed_pidController;
	//isReferencePIDSet = false;
	DifferentialDrive my_arcade_drive;

	DriveRotationPIDOutput(DifferentialDrive my_arcade_drive) {
		this.my_arcade_drive = my_arcade_drive;
	}

	/*
	public void setReferencePIDController(PIDController driveSpeed_pidController) {
		this.driveSpeed_pidController = driveSpeed_pidController;
		isReferencePIDSet = true;
	}
	*/

	@Override
	public void pidWrite(double output) {
		/*
		if (isReferancePIDSet) {
			if (driveSpeed_pidController.isEnabled()) {
				my_arcade_drive.arcadeDrive(driveSpeed_pidController.get(), output);
			} else {
				//停止してその場で回転する
				my_arcade_drive.arcadeDrive(0.0, output);
			}
		}
		*/
		/*
		else {
		*/
		my_arcade_drive.arcadeDrive(0.0, output);
		/*
		}
		*/
	}

}

/*
class DriveSpeedPIDOutput implements PIDOutput {
	PIDController driveRotation_pidController;
	DifferentialDrive my_arcade_drive;
	boolean isReferencePIDSet = false;

	DriveSpeedPIDOutput(DifferentialDrive my_arcade_drive) {
		this.my_arcade_drive = my_arcade_drive;
	}

	public void setReferencePIDController(PIDController driveRotation_pidController) {
		this.driveRotation_pidController = driveRotation_pidController;
		isReferencePIDSet = true;
	}
/

	@Override
	public void pidWrite(double output) {
		if (isReferencePIDSet) {
			if (driveRotation_pidController.isEnabled()) {
				my_arcade_drive.arcadeDrive(output, driveRotation_pidController.get());
			}else {
				my_arcade_drive.arcadeDrive(output, 0.0);
			}
		}else {
			my_arcade_drive.arcadeDrive(output, 0.0);
		}

	}
}
*/