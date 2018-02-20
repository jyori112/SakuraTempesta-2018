package org.usfirst.frc.team69.robot;

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
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
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
	public double pid_straight_output;
	public double pid_rotate_output;
	public PIDController straightPID;
	public RotatePIDSource rotatePIDSource;
	public PIDController rotatePID;

	static final double kStraight_P = 0.05;
	static final double kStraight_I = 0.0;
	static final double kStraight_D = 0.0;

	static final double kRotate_P_ForStraight = 0.034;
	static final double kRotate_I_ForStraight = 0.00;
	static final double kRotate_D_ForStraight = 0.042;
	static final double kRotate_P = 0.04;
	static final double kRotate_I = 0.001;
	static final double kRotate_D = 0.04;
	//static final double kDriveSpeed_I = 0.00;
	//static final double kDriveSpeed_D = 0.00;
	//static final double kDriveRotation_P = 0.05; //調整中
	//static final double kDriveRotation_I = 0.00;
	//static final double kDriveRotation_D = 0.00;
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

		gyro = new  ADXRS450_Gyro();
		gyro.reset();

		rotatePIDSource = new RotatePIDSource(this);
		rotatePID = new PIDController(kRotate_P, kRotate_I, kRotate_D, rotatePIDSource, new RotatePIDOutput(this));
		rotatePID.setAbsoluteTolerance(3.0);
		rotatePID.setOutputRange(-0.8, 0.8);

		straightPID = new PIDController(kStraight_P, kStraight_I, kStraight_D, new StraightPIDSource(this), new StraightPIDOutput(this));
		straightPID.setAbsoluteTolerance(100);
		straightPID.setOutputRange(-0.8, 0.8);
	}

	void runRotationPID(double setpoint) {
		//rotatePIDSource.setGyroNoReact(3.0);
		rotatePID.setPID(kRotate_P, kRotate_I, kRotate_D);
		rotatePID.setOutputRange(-0.8, 0.8);
		rotatePID.setAbsoluteTolerance(5);
		rotatePID.setSetpoint(setpoint);
		rotatePID.enable();
	}

	void stopRotationPID() {
		rotatePID.disable();
	}

	void runSpeedPID(double setpoint) {


		if(liftEncoder.getArmsHeight() >= 200) {
			straightPID.setOutputRange(-0.6, 0.6);
		}else{
			straightPID.setOutputRange(-0.8, 0.8);
		}

		rotatePIDSource.setGyroNoReact(5.0);
		rotatePID.setPID(kRotate_P_ForStraight, kRotate_I_ForStraight, kRotate_D_ForStraight);
		rotatePID.setOutputRange(-0.6, 0.6);
		rotatePID.setSetpoint(gyro.getAngle());
		rotatePID.enable();
		straightPID.setSetpoint(setpoint);
		straightPID.enable();
	}
	void stopSpeedPID() {
		rotatePID.disable();
		straightPID.disable();
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
		straightPID.disable();
		rotatePID.disable();
	}

	void teleopPeriodic() {
		handControl();
	}
}
class StraightPIDSource implements PIDSource {

	private PIDSourceType m_pidSource = PIDSourceType.kDisplacement;
	Drive drive;

	StraightPIDSource(Drive drive){
		this.drive = drive;
	}

	public double pidGet() {
		switch (m_pidSource) {
		case kDisplacement:
			return (drive.driveRightEncoder.getDistance() + drive.driveLeftEncoder.getDistance()) / 2;
		case kRate:
			return (drive.driveRightEncoder.getRate() + drive.driveLeftEncoder.getRate()) / 2;
		default:
			return 0.0;
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		m_pidSource = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return m_pidSource;
	}
}

class StraightPIDOutput implements PIDOutput {
	Drive drive;

	StraightPIDOutput(Drive drive){
		this.drive = drive;
	}

	public void pidWrite(double output) {
		drive.my_arcade_drive.arcadeDrive(output, drive.pid_rotate_output);
	}

}

class RotatePIDSource implements PIDSource {

	Drive drive;
	private PIDSourceType m_pidSource = PIDSourceType.kDisplacement;

	double gyro_value;
	double kGyroNoReact = 3.0; //初期化

	RotatePIDSource(Drive drive){
		this.drive = drive;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		m_pidSource = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return m_pidSource;
	}

	@Override
	public double pidGet() {
		if( Math.abs(drive.gyro.getAngle()) > kGyroNoReact) {
			gyro_value  = drive.gyro.getAngle();
		}else {
			gyro_value = 0.0;
		}
		return gyro_value;
	}

	public void setGyroNoReact(double kGyroNoReact) {
		this.kGyroNoReact = kGyroNoReact;
	}

}


class RotatePIDOutput implements PIDOutput {

	Drive drive;

	RotatePIDOutput(Drive drive){
		this.drive= drive;
	}

	@Override
	public void pidWrite(double output) {
		drive.pid_rotate_output = output;
		drive.my_arcade_drive.arcadeDrive(0.0, output);
	}

}
