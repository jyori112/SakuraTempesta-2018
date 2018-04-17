package org.usfirst.frc.team69.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drive_BASE {
	//Port
	private static final int kLeftFrontPort = 0;
	private static final int kLeftRearPort = 1;
	private static final int kRightFrontPort = 2;
	private static final int kRightRearPort = 3;
	//Gyro関連
	public ADXRS450_Gyro gyro;
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
	public final double rightModifier = 1.1;
	public final double leftModifier = 1.0; //12.0Vの時0.89, 12.6Vの時0.93 12.2-0.94, 12.7 -- 1.1

	Drive_BASE(XboxController xbox_drive, EncoderWithNewFuncs liftEncoder) {
		this.xbox_drive = xbox_drive;
		this.liftEncoder = liftEncoder;
		leftFront = new Spark(kLeftFrontPort);
		leftRear = new Spark(kLeftRearPort);
		rightFront = new Spark(kRightFrontPort);
		rightRear = new Spark(kRightRearPort);
		leftMotors = new SpeedControllerGroup(leftFront, leftRear);
		rightMotors = new SpeedControllerGroup(rightFront, rightRear);
		my_arcade_drive = new DifferentialDrive(leftMotors, rightMotors);

		gyro = new ADXRS450_Gyro();
		gyro.reset();
	}

	void handControl() {
		xfb = xbox_drive.getY(Hand.kLeft);
		xlr = xbox_drive.getX(Hand.kRight);
		//yukkuri = xbox_drive.getTriggerAxis(Hand.kRight);

		if (liftEncoder.getArmsHeight() >= 200) {
			if (xfb >= 0.5) {
				my_arcade_drive.arcadeDrive(-0.5, Util.outputCalc(kNoReact, xlr));
			} else if (xfb <= -0.5) {
				my_arcade_drive.arcadeDrive(0.5, Util.outputCalc(kNoReact, xlr));
			} else {
				my_arcade_drive.arcadeDrive(Util.outputCalc(kNoReact, -xfb), Util.outputCalc(kNoReact, xlr));
			}
		} else {
			my_arcade_drive.arcadeDrive(Util.outputCalc(kNoReact, -xfb) , Util.outputCalc(kNoReact, xlr) );
			/* 入力に等しい出力が欲しいならこちら。
			my_arcade_drive.arcadeDrive(xfb, xlr);
			*/
		}
	}

	void teleopInit() {

	}

	void teleopPeriodic() {
		handControl();
	}
}




