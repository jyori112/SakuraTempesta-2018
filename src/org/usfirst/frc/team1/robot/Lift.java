package org.usfirst.frc.team1.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.XboxController;

public class Lift {
	//Port
	private static final int kLiftMotorPort = 4;
	private static final int kLiftEncoderChannelAPort = 0; //Digital
	private static final int kLiftEncoderChannelBPort = 1; //Digital
    //エンコーダ関連
	private EncoderWithNewFuncs liftEncoder;
	private static final int kLiftEncoderMMPerPulse = 2; //[mm / pulse]
	private static final double kArmsOriginalHeightFromGround = 200;
	private static final double kSecondndColumnLengthMM = 1350;
	private static final double kArmsHeightOfItselfMM = 100;
	private static final double kStringLengthMM = 1400;
	private static final double kStringLengthLossMM = 50;
	//目標高さ
	private static final int kSwitchMiddle = 500;
	private static final int kSwitchHigh = 700;
	private static final int kScaleMiddle = 1700;
	private static final int kScaleHigh = 1900;
	private static final int kClimb = 2200;
	//モーター
	private Spark lift;
	//PID
	private PIDController lift_pidController;
	private static final double kLift_P = 0.01; //調整中
	private static final double kLift_I = 0.00; //基本0とする
	private static final double kLift_D = 0.00; //基本0とする
	//不感帯
	private static final double kNoReact = 0.1;
	//操作するコントローラ
	private XboxController xbox_ope;


	Lift(XboxController xbox_ope) {
		this.xbox_ope = xbox_ope;
		lift = new Spark(kLiftMotorPort);
		liftEncoder = new EncoderWithNewFuncs(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort,
				kArmsOriginalHeightFromGround, kSecondndColumnLengthMM, kArmsHeightOfItselfMM, kStringLengthMM,
				kStringLengthLossMM);
		liftEncoder.setDistancePerPulse(kLiftEncoderMMPerPulse); // using [mm] as unit would be good
		lift_pidController = new PIDController(kLift_P, kLift_I, kLift_D, liftEncoder, lift);
	}

	void teleop_control() {

		if (xbox_ope.getAButton() && xbox_ope.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SWITCH MIDDLE
			lift_pidController.setSetpoint(kSwitchMiddle);
			lift_pidController.enable();
		} else if (xbox_ope.getAButton() && xbox_ope.getBumper(Hand.kRight)) {
			// Lift up/down the arm SWITCH HIGH
			lift_pidController.setSetpoint(kSwitchHigh);
			lift_pidController.enable();
		} else if (xbox_ope.getBButton() && xbox_ope.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SCALE MIDDLE
			lift_pidController.setSetpoint(kScaleMiddle);
			lift_pidController.enable();
		} else if (xbox_ope.getBButton() && xbox_ope.getBumper(Hand.kRight)) {
			// Lift up/down the arm SCALE HIGH
			lift_pidController.setSetpoint(kScaleHigh);
			lift_pidController.enable();
		} else if (xbox_ope.getBButton() && xbox_ope.getPOV() == 0) {
			// Lift up the arm CLIMB High;
			lift_pidController.setSetpoint(kClimb);
			lift_pidController.enable();
		} else if (xbox_ope.getBumper(Hand.kRight) && xbox_ope.getBumper(Hand.kLeft)) {
			// 降下
			lift_pidController.setSetpoint(kArmsOriginalHeightFromGround);
			lift_pidController.enable();
		} else if (Math.abs(xbox_ope.getY(Hand.kLeft)) < kNoReact) {
			// 手動操作
			lift_pidController.disable();
			lift.set(xbox_ope.getY(Hand.kLeft));
		} else {
			lift_pidController.disable();
		}

	}
}
