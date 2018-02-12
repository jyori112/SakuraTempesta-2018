package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/* ToDo
 * ・エンコーダー関連の定数調整
 * ・PID目標高さの定数調整
 * ・setInputRange()が意図通りに使えているか確認
 *
 */

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.XboxController;

public class Lift {
	//Port
	static final int kLiftMotorPort = 4;
	static final int kLiftEncoderChannelAPort = 0; //Digital
	static final int kLiftEncoderChannelBPort = 1; //Digital
	static final int kLiftMaxTouchChannel = 2;
	static final int kLiftMinTouchChannel = 3;
	//エンコーダ関連
	public EncoderWithNewFuncs liftEncoder;
	static final int kLiftEncoderMMPerPulse = 2; //[mm / pulse]
	static final double kArmsOriginalHeightFromGround = 200;
	static final double kSecondndColumnLengthMM = 1350;
	static final double kArmsHeightOfItselfMM = 100;
	static final double kStringLengthMM = 1400;
	static final double kStringLengthLossMM = 50;
	//PID目標高さ
	static final int kSwitchMiddle = 500;
	static final int kSwitchHigh = 700;
	static final int kScaleMiddle = 1700;
	static final int kScaleHigh = 1900;
	static final int kClimb = 2200;
	//モーター
	private PWMTalonSRX lift;
	//PID
	public PIDController lift_pidController;
	static final double LiftTolerance = 1.0;
	static final double kLift_P = 0.01; //調整中
	static final double kLift_I = 0.00; //基本0とする
	static final double kLift_D = 0.00; //基本0とする
	//不感帯
	static final double kNoReact = 0.1;

	//操作するコントローラ
	public XboxController xbox_ope;
	//右Y軸の値を格納
	private double x;
	//タッチセンサーの宣言
	private DigitalInput MaxTouch;
	private DigitalInput MinTouch;

	//重力のオフセット
	static final double kgravity = 0.7;

	//出力マックス値
	static final double kMaxOutput = 0.9;

	Lift(XboxController xbox_ope) {
		this.xbox_ope = xbox_ope;
		lift = new PWMTalonSRX(kLiftMotorPort);

		liftEncoder = new EncoderWithNewFuncs(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort,
				kArmsOriginalHeightFromGround, kSecondndColumnLengthMM, kArmsHeightOfItselfMM, kStringLengthMM,
				kStringLengthLossMM);
		liftEncoder.setDistancePerPulse(kLiftEncoderMMPerPulse); // using [mm] as unit would be good

		lift_pidController = new PIDController(kLift_P, kLift_I, kLift_D, liftEncoder, lift);
		lift_pidController.setEnabled(false);
		lift_pidController.setPercentTolerance(LiftTolerance);
		MaxTouch = new DigitalInput(kLiftMaxTouchChannel);
		MinTouch = new DigitalInput(kLiftMinTouchChannel);
		liftEncoder.setMaxPeriod(kClimb);
		lift_pidController.setOutputRange(kgravity, kMaxOutput);
	}

	void runPID(double setpoint) {//要再考
		lift_pidController.setInputRange(0, kClimb);//InputRangeを最大値で固定
		lift_pidController.setSetpoint(setpoint);
		lift_pidController.enable();
	}

	void stopPID() {
		lift_pidController.disable();
	}

	void handControl() {
		x = xbox_ope.getY(Hand.kRight);
		runPID((lift_pidController.getSetpoint() + 10 * x));
	}

	void teleopPeriodic() {

		if (xbox_ope.getAButton() && xbox_ope.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SWITCH MIDDLE
			runPID(kSwitchMiddle);
		} else if (xbox_ope.getAButton() && xbox_ope.getBumper(Hand.kRight)) {
			// Lift up/down the arm SWITCH HIGH
			runPID(kSwitchHigh);
		} else if (xbox_ope.getBButton() && xbox_ope.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SCALE MIDDLE
			runPID(kScaleMiddle);
		} else if (xbox_ope.getBButton() && xbox_ope.getBumper(Hand.kRight)) {
			// Lift up/down the arm SCALE HIGH
			runPID(kScaleHigh);
		} else if (xbox_ope.getBButton() && 350 < xbox_ope.getPOV() && xbox_ope.getPOV() < 10) {
			// Lift up the arm CLIMB High;
			runPID(kClimb);
		} else if (xbox_ope.getBumper(Hand.kRight) && xbox_ope.getBumper(Hand.kLeft)) {
			// 降下
			runPID(kArmsOriginalHeightFromGround);
			if (MinTouch.get()) {
				stopPID();
			}
		} else {
			handControl();
		}
	}

}
