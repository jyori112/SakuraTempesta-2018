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
    static final int kLiftUpperLimitSwitchPort = 2; //Digital
    static final int kLiftBottomLimitSwitchPort = 3; //Digital
	//エンコーダ関連
	public EncoderWithNewFuncs liftEncoder;
	static final double kLiftEncoderMMPerPulse = 50.4 * Math.PI / 12.75; //調整
	static final double kArmsOriginalHeightFromE1 = 200; //要調整
	static final double kE2OriginalHeightFromGround = 50;
	static final double kE2LengthMM = 1285;
	static final double kArmsHeightOfItselfMM = 245;
	static final double kStringLengthMM = 1107;
	static final double kStringLengthLossMM = 47;
	//PID目標高さ
	static final int kSwitchMiddle = 500; //要調整
	static final int kSwitchHigh = 700;
	static final int kScaleMiddle = 1610;
	static final int kScaleHigh = 1910;
	static final int kMaxArmHeight = 2100;
	//モーター
	private PWMTalonSRX lift;
	//Limit Switch
	public DigitalInput liftUpperLimitSwitch;
	public DigitalInput liftBottomLimitSwitch;
	//PID
	public PIDController lift_pidController;
	static final double LiftTolerance = 1.0; //許容範囲
	static final double kLift_P = 0.01; //調整中
	static final double kLift_I = 0.00; //基本0とする
	static final double kLift_D = 0.00; //基本0とする
	static final double kOutputResistingGravity = 0.3; //要調整
	//不感帯
	static final double kNoReact = 0.1;

	//操作するコントローラ
	public XboxController xbox_ope;
	//右Y軸の値を格納
	private double x;

	Lift(XboxController xbox_ope) {
		this.xbox_ope = xbox_ope;
		lift = new PWMTalonSRX(kLiftMotorPort);
		liftEncoder = new EncoderWithNewFuncs(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort,
				kArmsOriginalHeightFromE1, kE2OriginalHeightFromGround,kE2LengthMM, kArmsHeightOfItselfMM, kStringLengthMM,
				kStringLengthLossMM);
		liftEncoder.setDistancePerPulse(kLiftEncoderMMPerPulse); // using [mm] as unit would be good
		lift_pidController = new PIDController(kLift_P, kLift_I, kLift_D, liftEncoder, lift);
		lift_pidController.setEnabled(false);
		lift_pidController.setPercentTolerance(LiftTolerance);
		lift_pidController.setInputRange(0, kMaxArmHeight);
		lift_pidController.setOutputRange(kOutputResistingGravity, 1.0); //重力でずり落ちないようにする

		liftUpperLimitSwitch = new DigitalInput(kLiftUpperLimitSwitchPort);
		liftBottomLimitSwitch = new DigitalInput(kLiftBottomLimitSwitchPort);
	}

	void runPID(double setpoint) {
		lift_pidController.setSetpoint(setpoint);
		lift_pidController.enable();
	}

	void stopPID() {
		lift_pidController.disable();
	}

	void handControl() {
		x = xbox_ope.getY(Hand.kRight);

		lift.set(Util.outputCalc(kNoReact, x));
		/*入力に等しい出力が欲しいならこちら
		lift.set(x);
		*/
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
			runPID(kMaxArmHeight);
			if (liftUpperLimitSwitch.get()) {
				stopPID();
				lift.set(kOutputResistingGravity);
			}
		} else if (xbox_ope.getBumper(Hand.kRight) && xbox_ope.getBumper(Hand.kLeft)) {
			// 降下
			runPID(kArmsOriginalHeightFromE1); //Relay使用に変更予定
			if (liftBottomLimitSwitch.get()) {
				stopPID();
				liftEncoder.reset();
			}
		} else {
			// 手動操作
			stopPID();
			handControl();
		}
	}

}
