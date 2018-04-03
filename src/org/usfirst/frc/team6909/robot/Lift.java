package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/* ToDo
 * ・最低動作出力の確認 → 出力の下限がそこになる新しい関数を用意?
 *
 *
 *
 */

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {
	//Port
	static final int kLiftMotorPort = 4;
	static final int kLiftEncoderChannelAPort = 0; //Digital
	static final int kLiftEncoderChannelBPort = 1; //Digital
    static final int kLiftBottomSwitchPort = 2; //Digital
    static final int kLiftHighestSwitchPort = 3; //Digital
	//エンコーダ関連
	public EncoderWithNewFuncs liftEncoder;
	static final double kLiftEncoderMMPerPulse = 50.08 * Math.PI / (12.75 * 20);
	static final double kCubeOriginalHeightFromE1 = -80;
	static final double kE2OriginalHeightFromGround = 50;
	static final double kE2LengthMM = 1285;
	static final double kArmsHeightOfItselfMM = 245;
	static final double kStringLengthMM = 1107;
	static final double kStringLengthLossMM = 47;
	//PID目標高さ
	static final int kSwitchMiddle = 240;
	static final int kSwitchHigh = 400;
	static final int kScaleMiddle = 1610;
	static final int kScaleHigh = 1910;
	static final int kMaxArmHeight = 2100;
	//モーター
	public PWMTalonSRX lift;
	//Limit Switch
	public DigitalInput liftHighestSwitch; //falseでつながってる
	public DigitalInput liftBottomSwitch;
	//PID
	public PIDController lift_pidController;
	static final double LiftAbsoluteTolerance = 50; //許容範囲
	static final double kLift_P = 0.01;
	static final double kLift_I = 0.00; //基本0とする
	static final double kLift_D = 0.01; //基本0とする
	static final double kOutputResistingGravity = 0.2; //batteryの充電具合によって変動
	//不感帯
	static final double kNoReact = 0.1;

	//操作するコントローラ
	public XboxController xbox_ope;
	//右Y軸の値を格納
	private double x;

	Lift(XboxController xbox_ope) {
		this.xbox_ope = xbox_ope;
		lift = new PWMTalonSRX(kLiftMotorPort);
		lift.setInverted(true); //liftモーターは逆転させる
		liftEncoder = new EncoderWithNewFuncs(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort, false,
				kCubeOriginalHeightFromE1, kE2OriginalHeightFromGround,kE2LengthMM, kArmsHeightOfItselfMM, kStringLengthMM,
				kStringLengthLossMM);

		liftEncoder.setDistancePerPulse(kLiftEncoderMMPerPulse); // using [mm] as unit would be good
		lift_pidController = new PIDController(kLift_P, kLift_I, kLift_D, liftEncoder, new LiftPIDOutput(lift));
		lift_pidController.setEnabled(false);
		lift_pidController.setAbsoluteTolerance(LiftAbsoluteTolerance);
		lift_pidController.setInputRange(0, kMaxArmHeight);
		lift_pidController.setOutputRange(-(0.7-kOutputResistingGravity), 0.5);

		liftBottomSwitch = new DigitalInput(kLiftBottomSwitchPort);
		liftHighestSwitch = new DigitalInput(kLiftHighestSwitchPort);
	}

	void runPID(double setpoint) {
			lift_pidController.setSetpoint(setpoint);
			lift_pidController.enable();
	}

	void stopPID() {
		lift_pidController.disable(); //この結果自重で落ちていく
	}

	void handControl() {
		//x = xbox_ope.getY(Hand.kLeft);

		if (xbox_ope.getTriggerAxis(Hand.kRight) > kNoReact && xbox_ope.getTriggerAxis(Hand.kLeft) < kNoReact) {
			x = - xbox_ope.getTriggerAxis(Hand.kRight);
		}else if (xbox_ope.getTriggerAxis(Hand.kRight) < kNoReact && xbox_ope.getTriggerAxis(Hand.kLeft) > kNoReact) {
			x = xbox_ope.getTriggerAxis(Hand.kLeft);
		}else if (liftEncoder.getArmsHeight() < 200){
			x = 1.7 * kOutputResistingGravity;
		}else {
			x = 0;
		}

		if (xbox_ope.getStartButton()) {
			lift.set(Util.outputCalc(kNoReact, x) - kOutputResistingGravity);
		}else {
			lift.set( Util.outputCalc(kNoReact, x) / 1.7 - kOutputResistingGravity);
			/*入力に等しい出力が欲しいならこちら
			lift.set(x);
			 */
		}

		if (!liftBottomSwitch.get() && xbox_ope.getTriggerAxis(Hand.kRight) < kNoReact) {
			liftEncoder.reset();
		}

	}

	void teleopInit() {
		lift_pidController.setOutputRange(-(0.7-kOutputResistingGravity), 0.7);
	}

	void teleopPeriodic() {
		if (xbox_ope.getAButton() &&  xbox_ope.getPOV() == 0) {
			runPID(kSwitchHigh);
		} else if (xbox_ope.getBButton() && xbox_ope.getPOV() == 180) {
			// Scaleのつり合いまで
			runPID(kScaleMiddle);
		} else if (xbox_ope.getBButton() && xbox_ope.getPOV() == 0) {
			// Scaleの高いほう
			runPID(kScaleHigh);
		}
		/*
		else if (xbox_ope.getBButton() && xbox_ope.getAButton()) { //handConntrolで代用できる
		// Climb
			runPID(kMaxArmHeight);
			if (!liftHeighestSwitch.get()) {
				stopPID();
				lift.set(kOutputResistingGravity);
			}
		} else if (xbox_ope.getXButton() && xbox_ope.getYButton()) { //handControlで代用できる
			// 降下
			runPID(kCubeOriginalHeightFromE1);
			if (!liftBottomSwitch.get()) {
				stopPID();
				liftEncoder.reset();
			}
		}
		*/
		 else {
			// 手動操作
			stopPID();
			handControl();
		}

		SmartDashboard.putNumber("LiftMotorOutput", lift.get());
		SmartDashboard.putBoolean("HeighestSwitch", liftHighestSwitch.get());
		SmartDashboard.putBoolean("BottomSwitch", liftBottomSwitch.get());
	}

}

class LiftPIDOutput implements PIDOutput {

	PWMTalonSRX lift;

	LiftPIDOutput (PWMTalonSRX lift) {
		this.lift = lift;
	}

	@Override
	public void pidWrite(double output) {
		lift.set(-(output + Lift.kOutputResistingGravity));
	}
}
