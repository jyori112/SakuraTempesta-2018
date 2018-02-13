package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.DigitalInput;

/* ToDo
 *
 *
 * 今のところなし
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
    static final int kLiftHeighestSwitchPort = 2; //Digital
    static final int kLiftBottomSwitchPort = 3; //Digital
	//エンコーダ関連
	public EncoderWithNewFuncs liftEncoder;
	static final double kLiftEncoderMMPerPulse = 45.08 * Math.PI / (12.75 * 20); //調整中
	static final double kCubeOriginalHeightFromE1 = -80;
	static final double kE2OriginalHeightFromGround = 50;
	static final double kE2LengthMM = 1285;
	static final double kArmsHeightOfItselfMM = 245;
	static final double kStringLengthMM = 1107;
	static final double kStringLengthLossMM = 47;
	//PID目標高さ
	static final int kSwitchMiddle = 240; //調整中
	static final int kSwitchHigh = 400;
	static final int kScaleMiddle = 1610;
	static final int kScaleHigh = 1910;
	static final int kMaxArmHeight = 2100;
	//モーター
	public PWMTalonSRX lift;
	//Limit Switch
	public DigitalInput liftHeighestSwitch; //falseでつながってる
	public DigitalInput liftBottomSwitch;
	//PID
	public PIDController lift_pidController;
	static final double LiftAbsoluteTolerance = 20; //許容範囲
	static final double kLift_P = 0.01; //調整中
	static final double kLift_I = 0.00; //基本0とする
	static final double kLift_D = 0.00; //基本0とする
	static final double kOutputResistingGravity = 0.34; //調整済み
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

		liftHeighestSwitch = new DigitalInput(kLiftHeighestSwitchPort);
		liftBottomSwitch = new DigitalInput(kLiftBottomSwitchPort);
	}

	void runPID(double setpoint) {
		if (lift_pidController.onTarget()) {
			lift_pidController.setInputRange(kOutputResistingGravity, 0.5);
		}else{
			lift_pidController.setOutputRange(-0.5, 0.5);
			lift_pidController.setSetpoint(setpoint);
			lift_pidController.enable();
		}
	}

	void stopPID() {
		lift_pidController.disable();
	}

	void handControl() {
		x = xbox_ope.getY(Hand.kLeft);

		lift.set( Util.outputCalc(kNoReact, x) / 2.0 - kOutputResistingGravity);
		/*入力に等しい出力が欲しいならこちら
		lift.set(x);
		*/

		if (!liftBottomSwitch.get()) {
			liftEncoder.reset();
		}
	}

	void teleopPeriodic() {
		if (xbox_ope.getAButton() && xbox_ope.getBumper(Hand.kLeft)) {
			// Switchのつり合いの高さまでPIDで持ち上げる
			runPID(kSwitchMiddle);
		} else if (xbox_ope.getAButton() && xbox_ope.getBumper(Hand.kRight)) {
			// Switchの最大の高さまでPID
			runPID(kSwitchHigh);
		} else if (xbox_ope.getBButton() && xbox_ope.getBumper(Hand.kLeft)) {
			// Scaleのつり合いまで
			runPID(kScaleMiddle);
		} else if (xbox_ope.getBButton() && xbox_ope.getBumper(Hand.kRight)) {
			// Scaleの高いほう
			runPID(kScaleHigh);
		} else if (xbox_ope.getBButton() && 350 < xbox_ope.getPOV() && xbox_ope.getPOV() < 10) {
			// Climb
			runPID(kMaxArmHeight);
			if (!liftHeighestSwitch.get()) {
				stopPID();
				lift.set(kOutputResistingGravity);
			}
		} else if (xbox_ope.getBumper(Hand.kRight) && xbox_ope.getBumper(Hand.kLeft)) {
			// 降下
			runPID(kCubeOriginalHeightFromE1);
			if (!liftBottomSwitch.get()) {
				stopPID();
				liftEncoder.reset();
			}
		} else {
			// 手動操作
			stopPID();
			handControl();
		}

		SmartDashboard.putNumber("LiftMotorOutput", lift.get());
		SmartDashboard.putBoolean("HeighestSwitch", liftHeighestSwitch.get());
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
		lift.set(-output);
	}
}
