package org.usfirst.frc.team6909.robot;

/* ToDo
 *
 * 今のとこなし
 *
 *
 */

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;

public class Arm {
	//Port
	private static final int kRightArmPort = 5;
	private static final int kLeftArmPort = 6;
	//不感帯大きさ
	private static final double kNoReact = 0.1;
	//モーター
	public PWMTalonSRX rightArm;
	public PWMTalonSRX leftArm;
	public SpeedControllerGroup my_arms;
	//操作するコントローラ
public XboxController xbox_ope;
	//Triggerの入力を格納
	double xr;
	double xl;

	Arm(XboxController xbox_ope) {
		this.xbox_ope = xbox_ope;
		rightArm = new PWMTalonSRX(kRightArmPort);
		leftArm = new PWMTalonSRX(kLeftArmPort);
		leftArm.setInverted(true);
		my_arms = new SpeedControllerGroup(leftArm, rightArm);
	}

	void handControl() {
		xr = xbox_ope.getTriggerAxis(Hand.kRight);
		xl = xbox_ope.getTriggerAxis(Hand.kLeft);
		if (this.xbox_ope.getTriggerAxis(Hand.kRight) > kNoReact && this.xbox_ope.getTriggerAxis(Hand.kLeft) < kNoReact) {
			my_arms.set(Util.outputCalc(kNoReact, xr));
			/*入力に等しい出力が欲しいときはこちら
			my_arms.set(xr);
			*/
		} else if (this.xbox_ope.getTriggerAxis(Hand.kLeft) > kNoReact && this.xbox_ope.getTriggerAxis(Hand.kRight ) < kNoReact) {
			my_arms.set(Util.outputCalc(kNoReact, -xl));
			/*入力に等しい出力が欲しい場合はこちら
			my_arms.set(-xl);
			*/
		} else {
			my_arms.set(0.0);
		}
	}

	void teleopPeriodic() {
		handControl();
	}

}