package org.usfirst.frc.team1.robot;

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
	private PWMTalonSRX rightArm;
	private PWMTalonSRX leftArm;
	private SpeedControllerGroup my_arms;
	//操作するコントローラ
	private XboxController xbox_ope;
	//Triggerの入力を格納
	double xr;
	double xl;

	Arm(XboxController xbox_ope) {
		this.xbox_ope = xbox_ope;
		rightArm = new PWMTalonSRX(kRightArmPort);
		leftArm = new PWMTalonSRX(kLeftArmPort);
		my_arms = new SpeedControllerGroup(leftArm, rightArm);
	}

	void handControl() {
		xr = xbox_ope.getTriggerAxis(Hand.kRight);
		xl = xbox_ope.getTriggerAxis(Hand.kLeft);

		rightArm.set(outputCalc(kNoReact, xr));
		leftArm.set(outputCalc(kNoReact, xl));
	}

	double outputCalc(double kNoReact, double input) {
		if (input > kNoReact) {
			//不感帯の正の端でy=0、x=1.0でy=1.0となる一次関数によって出力を計算
			return 1 / (1 - kNoReact) * input - kNoReact / (1 - kNoReact);
		}else if (input < -kNoReact){
			//不感帯の負の端でy=0、x=-1.0でy=-1.0となる一次関数によって出力を計算
			return 1 / (1 - kNoReact) * input + kNoReact / (1 - kNoReact);
		}else {
			return 0.0;
		}
	}
}