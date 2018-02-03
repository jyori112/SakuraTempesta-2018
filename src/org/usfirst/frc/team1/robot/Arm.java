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

	Arm(XboxController xbox_ope) {
		this.xbox_ope = xbox_ope;
		rightArm = new PWMTalonSRX(kRightArmPort);
		leftArm = new PWMTalonSRX(kLeftArmPort);
		my_arms = new SpeedControllerGroup(leftArm, rightArm);
	}

	void teleop_control() {
		if ((xbox_ope.getTriggerAxis(Hand.kLeft) > kNoReact) && (xbox_ope.getTriggerAxis(Hand.kRight) < kNoReact)) {
			my_arms.set(-xbox_ope.getTriggerAxis(Hand.kLeft)); // Get Cube
		} else if ((xbox_ope.getTriggerAxis(Hand.kLeft) > kNoReact)	&& (xbox_ope.getTriggerAxis(Hand.kRight) < kNoReact)) {
			my_arms.set(xbox_ope.getTriggerAxis(Hand.kRight)); // Shoot Cube
		} else {
			my_arms.set(0.0); // Stay
		}
	}
}