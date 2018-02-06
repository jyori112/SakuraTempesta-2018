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
	//右Y軸の値を格納
	double x;

	Lift(XboxController xbox_ope) {
		this.xbox_ope = xbox_ope;
		lift = new Spark(kLiftMotorPort);
		liftEncoder = new EncoderWithNewFuncs(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort,
				kArmsOriginalHeightFromGround, kSecondndColumnLengthMM, kArmsHeightOfItselfMM, kStringLengthMM,
				kStringLengthLossMM);
		liftEncoder.setDistancePerPulse(kLiftEncoderMMPerPulse); // using [mm] as unit would be good
		lift_pidController = new PIDController(kLift_P, kLift_I, kLift_D, liftEncoder, lift);
	}

	void runPIDControl(double setPoint) {
		lift_pidController.setSetpoint(setPoint);
		lift_pidController.enable();
	}

	void stopPIDControl() {
		lift_pidController.disable();
	}

	void handControl() {
		x = xbox_ope.getY(Hand.kRight);
		lift.set(outputCalc(kNoReact, x));
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
