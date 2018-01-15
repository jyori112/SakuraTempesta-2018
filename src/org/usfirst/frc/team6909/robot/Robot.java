package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	// モーターコントローラ及びエンコーダのポート(RoboRio)
	private static final int kLeftFrontPort = 0;
	private static final int kLeftRearPort = 1;
	private static final int kRightFrontPort = 2;
	private static final int kRightRearPort = 3;
	private static final int kLiftMotorPort =4;
	private static final int kLiftEncoderChannelAPort = 0; //Analog Input
	private static final int kLiftEncoderChannelBPort = 1; //Analog Input
	private static final int kRelayPort = 0; //Digital Input
	private static final int kRightArmPort = 5;
	private static final int kLeftArmPort = 6;
	private static final int kLeftEyePingPort = 1;
	private static final int kLeftEyeEchoPort = 2;
	private static final int kRightEyePingPort = 3;
	private static final int kRightEyeEchoPort = 4;

	// Xboxコントローラのポート(PC)
	private static final int kXbox1Port = 0;

	// ドライブ用の宣言
	private Spark leftFront;
	private Spark leftRear;
	private Spark rightFront;
	private Spark rightRear;
	private SpeedControllerGroup leftMotors;
	private SpeedControllerGroup rightMotors;
	private DifferentialDrive my_arcade_drive;

	// リフト用の宣言
	private PWMTalonSRX lift;
	private Encoder liftEncoder;
	private Relay touch_floor;

	// アーム用の宣言
	private PWMTalonSRX rightArm;
	private PWMTalonSRX leftArm;
	private SpeedControllerGroup my_arms;

	//距離センサーの宣言
	private Ultrasonic leftEye;
	private Ultrasonic rightEye;


	// Xboxコントローラの宣言
	private XboxController xbox_1;

	@Override
	public void robotInit() {

		leftFront = new Spark(kLeftFrontPort);
		leftRear = new Spark(kLeftRearPort);
		rightFront = new Spark(kRightFrontPort);
		rightRear = new Spark(kRightRearPort);
		leftMotors = new SpeedControllerGroup(leftFront, leftRear);
		rightMotors = new SpeedControllerGroup(rightFront, rightRear);
		my_arcade_drive = new DifferentialDrive(leftMotors, rightMotors);

		lift = new PWMTalonSRX(kLiftMotorPort);
		liftEncoder = new Encoder(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort);
		liftEncoder.setDistancePerPulse(2); // using [mm] as unit would be good

		rightArm = new PWMTalonSRX(kRightArmPort);
		leftArm = new PWMTalonSRX(kLeftArmPort);
		my_arms = new SpeedControllerGroup(leftArm, rightArm);
		leftArm.setInverted(true);

		touch_floor = new Relay(kRelayPort);

		leftEye = new Ultrasonic(kLeftEyePingPort, kLeftEyeEchoPort, Unit.kMillimeters);
		rightEye = new Ultrasonic(kRightEyePingPort, kRightEyeEchoPort, Unit.kMillimeters);

		xbox_1 = new XboxController(kXbox1Port);

	}


	@Override
	public void autonomousInit() {

	}


	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {

	}

	@Override
	public void teleopPeriodic() {
		// 運転
		my_arcade_drive.arcadeDrive(xbox_1.getY(Hand.kLeft), xbox_1.getX(Hand.kLeft));

		// cubeをゲット
		my_arms.set(-xbox_1.getTriggerAxis(Hand.kLeft));
		// cubeを射出
		my_arms.set(xbox_1.getTriggerAxis(Hand.kRight));

		// switch <二種類の高さ(最も高いHIGHと中間のMIDDLE)に対応>
		if( xbox_1.getAButton() ) {
			// A + ↑ でHIGHに至るまで全速力UP → 超えたら速度0にする
			if( xbox_1.getPOV() == 0 && ( liftEncoder.getDistance() < 710 ) ) { //エンコーダの値によって計算する距離の閾値は要調整
				lift.set(1.0);
			}else if( xbox_1.getPOV() == 0 && ( liftEncoder.getDistance() > 710 ) ) {
				lift.set(0.0);
			}
			// A + ↓ でMIDDLEに至るまで全速力UP → 超えたら速度0にする
			if( xbox_1.getPOV() == 180 && ( liftEncoder.getDistance() < 550 ) ) {
				lift.set(1.0);
			}else if( xbox_1.getPOV() == 180 && ( liftEncoder.getDistance() > 550) ) {
				lift.set(0.0);
			}
		}

		// scale <三種類の高さ(HIGHとMIDDLEとクライム用の高さCLIMB)に対応>
		if( xbox_1.getBButton() ) {
			// B + ↑ でHIGHに至るまで全速力UP → 超えたら速度0にする
			if( xbox_1.getPOV() == 0 && ( liftEncoder.getDistance() < 1900) ) { //PID制御を導入予定
				lift.set(1.0);
			}else if( xbox_1.getPOV() == 0 && ( liftEncoder.getDistance() > 1900) ) {
				lift.set(0.0);
			}
			// B + ↓ でMIDDLEに至るまで全速力UP → 超えたら速度0にする
			if( xbox_1.getPOV() == 180 && ( liftEncoder.getDistance() < 1500 ) ) {
				lift.set(1.0);
			}else if( xbox_1.getPOV() == 180 && (liftEncoder.getDistance() > 1500) ) {
				lift.set(0.0);
			}
			// B + ← でCLIMBに至るまで全速力UP → 超えたら速度0にする
			if( xbox_1.getPOV() == 270 && ( liftEncoder.getDistance() < 2200) ) {
				lift.set(1.0);
			}else if( xbox_1.getPOV() == 270 && ( liftEncoder.getDistance() > 2200) ) {
				lift.set(0.0);
			}
		}

		// lift <liftが床(正確には初期位置)についているか判断するrelayを使う>
		if( xbox_1.getYButton() )  {
			// Y + (床についていない) で全速力DOWN → 床についたら速度0にして、エンコーダの値によって計算する距離を0にリセット
			if(touch_floor.get() == Value.kOff) {
				lift.set(-1.0);
			}else if(touch_floor.get() == Value.kOn) {
				lift.set(0.0);
				liftEncoder.reset();
			}
		}

		// 姿勢調整 <ロボットのドライブトレインの前面の左右にひとつずつ付けた距離センサーの値を使う>
		if( xbox_1.getXButton() ) {
			// Y + (正面したい面に対して右斜めに侵入) で50%右回転
			if( leftEye.getRangeMM() > rightEye.getRangeMM() ) {
				leftMotors.set(0.5);
				rightMotors.set(-0.5);
			}
			// Y + (正面したい面に対して左斜めに侵入) で50%左回転
			if( leftEye.getRangeMM() < rightEye.getRangeMM() ) {
				leftMotors.set(-0.5);
				rightMotors.set(0.5);
			}
			// 正面にいるとき停止
			if( leftEye.getRangeMM() == rightEye.getRangeMM() ) {
				leftMotors.set(0.0);
				rightMotors.set(0.0);
			}
		}

	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {

	}

}

