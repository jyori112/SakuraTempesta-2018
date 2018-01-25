package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";

	String autoSelected;

	SendableChooser<String> chooser = new SendableChooser<>();

	// モーターコントローラ, エンコーダ, Relay, 距離センサのポート(RoboRio)
	private static final int kLeftFrontPort = 0;
	private static final int kLeftRearPort = 1;
	private static final int kRightFrontPort = 2;
	private static final int kRightRearPort = 3;
	private static final int kLiftMotorPort = 4;
	private static final int kLiftEncoderChannelAPort = 0; //Digital
	private static final int kLiftEncoderChannelBPort = 1; //Digital
	private static final int kRelayPort = 0; //Relay
	private static final int kRightArmPort = 5;
	private static final int kLeftArmPort = 6;
	private static final int kLeftEyePingPort = 2; //Digital
	private static final int kLeftEyeEchoPort = 3;
	private static final int kRightEyePingPort = 4;
	private static final int kRightEyeEchoPort = 5;

	// Xboxコントローラのポート(PC)
	private static final int kXbox1Port = 0;
	private static final int kXbox2Port = 1;

	//エンコーダ関連
	private static final int kLiftEncoderMMPerPulse = 2; //[mm / pulse]
	private static final double armsOriginalHeightFromGround = 200;
	private static final double secondndColumnLengthMM = 1350;
	private static final double armsHeightOfItselfMM = 100;
	private static final double stringLengthMM = 1400;
	private static final double stringLengthLossMM = 50;

	// 不感帯の大きさ
	private static final double kNoReact = 0.2;

	// 目的高さ
	private static final int kGround = 0;
	private static final int kSwitchMiddle = 500;
	private static final int kSwitchHigh = 700;
	private static final int kScaleMiddle = 1700;
	private static final int kScaleHigh = 1900;
	private static final int kClimb = 2200;

	//オート処理
	//距離
	private static final int kSwitchDis = 36600;
	//Gyro関係
	private PIDController Gyro_Pid;
	private gyro_source gyro_source;
	private ADXRS450_Gyro gyrodeta;
	private static final int kAngle = 30;
	private static final double kkP = 0.01;
	private static final double kkI = 0.01;
	private static final double kkD = 0.01;

	// PID値
	private static final double kP = 0.01;
	private static final double kI = 0.01;
	private static final double kD = 0.01;

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
	private Encoder_withF liftEncoder;
	private Relay touch_floor;
	private PIDController lift_pidController;

	// アーム用の宣言
	private PWMTalonSRX rightArm;
	private PWMTalonSRX leftArm;
	private SpeedControllerGroup my_arms;

	//距離センサーの宣言
	private Ultrasonic leftEye;
	private Ultrasonic rightEye;

	//ジャイロセンサーの宣言
	private GyroBase GyroBase;
	private ADXRS450_Gyro gyro;

	// Xboxコントローラの宣言
	private XboxController xbox_drive;
	private XboxController xbox_lift;

	//タイマー起動
	private Timer timer;

	@Override
	public void robotInit() {
		/*
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);
		*/

		leftFront = new Spark(kLeftFrontPort);
		leftRear = new Spark(kLeftRearPort);
		rightFront = new Spark(kRightFrontPort);
		rightRear = new Spark(kRightRearPort);
		leftMotors = new SpeedControllerGroup(leftFront, leftRear);
		rightMotors = new SpeedControllerGroup(rightFront, rightRear);
		my_arcade_drive = new DifferentialDrive(leftMotors, rightMotors);

		lift = new PWMTalonSRX(kLiftMotorPort);
		//Encoder_withFのコンストラクタに渡す値はConstにまとめて7→3個にできる
		liftEncoder = new Encoder_withF(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort,
				armsOriginalHeightFromGround, secondndColumnLengthMM, armsHeightOfItselfMM, stringLengthMM,
				stringLengthLossMM);
		liftEncoder.setDistancePerPulse(kLiftEncoderMMPerPulse); // using [mm] as unit would be good

		rightArm = new PWMTalonSRX(kRightArmPort);
		leftArm = new PWMTalonSRX(kLeftArmPort);
		my_arms = new SpeedControllerGroup(leftArm, rightArm);
		leftArm.setInverted(true);

		touch_floor = new Relay(kRelayPort);

		leftEye = new Ultrasonic(kLeftEyePingPort, kLeftEyeEchoPort, Unit.kMillimeters);
		rightEye = new Ultrasonic(kRightEyePingPort, kRightEyeEchoPort, Unit.kMillimeters);

		xbox_drive = new XboxController(kXbox1Port);
		xbox_lift = new XboxController(kXbox2Port);

		gyro = new ADXRS450_Gyro();
		//accel = new ADXL362(Accelerometer.Range.k16G);

		timer = new Timer();
		gyro_source = new gyro_source();
		//カメラ起動
		CameraServer.getInstance().startAutomaticCapture();
		CameraServer.getInstance().getVideo();

		gyrodeta = new ADXRS450_Gyro();

	}

	@Override
	public void autonomousInit() {
		// PID用意
		lift_pidController = new PIDController(kP, kI, kD, liftEncoder, new LiftPidOutput());
		lift_pidController.setSetpoint(kSwitchHigh);
		Gyro_Pid = new PIDController(kkP, kkI, kkD, gyrodeta, new GyroPidOutput());
		Gyro_Pid.free();
		//gyro起動
		gyro.reset();
		//timer起動
		timer.start();
		timer.reset();
	}

	@Override
	public void autonomousPeriodic() {
		if (timer.get() < 5.0) {
			my_arcade_drive.arcadeDrive(1.0, 0.0);
		} else {
			if (Gyro_Pid.get() <= 30) {
				Gyro_Pid = new PIDController(kkP, kkI, kkD, gyrodeta, new GyroPidOutput());
				Gyro_Pid.enable();
				Gyro_Pid.setSetpoint(kAngle);
			} else {
				Gyro_Pid.free();
			}
			my_arcade_drive.arcadeDrive(1.0, 0.0);
		}

		/**timer.reset();
		if(timer.get() < 3.0) {
			my_arcade_drive.arcadeDrive(1.0, 0.0);
		}else {
			lift_pidController.setSetpoint(kSwitchHigh);
			my_arms.set(xbox_lift.getTriggerAxis(Hand.kRight));
			lift_pidController.setSetpoint(kGround);
		}**/

	}

	@Override
	public void teleopInit() {
		// PID用意
		lift_pidController = new PIDController(kP, kI, kD, liftEncoder, new LiftPidOutput());
		lift_pidController.enable();
		my_arms.set(1);
		//gyro起動
		gyro.reset();
		Gyro_Pid = new PIDController(kkP, kkI, kkD, gyrodeta, new GyroPidOutput());
		Gyro_Pid.free();
	}

	@Override
	public void teleopPeriodic() {
		//Drive
		if ((Math.abs(xbox_drive.getY(Hand.kLeft)) < kNoReact) && (Math.abs(xbox_drive.getX(Hand.kRight)) < kNoReact)) {
			my_arcade_drive.arcadeDrive(0.0, 0.0); // Stay
		} else if ((Math.abs(xbox_drive.getY(Hand.kLeft)) > kNoReact)
				&& (Math.abs(xbox_drive.getX(Hand.kRight)) < kNoReact)) {
			my_arcade_drive.arcadeDrive(xbox_drive.getY(Hand.kLeft), 0.0); // Drive forward/backward
		} else if ((Math.abs(xbox_drive.getY(Hand.kLeft)) < kNoReact)
				&& (Math.abs(xbox_drive.getX(Hand.kRight)) > kNoReact)) {
			my_arcade_drive.arcadeDrive(0.0, xbox_drive.getX(Hand.kRight)); // Turn right/left
		} else {
			my_arcade_drive.arcadeDrive(xbox_drive.getY(Hand.kLeft), xbox_drive.getX(Hand.kRight)); // Free drive
		}

		//Arm
		if ((xbox_lift.getTriggerAxis(Hand.kLeft) > kNoReact) && (xbox_lift.getTriggerAxis(Hand.kRight) < kNoReact)) {
			my_arms.set(-xbox_lift.getTriggerAxis(Hand.kLeft)); // Get Cube
		} else if ((xbox_lift.getTriggerAxis(Hand.kLeft) > kNoReact)
				&& (xbox_lift.getTriggerAxis(Hand.kRight) < kNoReact)) {
			my_arms.set(xbox_lift.getTriggerAxis(Hand.kRight)); // Shoot Cube
		} else {
			my_arms.set(0.0); // Stay
		}

		// SWITCH用PID
		if (xbox_lift.getAButton() && xbox_lift.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SWITCH MIDDLE
			lift_pidController.setSetpoint(kSwitchMiddle);
		} else if (xbox_lift.getAButton() && xbox_lift.getBumper(Hand.kRight)) {
			// Lift up/down the arm SWITCH HIGH
			lift_pidController.setSetpoint(kSwitchHigh);
		} else if (xbox_lift.getAButton() && xbox_lift.getBumper(Hand.kRight) && xbox_lift.getBumper(Hand.kLeft)) {
			lift_pidController.setSetpoint(kGround);
		}

		// SCALE & CLIMB用PID
		if (xbox_lift.getBButton() && xbox_lift.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SCALE MIDDLE
			lift_pidController.setSetpoint(kScaleMiddle);
		} else if (xbox_lift.getBButton() && xbox_lift.getBumper(Hand.kRight)) {
			// Lift up/down the arm SCALE HIGH
			lift_pidController.setSetpoint(kScaleHigh);
		} else if (xbox_lift.getBButton() && xbox_lift.getPOV() == 0) {
			// Lift up the arm CLIMB High;
			lift_pidController.setSetpoint(kClimb);
		} else if (xbox_lift.getBButton() && xbox_lift.getBumper(Hand.kRight) && xbox_lift.getBumper(Hand.kLeft)) {
			lift_pidController.setSetpoint(kGround);
		}

		//Gyro読み取り
		SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
		SmartDashboard.putNumber("Gyro Rate", gyro.getRate());
	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {

	}

	private class LiftPidOutput implements PIDOutput {
		@Override
		public void pidWrite(double output) {
			lift.set(output);
		}
	}

	private class GyroPidOutput implements PIDOutput {
		@Override
		public void pidWrite(double output) {
			my_arcade_drive.arcadeDrive(0, output);
		}
	}
}
