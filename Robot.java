
package org.usfirst.frc.team6909.robot;


import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	// モーターコントローラ, エンコーダ, Relay, 距離センサのポート(RoboRio)
	private static final int kLeftFrontPort = 0;
	private static final int kLeftRearPort = 1;
	private static final int kRightFrontPort = 2;
	private static final int kRightRearPort = 3;
	private static final int kLiftMotorPort = 4;
	private static final int kLiftEncoderChannelAPort = 0; //Digital
	private static final int kLiftEncoderChannelBPort = 1; //Digital
	private static final int kDriveEncodeerChannelAPort = 6;
	private static final int kDriveEncoderChannelBPort = 7;
	private static final int kRelayPort = 0; //Relay
	private static final int kRightArmPort = 5;
	private static final int kLeftArmPort = 6;
	private static final int kLeftEyePingPort = 2; //Digital
	private static final int kLeftEyeEchoPort = 3;
	private static final int kRightEyePingPort = 4;
	private static final int kRightEyeEchoPort = 5;

	// Xboxコントローラのポート(PC)
	private static final int kXbox1Port = 1;
	private static final int kXbox2Port = 0;

	//エンコーダ関連
	private static final int kLiftEncoderMMPerPulse = 2; //[mm / pulse]
	private static final double armsOriginalHeightFromGround = 200;
	private static final double secondndColumnLengthMM = 1350;
	private static final double armsHeightOfItselfMM = 100;
	private static final double stringLengthMM = 1400;
	private static final double stringLengthLossMM = 50;
	private static final double kDriveEncoderMMPerPulse = 10;

	// 不感帯の大きさ
	private static final double kNoReact = 0.2;

	// 目的高さ
	private static final int kSwitchMiddle = 500;
	private static final int kSwitchHigh = 700;
	private static final int kScaleMiddle = 1700;
	private static final int kScaleHigh = 1900;
	private static final int kClimb  = 2200;

	// PID値
	private static final double kP = 0.01;
	private static final double kI = 0.01;
	private static final double kD = 0.01;

	//Autoに必要な値
	public int status;
	public int changer;
	private double DriveDistance1 = 36660;
	private double DistanceFromSwitch = 100;
	private double Angle1 = 90;

	// ドライブ用の宣言
	private Spark leftFront;
	private Spark leftRear;
	private Spark rightFront;
	private Spark rightRear;
	private SpeedControllerGroup leftMotors;
	private SpeedControllerGroup rightMotors;
	private DifferentialDrive my_arcade_drive;
	private Encoder DriveEncoder;

	// リフト用の宣言
	private Spark lift;
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


	// Xboxコントローラの宣言
	private XboxController xbox_drive;
	private XboxController xbox_lift;

	//Auto処理宣言
	private String gameData;
	private int location;
	private Timer timer;
	private ADXRS450_Gyro Gyro;

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

		lift = new Spark(kLiftMotorPort);
		//Encoder_withFのコンストラクタに渡す値はConstにまとめて7→3個にできる
		liftEncoder = new Encoder_withF(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort, armsOriginalHeightFromGround, secondndColumnLengthMM, armsHeightOfItselfMM, stringLengthMM,  stringLengthLossMM);
		liftEncoder.setDistancePerPulse(kLiftEncoderMMPerPulse); // using [mm] as unit would be good

		rightArm = new PWMTalonSRX(kRightArmPort);
		leftArm = new PWMTalonSRX(kLeftArmPort);
		my_arms = new SpeedControllerGroup(leftArm, rightArm);

		touch_floor = new Relay(kRelayPort);

		leftEye = new Ultrasonic(kLeftEyePingPort, kLeftEyeEchoPort, Unit.kMillimeters);
		rightEye = new Ultrasonic(kRightEyePingPort, kRightEyeEchoPort, Unit.kMillimeters);

		xbox_drive = new XboxController(kXbox1Port);
		xbox_lift = new XboxController(kXbox2Port);

		DriveEncoder = new Encoder(kDriveEncodeerChannelAPort, kDriveEncoderChannelBPort);
		DriveEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);

		Gyro = new ADXRS450_Gyro();
		timer = new Timer();

	}

	@Override
	public void autonomousInit() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		location = DriverStation.getInstance().getLocation();
		Gyro.reset();
		DriveEncoder.reset();
		status = 0;
		changer = 0;
	}


	@Override
	public void autonomousPeriodic() {
	if(gameData.charAt(0) == 'L' && location == 1) {
		//Phase1,2
				if(DriveEncoder.getDistance() < DriveDistance1 && Gyro.getAngle() < Angle1) {
					status = 1;
				}else if(DriveEncoder.getDistance() <= DriveDistance1 && Gyro.getAngle() < Angle1) {
					status = 2;
				}else {
					changer = 1;
				}
				//Phase3
				if(changer == 1 && leftEye.getRangeMM() >=  DistanceFromSwitch ) {
					status = 3;
				}else if(changer == 1) {
					changer = 2;
				}
				//Phase4
				if(changer == 2 && leftEye.getRangeMM() <  DistanceFromSwitch) {
					status = 4;
				}
				//Phase5
				if(liftEncoder.getArmsHeight() >= kSwitchHigh) {
					status = 5;
				}
	}

		//入力
		switch(status) {
		case 1:
		case 3:
			my_arcade_drive.arcadeDrive(1.0,0.0);
			break;
		case 2:
			my_arcade_drive.arcadeDrive(0.0,1.0);
		case 4:
			//リフト上昇
			break;
		case 5:
			//キューブ射出
			break;
		default:
			break;
		}

	}


	@Override
	public void teleopInit() {
		// PID用意
		lift_pidController = new PIDController(kP, kI, kD, liftEncoder, new LiftPidOutput());
		lift_pidController.enable();
	}

	@Override
	public void teleopPeriodic() {
		//Drive
		if( (Math.abs(xbox_drive.getY(Hand.kLeft)) < kNoReact) && (Math.abs(xbox_drive.getX(Hand.kRight)) < kNoReact) ) {
			my_arcade_drive.arcadeDrive(0.0, 0.0); // Stay
		}else if( (Math.abs(xbox_drive.getY(Hand.kLeft)) > kNoReact) && (Math.abs(xbox_drive.getX(Hand.kRight)) < kNoReact) ) {
			my_arcade_drive.arcadeDrive(xbox_drive.getY(Hand.kLeft), 0.0); // Drive forward/backward
		}else if( (Math.abs(xbox_drive.getY(Hand.kLeft)) < kNoReact) && (Math.abs(xbox_drive.getX(Hand.kRight)) > kNoReact) ) {
			my_arcade_drive.arcadeDrive(0.0, xbox_drive.getX(Hand.kRight)); // Turn right/left
		}else {
			my_arcade_drive.arcadeDrive(xbox_drive.getY(Hand.kLeft), xbox_drive.getX(Hand.kRight)); // Free drive
		}

		//Arm
		if( (xbox_lift.getTriggerAxis(Hand.kLeft) > kNoReact) && (xbox_lift.getTriggerAxis(Hand.kRight) < kNoReact) ) {
			my_arms.set(-xbox_lift.getTriggerAxis(Hand.kLeft)); // Get Cube
		}else if( (xbox_lift.getTriggerAxis(Hand.kLeft) > kNoReact) && (xbox_lift.getTriggerAxis(Hand.kRight) < kNoReact) ) {
			my_arms.set(xbox_lift.getTriggerAxis(Hand.kRight)); // Shoot Cube
		}else {
			my_arms.set(0.0); // Stay
		}


		// SWITCH用PID
		if(xbox_lift.getAButton() && xbox_lift.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SWITCH MIDDLE
			lift_pidController.setSetpoint(kSwitchMiddle);
		}else if(xbox_lift.getAButton() && xbox_lift.getBumper(Hand.kRight)) {
			// Lift up/down the arm SWITCH HIGH
			lift_pidController.setSetpoint(kSwitchHigh);
		}

		// SCALE & CLIMB用PID
		if(xbox_lift.getBButton() && xbox_lift.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SCALE MIDDLE
			lift_pidController.setSetpoint(kScaleMiddle);
		}else if(xbox_lift.getBButton() && xbox_lift.getBumper(Hand.kRight)) {
			// Lift up/down the arm SCALE HIGH
			lift_pidController.setSetpoint(kScaleHigh);
		}else if(xbox_lift.getBButton() && xbox_lift.getPOV() == 0) {
			// Lift up the arm CLIMB High;
			lift_pidController.setSetpoint(kClimb);
		}

	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {

	}
	private class LiftPidOutput implements PIDOutput{
		@Override
		public void pidWrite(double output) {
			lift.set(output);
		}
	}
}
