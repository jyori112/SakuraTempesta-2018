package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
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
	private static final int kLiftMotorPort =4;
	private static final int kLiftEncoderChannelAPort = 0; //Digital Input
	private static final int kLiftEncoderChannelBPort = 1; //Digital Input
	private static final int kRelayPort = 0; //Digital Input
	private static final int kRightArmPort = 5;
	private static final int kLeftArmPort = 6;
	private static final int kLeftEyePingPort = 1;
	private static final int kLeftEyeEchoPort = 2;
	private static final int kRightEyePingPort = 3;
	private static final int kRightEyeEchoPort = 4;

	// Xboxコントローラのポート(PC)
	private static final int kXbox1Port = 0;
	private static final int kXbox2Port = 1;

	//エンコーダ1Pulseごとの移動距離
	private static final int kLiftEncoderMMPerPulse = 2; //[mm / pulse]
	// 不感帯の大きさ
	private static final double kNoReact = 0.2;

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
	private XboxController xbox_drive;
	private XboxController xbox_lift;

	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		SmartDashboard.putData("Auto choices", chooser);

		leftFront = new Spark(kLeftFrontPort);
		leftRear = new Spark(kLeftRearPort);
		rightFront = new Spark(kRightFrontPort);
		rightRear = new Spark(kRightRearPort);
		leftMotors = new SpeedControllerGroup(leftFront, leftRear);
		rightMotors = new SpeedControllerGroup(rightFront, rightRear);
		my_arcade_drive = new DifferentialDrive(leftMotors, rightMotors);

		lift = new PWMTalonSRX(kLiftMotorPort);
		liftEncoder = new Encoder(kLiftEncoderChannelAPort, kLiftEncoderChannelBPort);
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
	}


	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		System.out.println("Auto selected" + autoSelected);
	}


	@Override
	public void autonomousPeriodic() {
		switch(autoSelected) {
		case customAuto:
			// Put custom auto code here

			break;
		case defaultAuto:
			// Put default auto code here

			break;
		}
	}

	@Override
	public void teleopInit() {

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
		}else if(xbox_lift.getAButton() && xbox_lift.getBumper(Hand.kRight)) {
			// Lift up/down the arm SWITCH HIGH
		}

		// SCALE & CLIMB用PID
		if(xbox_lift.getBButton() && xbox_lift.getBumper(Hand.kLeft)) {
			// Lift up/down the arm SCALE MIDDLE
		}else if(xbox_lift.getBButton() && xbox_lift.getBumper(Hand.kRight)) {
			// Lift up/down the arm SCALE HIGH
		}else if(xbox_lift.getBButton() && xbox_lift.getPOV() == 0) {
			// Lift up the arm CLIMB high
		}

	}

	@Override
	public void testInit() {

	}

	@Override
	public void testPeriodic() {

	}

}

