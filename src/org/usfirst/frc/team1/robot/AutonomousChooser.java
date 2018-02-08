package org.usfirst.frc.team1.robot;

import edu.wpi.first.wpilibj.Timer;

public class AutonomousChooser {

	Drive drive;
	Lift lift;
	Arm arm;
	Timer timer;

	String gameData;
	int location;

	String status;
	String forward = "forward";
	String rotate = "rotate";
	String up = "up";
	String shoot = "shoot";
	String end = "end";
	int cnt_forward;
	int cnt_rotate;
	int cnt_up;
	int cnt_shoot;

	//Turnのために壁から少し移動
	static final int kForwardBitToTurn = 50;
	//全速力Scale
	static final int kForwardZeroToScale = 7000; //値は仮
	static final int kTurnRightToScaleToShoot = 90;
	static final int kTurnLeftToScaleToShoot = -90;
	//回り込みScale
	static final int kTurnRightToOuterSwitchAngle = 55;
	static final int kTurnLeftToOuterSwitchAngle = -55;
	static final int kForwardZeroToOuterSwitch = 3000;
	static final int kTurnRightToScaleAngle = 90;
	static final int kTurnLeftToScaleAngle = -90;
	static final int kForwardOuterToScaleAndShoot = 4000;
	//全力Switch
	static final int kTurnRightToInerSwitchAngle = 45;
	static final int kTurnLeftToInnerSwitchAngle = -45;
	static final int kForwardZeroToInnerSwitch = 2000;
	static final int kTurnRightToSwitchToShoot = 135;
	static final int kTurnLeftToSwitchToShoot = -135;
	//戻ってきてSwitch
	static final int kForwardZeroToOverSwitch = 5000;
	static final int kTurnRightToReturnToSwitchAngle = 150;
	static final int kTurnLeftToReturnToSwitchAngle = -150;
	static final int kForwardReturnToSwitchAndShoot = 2000;
	//通過して待機
	static final int kForwardZeroToMiddleOfField = 6000;
	static final int kTurnJustRight = 90;
	static final int kTurnJustLeft = -90;
	static final int kForwardToMiddle = 1000;
	//armのshootパワー
	static final double kShootPower = 0.5;
	//PID許容範囲%
	static int DriveSpeedTolerance = 3;
	static int DriveRotationTolerane = 3;
	static int LiftTolerance = 3;

	AutonomousChooser(String gameData, int location, Drive drive, Lift lift, Arm arm) {
		this.gameData = gameData;
		this.location = location;
		this.drive = drive;
		this.lift = lift;
		this.arm = arm;

		timer = new Timer();

		status = null;
		cnt_forward = 1;
		cnt_rotate = 1;
		cnt_up = 1;
		cnt_shoot = 1;
	}

	void autonomousInit() {

	}

	void autonomousPeriodic() {
		if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
			// 全速力Scale or 戻ってきてSwitch
		}else if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
			// 戻ってきてSwitch
			Start(forward);
			DriveForward(rotate, kForwardZeroToOverSwitch, 50);
			DriveRotate(forward, kTurnRightToReturnToSwitchAngle, 50);
			DriveForward(up, kForwardReturnToSwitchAndShoot, 50);
			LiftUp(shoot, Lift.kSwitchHigh, 50);
			ArmShoot(end, kShootPower, 50);
			End();
		}else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
			// 全速力Scale
			Start(forward);
			DriveForward(rotate, kForwardZeroToScale, 50);
			DriveRotate(forward,  kTurnRightToScaleToShoot, 50);
			LiftUp(shoot, Lift.kScaleHigh, 50);
			ArmShoot(end, kShootPower, 50);
			End();
		}else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
			// フィールド中心へ
			Start(forward);
			DriveForward(rotate, kForwardZeroToMiddleOfField, 50);
			DriveRotate(forward, kTurnJustRight, 50);
			DriveForward(end, kForwardToMiddle, 50);
			End();
		}else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
			//
		}else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
			//
		}else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
			//
		}else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
			//
		}else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
			// フィールド中心へ
			Start(forward);
			DriveForward(rotate, kForwardZeroToMiddleOfField, 50);
			DriveRotate(forward, kTurnJustLeft, 50);
			DriveForward(end, kForwardToMiddle, 50);
			End();
		}else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
			// 全速力Scale
			Start(forward);
			DriveForward(rotate, kForwardZeroToScale, 50);
			DriveRotate(forward,  kTurnLeftToScaleToShoot, 50);
			LiftUp(shoot, Lift.kScaleHigh, 50);
			ArmShoot(end, kShootPower, 50);
			End();
		}else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
			// 戻ってきてSwitch
			Start(forward);
			DriveForward(rotate, kForwardZeroToOverSwitch, 50);
			DriveRotate(forward, kTurnLeftToReturnToSwitchAngle, 50);
			DriveForward(up, kForwardReturnToSwitchAndShoot, 50);
			LiftUp(shoot, Lift.kSwitchHigh, 50);
			ArmShoot(end, kShootPower, 50);
			End();
		}else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
			// 全速力Scale or 戻っってきてSwitch
		}
	}

	void Start(String next) {
		if(status == null) {
			status = next;
		}
	}

	void DriveForward(String next, double setpoint, double delayms) {
		if (status == "forward") {
			if (drive.driveSpeed_pidController.isEnabled() == false) {
				if (cnt_forward == 1) {
					drive.driveRightEncoder.reset();
					drive.runSpeedPID(setpoint);
				}else if (cnt_forward == 2) {
					drive.driveRightEncoder.reset();
					drive.runSpeedPID(setpoint);
				}else if (cnt_forward == 3) {
					drive.driveRightEncoder.reset();
					drive.runSpeedPID(setpoint);
				}
			}else if (drive.driveSpeed_pidController.onTarget()) {
				drive.stopSpeedPID();
				Timer.delay(delayms);
				status = next;
				cnt_forward++;
			}
		}
	}

	void DriveRotate(String next, double setpoint, double delayms) {
		if (status == "rotate") {
			if (drive.driveRotation_pidController.isEnabled() == false) {
				if (cnt_rotate == 1) {
					drive.runRotationPID(setpoint);
				}else if (cnt_rotate == 2) {
					drive.runRotationPID(setpoint);
				}else if (cnt_rotate == 3) {
					drive.runRotationPID(setpoint);
				}
			}else if (drive.driveRotation_pidController.onTarget()) {
				drive.stopRotationPID();
				Timer.delay(delayms);
				status = next;
				cnt_rotate++;
			}
		}
	}

	void LiftUp(String next, double setpoint, double delayms) {
		if (status == "up") {
			if (lift.lift_pidController.isEnabled() == false) {
				lift.runPID(setpoint);
			}else if (lift.lift_pidController.onTarget()) {
				lift.stopPID();
				Timer.delay(delayms);
				status = next;
				cnt_up++;
			}
		}
	}

	void ArmShoot(String next, double setpoint, double delayms) {
		if (status == "shoot") {
			timer.reset();
			timer.start();
			 if (timer.get() < 1) {
				 arm.my_arms.set(setpoint);
			 }else {
				 Timer.delay(delayms);
				 status = next;
				 cnt_shoot++;
			 }
		}
	}

	void End() {
		if (status == "end") {
			//Do nothing
		}
	}
}