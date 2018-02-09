package org.usfirst.frc.team6909.robot;

/* ToDo
 * ・距離および角度等の定数の値調整
 * ・liftのPID試行
 *
 *
 */

import edu.wpi.first.wpilibj.Timer;

public class AutonomousChooser {

	Drive drive;
	Lift lift;
	Arm arm;
	Timer timer;

	String gameData;
	int location;

	int phase;

	//Turnのために壁から少し移動
	static final int kForwardBitToTurn = 500;
	//全速力Scale
	static final int kForwardZeroToScale = 7000; //値は仮
	static final int kTurnRightToScaleToShoot = 90;
	static final int kTurnLeftToScaleToShoot = -90;
	//回り込みScale
	static final int kTurnRightToOuterSwitchAngle = 55;
	static final int kTurnLeftToOuterSwitchAngle = -55;
	static final int kForwardZeroToOuterSwitch = 3000;
	static final int kTurnRightToScaleAngle = 45;
	static final int kTurnLeftToScaleAngle = -45;
	static final int kForwardOuterToScaleAndShoot = 4000;
	//全力Switch
	static final int kTurnRightToInnerSwitchAngle = 45;
	static final int kTurnLeftToInnerSwitchAngle = -45;
	static final int kForwardZeroToInnerSwitch = 2000;
	static final int kTurnRightToSwitchToShoot = 90;
	static final int kTurnLeftToSwitchToShoot = -90;
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
	static double DriveSpeedTolerance = 1.0;
	static double DriveRotationTolerane = 1.0;
	static double LiftTolerance = 1.0;

	AutonomousChooser(String gameData, int location, Drive drive, Lift lift, Arm arm) {
		this.gameData = gameData;
		this.location = location;
		this.drive = drive;
		this.lift = lift;
		this.arm = arm;

		timer = new Timer();

		phase = 0;
	}

	void autonomousInit() {

	}

	void autonomousPeriodic() {
		if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
			// (左)全速力Scale (*第二候補 (左)戻ってきてSwitch)
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardZeroToScale, 0.5);
				break;
			case 2:
				DriveRotateAndLiftUp(kTurnRightToScaleToShoot, Lift.kScaleHigh, 0.5);
				break;
			case 3:
				ArmShoot(kShootPower, 0.5);
				break;
			case 4:
				End();
				break;
			}
		}else if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
			// (左)戻ってきてSwitch
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardZeroToOverSwitch, 0.5);
				break;
			case 2:
				DriveRotate(kTurnRightToReturnToSwitchAngle, 0.5);
				break;
			case 3:
				DriveForwardAndLiftUp(kForwardReturnToSwitchAndShoot, Lift.kSwitchHigh, 0.5);
				break;
			case 4:
				ArmShoot(kShootPower, 0.5);
				break;
			case 5:
				End();
				break;
			}
		}else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
			// (左)全速力Scale
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardZeroToScale, 0.5);
				break;
			case 2:
				DriveRotateAndLiftUp(kTurnRightToScaleToShoot, Lift.kScaleHigh, 0.5);
				break;
			case 3:
				ArmShoot(kShootPower, 0.5);
				break;
			case 4:
				End();
				break;
			}
		}else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
			// (左)フィールド中心へ
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardZeroToMiddleOfField, 0.5);
				break;
			case 2:
				DriveRotate(kTurnJustRight, 0.5);
				break;
			case 3:
				DriveForward(kForwardToMiddle, 0.5);
				break;
			case 4:
				End();
				break;
			}
		}else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
			// (左)全力Switch (*第二候補 (左)回り込みScale)
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardBitToTurn, 0.5);
				break;
			case 2:
				DriveRotate(kTurnLeftToInnerSwitchAngle, 0.5);
				break;
			case 3:
				DriveForward(kForwardZeroToInnerSwitch, 0.5);
				break;
			case 4:
				DriveRotateAndLiftUp(kTurnRightToSwitchToShoot, Lift.kSwitchHigh, 0.5);
				break;
			case 5:
				ArmShoot(kShootPower, 0.5);
				break;
			case 6:
				End();
				break;
			}
		}else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
			// (左)全力Switch (*第二候補 (右)回り込みScale)
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardBitToTurn, 0.5);
				break;
			case 2:
				DriveRotate(kTurnLeftToInnerSwitchAngle, 0.5);
				break;
			case 3:
				DriveForward(kForwardZeroToInnerSwitch, 0.5);
				break;
			case 4:
				DriveRotateAndLiftUp(kTurnRightToSwitchToShoot, Lift.kSwitchHigh, 0.5);
				break;
			case 5:
				ArmShoot(kShootPower, 0.5);
				break;
			case 6:
				End();
				break;
			}
		}else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
			// (右)全力Switch (*第二候補 (左)回り込みScale)
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardBitToTurn, 0.5);
				break;
			case 2:
				DriveRotate(kTurnRightToInnerSwitchAngle, 0.5);
				break;
			case 3:
				DriveForward(kForwardZeroToInnerSwitch, 0.5);
				break;
			case 4:
				DriveRotateAndLiftUp(kTurnLeftToSwitchToShoot, Lift.kSwitchHigh, 0.5);
				break;
			case 5:
				ArmShoot(kShootPower, 0.5);
				break;
			case 6:
				End();
				break;
			}
		}else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
			// (右)全力Switch (*第二候補 (右)回り込みScale)
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardBitToTurn, 0.5);
				break;
			case 2:
				DriveRotate(kTurnRightToInnerSwitchAngle, 0.5);
				break;
			case 3:
				DriveForward(kForwardZeroToInnerSwitch, 0.5);
				break;
			case 4:
				DriveRotateAndLiftUp(kTurnLeftToSwitchToShoot, Lift.kSwitchHigh, 0.5);
				break;
			case 5:
				ArmShoot(kShootPower, 0.5);
				break;
			case 6:
				End();
				break;
			}

		}else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
			// (右)フィールド中心へ
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardZeroToMiddleOfField, 0.5);
				break;
			case 2:
				DriveRotate(kTurnJustLeft, 0.5);
				break;
			case 3:
				DriveForward(kForwardToMiddle, 0.5);
				break;
			case 4:
				End();
				break;
			}
		}else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
			// (右)全速力Scale
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardZeroToScale, 0.5);
				break;
			case 2:
				DriveRotateAndLiftUp(kTurnLeftToScaleToShoot, Lift.kScaleHigh, 0.5);
				break;
			case 3:
				ArmShoot(kShootPower, 0.5);
				break;
			case 4:
				End();
				break;
			}
		}else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
			// (右)戻ってきてSwitch
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardZeroToOverSwitch, 0.5);
				break;
			case 2:
				DriveRotate(kTurnLeftToReturnToSwitchAngle, 0.5);
				break;
			case 3:
				DriveForwardAndLiftUp(kForwardReturnToSwitchAndShoot, Lift.kSwitchHigh, 0.5);
				break;
			case 4:
				ArmShoot(kShootPower, 0.5);
				break;
			case 5:
				End();
				break;
			}
		}else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
			// (右)全速力Scale (*第二候補 (右)戻ってきてSwitch)
			switch (phase) {
			case 0:
				Start();
				break;
			case 1:
				DriveForward(kForwardZeroToScale, 0.5);
				break;
			case 2:
				DriveRotateAndLiftUp(kTurnLeftToScaleToShoot, Lift.kScaleHigh, 0.5);
				break;
			case 3:
				ArmShoot(kShootPower, 0.5);
				break;
			case 4:
				End();
				break;
			}
		}
	}

	void Start() {
		if(phase == 0) {
			phase = 1;
		}
	}

	void DriveForward(double setpoint, double delaysec) {
		if (drive.driveSpeed_pidController.isEnabled() == false) {
			//エンコーダーreset
			drive.driveRightEncoder.reset();
			//PID開始
			drive.runSpeedPID(setpoint);
		}

		if (drive.driveSpeed_pidController.onTarget()) {
			drive.stopSpeedPID();
			Timer.delay(delaysec);
			phase++;
		}

	}

	void DriveForwardAndLiftUp(double driveSetpoint, double liftSetpoint, double delaysec) {
		if (drive.driveSpeed_pidController.isEnabled() == false && lift.lift_pidController.isEnabled() == false) {
			//エンコーダーreset
			drive.driveRightEncoder.reset();
			//PID開始
			drive.runSpeedPID(driveSetpoint);
			lift.runPID(liftSetpoint);
		}

		if (drive.driveSpeed_pidController.onTarget() && lift.lift_pidController.onTarget()) {
			drive.stopSpeedPID();
			lift.stopPID();
			phase++;
			Timer.delay(delaysec);
		}
	}

	void DriveRotate(double setpoint, double delaysec) {
		if (drive.driveRotation_pidController.isEnabled() == false) {
			//PID開始
			drive.runRotationPID(setpoint);
		}

		if (drive.driveRotation_pidController.onTarget()) {
			drive.stopRotationPID();
			Timer.delay(delaysec);
			phase++;
		}
	}

	void DriveRotateAndLiftUp(double driveSetpoint, double liftSetpoint, double delaysec) {
		if (drive.driveRotation_pidController.isEnabled() == false && lift.lift_pidController.isEnabled() == false) {
			//PID開始
			drive.runRotationPID(driveSetpoint);
			lift.runPID(liftSetpoint);
		}

		if (drive.driveRotation_pidController.onTarget() && lift.lift_pidController.onTarget()) {
			drive.stopRotationPID();
			lift.stopPID();
			phase++;
			Timer.delay(delaysec);
		}
	}

	void LiftUp(double setpoint, double delaysec) {
		if (lift.lift_pidController.isEnabled() == false) {
			//PID開始
			lift.runPID(setpoint);
		}

		if (lift.lift_pidController.onTarget()) {
			lift.stopPID();
			phase++;
			Timer.delay(delaysec);
		}
	}

	void ArmShoot(double setpoint, double delaysec) {
		timer.reset();
		timer.start();
		if (timer.get() < 1) {
			 arm.my_arms.set(setpoint);
		}else {
			phase++;
			Timer.delay(delaysec);
		 }
	}


	void End() {
		//Do nothing;
	}
}