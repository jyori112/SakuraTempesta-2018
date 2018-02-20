package org.usfirst.frc.team6909.robot;

/* ToDo
 * ・距離および角度等の定数の値調整
 * ・liftのPID試行
 * ・drive, operationそれぞれの操作マニュアルを作成
 *
 */

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class AutonomousChooser {

	Drive drive;
	Lift lift;
	Arm arm;
	Timer timer;

	String gameData;
	int location;

	int phase;
	int autonomousChooser; // 0:何もしない 1:第一候補 2:第二候補
	boolean isAutonomousModeChosen;
	boolean isAutonomousDone;

	//Turnのために壁から少し移動
	static final int kForwardBitToTurn = 550; //ロボットの対角線の半分の長さ
	//全速力Scale
	static final int kForwardZeroToScale = 8100; //調整中,あのツルツルだとめっちゃdrift
	static final int kTurnRightToScale = 90;
	static final int kTurnLeftToScale = -90;
	static final int kForwardToScaleToShoot = 1000;
	//回り込みScale
	static final int kTurnRightToOuterSwitchAngle = 55; //調整未着手
	static final int kTurnLeftToOuterSwitchAngle = -55;
	static final int kForwardZeroToOuterSwitch = 3000;
	static final int kTurnRightToScaleAngle = 45;
	static final int kTurnLeftToScaleAngle = -45;
	static final int kForwardOuterToScaleAndShoot = 4000;
	//(右)全力Switch
	static final int kTurnRightToInnerSwitchAngle = 31; //調整中
	static final int kForwardZeroToRightInnerSwitch = 4365;
	static final int kTurnLeftToInnerSwitch = -90;
	static final int kForwardToRightShoot = 900;
	//(左)全力Switch
	static final int kTurnLeftToInnerSwitchAngle = -47; //調整中
	static final int kForwardZeroToLeftInnerSwitch = 5330;
	static final int kTurnRightToInnerSwitch = 90;
	static final int kForwardToLeftShoot = 1300;
	//戻ってきてSwitch
	static final int kForwardZeroToOverSwitch = 5000; //調整未着手
	static final int kTurnRightToReturnToSwitchAngle = 150;
	static final int kTurnLeftToReturnToSwitchAngle = -150;
	static final int kForwardReturnToSwitchAndShoot = 2000;
	//通過して待機
	static final int kForwardZeroToMiddleOfField = 6000; //調整未着手
	static final int kTurnJustRight = 90;
	static final int kTurnJustLeft = -90;
	static final int kForwardToMiddle = 1000;
	//armのshootパワー
	static final double kShootPower = 1.0;
	//使用するXboxController
	XboxController xbox_drive;
	XboxController xbox_ope;

	boolean timerStarted;


	AutonomousChooser(String gameData, int location, XboxController xbox_drive, XboxController xbox_ope, Drive drive,
			Lift lift, Arm arm) {
		this.gameData = gameData;
		this.location = location;
		this.xbox_drive = xbox_drive;
		this.xbox_ope = xbox_ope;
		this.drive = drive;
		this.lift = lift;
		this.arm = arm;

		timer = new Timer();
		timerStarted = false;

		phase = 0;
		isAutonomousDone = false;
		autonomousChooser = 999;
	}

	void chooseAutonomusMode() {
		if (isAutonomousModeChosen == false) {
			if (xbox_ope.getAButton() && xbox_ope.getBackButton()) {
				autonomousChooser = 0; //何もしないMode
				isAutonomousModeChosen = true;
			} else if (xbox_ope.getAButton() && xbox_ope.getXButton()) {
				autonomousChooser = 1; //第一候補を選択
				isAutonomousModeChosen = true;
			} else if (xbox_ope.getAButton() && xbox_ope.getYButton()) {
				autonomousChooser = 2; //第二候補を選択
				isAutonomousModeChosen = true;
			}
		}

		if (xbox_ope.getStartButton()) {
			isAutonomousModeChosen = false; //選択やり直し
			autonomousChooser = 999;
		}
	}

	void autonomousInit() {
		isAutonomousDone = false;
		lift.lift_pidController.setOutputRange(Lift.kOutputResistingGravity, 0.7);
		drive.driveRightEncoder.reset();
		drive.driveLeftEncoder.reset();
		drive.gyro.reset();
		lift.liftEncoder.reset();
	}

	void autonomousPeriodic() {
		//未完成なので問題のないところ以外をコメントアウトしてもらいたい
		//あと進む距離(not 角度)はかなり長いので、上に定義した定数[mm]を3とか2で暫定的に割ったほうがいい
		if (isAutonomousDone == false) {
			if (autonomousChooser == 0) {
				//End autonomous
				End();
			}

			//delaysecは0.3から0.5のあたり

			if (isAutonomousModeChosen == true) { //選択なしの場合は第一候補が実行される
				if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (左)全速力Scale *第一候補
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToScale, 0.3);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnRightToScale, Lift.kScaleHigh, 0.5);
							//DriveRotate(kTurnRightToScale, 0.3);
							break;
						case 3:
							DriveForward(kForwardToScaleToShoot, 0.5);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					} else if (autonomousChooser == 2) { // (左)戻ってきてSwitch *第二候補
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToOverSwitch, 0.3);
							break;
						case 2:
							DriveRotate(kTurnRightToReturnToSwitchAngle, 0.3);
							break;
						case 3:
							DriveForward(kForwardReturnToSwitchAndShoot,0.5);
							break;
						case 4:
							ArmShoot(kShootPower, 0.5);
							break;
						case 5:
							End();
							break;
						}
					}

				} else if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {
						switch (phase) {// (左)戻ってきてSwitch *第一候補
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
					}
				} else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (左)全速力Scale *第一候補
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToScale, 0.5);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnRightToScaleAngle, Lift.kScaleHigh, 0.5);
							break;
						case 3:
							ArmShoot(kShootPower, 0.5);
							break;
						case 4:
							End();
							break;
						}
					}
				} else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (左)フィールド中心へ *第一候補
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
					}
				} else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (左)全力Switch *第一候補
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardBitToTurn, 0.3);
							break;
						case 2:
							DriveRotate(kTurnLeftToInnerSwitchAngle, 0.3);
							break;
						case 3:
							DriveForwardAndLiftUp(kForwardZeroToLeftInnerSwitch, Lift.kSwitchHigh, 0.3);
							break;
						case 4:
							DriveRotate(kTurnRightToInnerSwitch, 0.3);
							break;
						case 5:
							DriveForward(kForwardToLeftShoot, 0.3);
							break;
						case 6:
							ArmShoot(kShootPower, 0.3);
							break;
						case 7:
							End();
							break;
						}
					} else if (autonomousChooser == 2) {// (左)回り込みScale *第二候補

					}
				/*} else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (左)全力Switch *第一候補
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
							DriveForward2AndLiftUp(kForwardZeroToLeftInnerSwitch, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							DriveRotate2(kTurnRightToInnerSwitch, 0.5);
							break;
						case 5:
							DriveForward3(kForwardToLeftShoot, 0.5);
							break;
						case 6:
							ArmShoot(kShootPower, 0.5);
							break;
						case 7:
							End();
							break;
						}
					} else if (autonomousChooser == 2) {// (右)回り込みScale *第二候補

					}
				} else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (右)全力Switch *第一候補
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
							DriveForward2AndLiftUp(kForwardZeroToRightInnerSwitch, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							DriveRotate2(kTurnLeftToInnerSwitch, 0.5);
							break;
						case 5:
							DriveForward3(kForwardToRightShoot, 0.5);
							break;
						case 6:
							ArmShoot(kShootPower, 0.5);
							break;
						case 7:
							End();
							break;
						}
					} else if (autonomousChooser == 2) {// (左)回り込みScale *第二候補

					}
				} else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (右)全力Switch *第一候補
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
							DriveForward2AndLiftUp(kForwardZeroToRightInnerSwitch, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							DriveRotate2(kTurnLeftToInnerSwitch, 0.5);
							break;
						case 5:
							DriveForward3(kForwardToRightShoot, 0.5);
							break;
						case 6:
							ArmShoot(kShootPower, 0.5);
							break;
						case 7:
							End();
							break;
						}
					} else if (autonomousChooser == 2) {// (右)回り込みScale *第二候補

					}
				} else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (右)フィールド中心へ *第一候補
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
							DriveForward2(kForwardToMiddle, 0.5);
							break;
						case 4:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (右)全速力Scale *第一候補
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToScale, 0.5);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnLeftToScale, Lift.kScaleHigh, 0.5);
							break;
						case 3:
							ArmShoot(kShootPower, 0.5);
							break;
						case 4:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (右)戻ってきてSwitch *第一候補
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
							DriveForward2AndLiftUp(kForwardReturnToSwitchAndShoot, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							ArmShoot(kShootPower, 0.5);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (右)全速力Scale *第一候補
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToScale, 0.5);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnLeftToScale, Lift.kScaleHigh, 0.5);
							break;
						case 3:
							ArmShoot(kShootPower, 0.5);
							break;
						case 4:
							End();
							break;
						}
					} else if (autonomousChooser == 2) {// (右)戻ってきてSwitch) *第二候補
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
							DriveForward2AndLiftUp(kForwardReturnToSwitchAndShoot, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							ArmShoot(kShootPower, 0.5);
							break;
						case 5:
							End();
							break;
						}
					}
				*/
				}

			} else {// 選択なしの時自動的に第一候補が選択される
				autonomousChooser = 1;
				isAutonomousModeChosen = true;
			}

		}

	}

	void Start() {
		if (phase == 0) {
			phase = 1;
		}
	}

	void DriveForward(double setpoint, double delaysec) {
		drive.runStraightPID(setpoint);

		if(drive.getStraightPIDSetpoint() != 0) {
			if (drive.getStraightPIDOnTarget()) {
				Timer.delay(delaysec);
				drive.stopDrive();
				phase++;
			}
		}
	}



	void DriveForwardAndLiftUp(double driveSetpoint, double liftSetpoint, double delaysec) {
		drive.runStraightPID(driveSetpoint);
		lift.lift_pidController.setSetpoint(liftSetpoint);
		lift.lift_pidController.enable();

		if(drive.getStraightPIDSetpoint() != 0) {
			if (drive.getStraightPIDOnTarget() && lift.lift_pidController.onTarget()) {
				drive.stopDrive();
				Timer.delay(delaysec);
				phase++;
			}
		}
	}

	void DriveRotate(double setpoint, double delaysec) {
		drive.runRotatePID(setpoint);

		if(drive.getRotatePIDSetpoint() != 0) {
			if (drive.getRotatePIDOnTarget()) {
				Timer.delay(delaysec);
				drive.stopDrive();
				phase++;
			}
		}
	}

	void DriveRotateAndLiftUp(double driveSetpoint, double liftSetpoint, double delaysec) {
		drive.runRotatePID(driveSetpoint);
		lift.lift_pidController.setSetpoint(liftSetpoint);
		lift.lift_pidController.enable();

		if(drive.getRotatePIDSetpoint() != 0) {
			if (drive.getRotatePIDOnTarget() && lift.lift_pidController.onTarget()) {
				Timer.delay(delaysec);
				drive.stopDrive();
				phase++;
			}
		}
	}

	void LiftUp(double setpoint, double delaysec) {
		drive.stopDrive();

		if (lift.lift_pidController.isEnabled() == false) {
			//PID開始
			lift.runPID(setpoint);
		}

		if (lift.lift_pidController.onTarget()) {
			Timer.delay(delaysec);
			phase++;

		}
	}

	void ArmShoot(double setpoint, double delaysec) {
		drive.stopDrive();

		if (!timerStarted) {
			timerStarted = true;
			timer.reset();
			timer.start();
		} else {
			if (timer.get() < 3) {
				arm.my_arms.set(setpoint);
			} else {
				Timer.delay(delaysec);
				arm.my_arms.set(0.0);
				phase++;
			}
		}
	}

	void End() {
		isAutonomousDone = true;
	}
}