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

	public String gameData;
	public int location;

	int phase;
	int autonomousChooser; // 0:何もしない 1:第一候補 2:第二候補
	boolean isAutonomousModeChosen;
	boolean isAutonomousDone;

	//Turnのために壁から少し移動
	static final int kForwardBitToTurn = 550; //ロボットの対角線の半分の長さ
	//(右)(左)全速力Scaler
	static final int kForwardZeroToScale = 8180; //Done
	static final int kTurnRightToScale = 90;
	static final int kTurnLeftToScale = -90;
	static final int kForwardToScaleToShoot = 1000;
	//(右)回り込みScale
	static final int kTurnRightToOuterSwitchAngle = 31; //Done
	static final int kForwardZeroToRightOuterSwitch = 4365;
	static final int kTurnLeftToScaleAngle = -14;
	static final int kForwardOuterToRightScaleAndLift = 3300;
	static final int kForwardBitToRightScaleAndShoot = 110;
	//(左)回り込みScale
	static final int kTurnLeftToOuterSwitchAngle = -47; //Done
	static final int kForwardZeroToOuterSwitch = 5330;
	static final int kTurnRightToScaleAngle = 20;
	static final int kForwardOuterToLeftScaleAndLift = 3500;
	static final int kForwardBitToLeftScaleAndShoot = 100;
	//(右)全力Switch
	static final int kTurnRightToInnerSwitchAngle = 31; //Done
	static final int kForwardZeroToRightInnerSwitch = 4365;
	static final int kTurnLeftToInnerSwitch = -90;
	static final int kForwardToRightShoot = 900;
	//(左)全力Switch
	static final int kTurnLeftToInnerSwitchAngle = -47; //Done
	static final int kForwardZeroToLeftInnerSwitch = 5330;
	static final int kTurnRightToInnerSwitch = 90;
	static final int kForwardToLeftShoot = 1300;
	//(右)(左)戻ってきてSwitch
	static final int kForwardZeroToOverSwitch = 5730; //Done // 一次的に語で割る
	static final int kTurnRightToReturnToSwitchAngle = 130;
	static final int kTurnLeftToReturnToSwitchAngle = -130;
	static final int kForwardReturnToSwitchAndShoot = 1200;
	//(右)(左)通過して待機
	static final int kForwardZeroToMiddleOfField = 5800; //Done
	static final int kTurnJustRight = 90;
	static final int kTurnJustLeft = -90;
	static final int kForwardToMiddle = 2000;
	//armのshootパワー
	static final double kShootPower = 1.0;
	//使用するXboxController
	XboxController xbox_drive;
	XboxController xbox_ope;

	boolean timerStarted;
	boolean patternChosen;
	boolean locationChosen;

	AutonomousChooser(XboxController xbox_drive, XboxController xbox_ope, Drive drive,
			Lift lift, Arm arm) {
		this.xbox_drive = xbox_drive;
		this.xbox_ope = xbox_ope;
		this.drive = drive;
		this.lift = lift;
		this.arm = arm;

		timer = new Timer();
		timerStarted = false;

		phase = 0;
		isAutonomousDone = false;
		isAutonomousModeChosen = false;
		autonomousChooser = 9999;
		location = 9999;
	}

	void chooseAutonomusMode() {
		if (isAutonomousModeChosen == false) {
			if (xbox_ope.getAButton() && xbox_ope.getBackButton()) {
				autonomousChooser = 0; //何もしないMode
				patternChosen = true;
			} else if (xbox_ope.getAButton() && xbox_ope.getXButton()) {
				autonomousChooser = 1; //第一候補を選択
				patternChosen = true;
			} else if (xbox_ope.getAButton() && xbox_ope.getYButton()) {
				autonomousChooser = 2; //第二候補を選択
				patternChosen = true;
			}else if (xbox_ope.getAButton() && xbox_ope.getBButton()) {
				autonomousChooser = 5;
				patternChosen = true;
			}

			if (xbox_ope.getPOV() == 270) {
				this.location = 1;
				locationChosen = true;
			}else if(xbox_ope.getPOV() == 0) {
				this.location = 2;
				locationChosen = true;
			}else if(xbox_ope.getPOV() == 90) {
				this.location = 3;
				locationChosen = true;
			}

			if (patternChosen && locationChosen) {
				isAutonomousModeChosen = true;
			}
		}

		if (xbox_ope.getStartButton()) {
			isAutonomousModeChosen = false; //選択やり直Si
			patternChosen = false;
			locationChosen = false;
			autonomousChooser = 9999;
			location = 9999;
		}
	}

	void autonomousInit() {
		isAutonomousDone = false;
		lift.lift_pidController.setOutputRange(Lift.kOutputResistingGravity, 0.7);
		drive.driveRightEncoder.reset();
		drive.driveLeftEncoder.reset();
		drive.gyro.reset();
		lift.liftEncoder.reset();
		timer.reset();
	}

	void autonomousPeriodic() {
		if (isAutonomousDone == false) {
			if (autonomousChooser == 0) {
				//End autonomous
				End();
			}

			//delaysecは0.3から0.5のあたり

			if (isAutonomousModeChosen == true) { //選択なしの場合は第一候補が実行される
				if (autonomousChooser == 5) {
					switch (phase) {
					case 0:
						Start();
						break;
					case 1:
						if (location == 2) {
							JustGoForward(0.5, 2.5);
						}else {
							JustGoForward(1.0, 1.3);
						}
						break;
					case 2:
						if (location == 1) {
							if (gameData.charAt(0) =='L') {
								JustRotateAndLiftUp(0.7, 90, Lift.kSwitchHigh, 5.0);
								/*
								DriveRotateAndLiftUp(90, Lift.kSwitchHigh, 0.3);
								*/
								break;
							}else {
								End();
								break;
							}
						}else if(location == 3) {
							if(gameData.charAt(0) == 'R') {
								JustRotateAndLiftUp(0.7, - 90, Lift.kSwitchHigh, 5.0);
								/*
								DriveRotateAndLiftUp(-90, Lift.kSwitchHigh, 0.3);
								*/
								break;
							}else{
								End();
								break;
							}
						}else {
							End();
						}
					case 3:
						JustGoForward(0.6, 0.7);
						break;
					case 4:
						ArmShoot(1.0, 0.3);
						break;
					case 5:
						End();
						break;
					}
				}

				if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (左)戻ってきてSwitch *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToOverSwitch, 0.3);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnRightToReturnToSwitchAngle, Lift.kSwitchHigh, 0.5);
							break;
						case 3:
							DriveForward(kForwardReturnToSwitchAndShoot,0.3);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					} else if (autonomousChooser == 2) { // (左)戻ってきてSwitch *第二候補 <Done>(左)全速力Scale *第二候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToScale, 0.3);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnRightToScale, Lift.kScaleHigh, 0.5);
							break;
						case 3:
							DriveForward(kForwardToScaleToShoot, 0.3);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {
						switch (phase) {// (左)戻ってきてSwitch *第一候補 <Done>
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
							DriveForwardAndLiftUp(kForwardReturnToSwitchAndShoot, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (左)全速力Scale *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToScale, 0.3);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnRightToScale, Lift.kScaleHigh, 0.5);
							break;
						case 3:
							DriveForward(kForwardToScaleToShoot, 0.3);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (左)フィールド中心へ *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToMiddleOfField, 0.3);
							break;
						case 2:
							DriveRotate(kTurnJustRight, 0.3);
							break;
						case 3:
							DriveForward(kForwardToMiddle, 0.3);
							break;
						case 4:
							End();
							break;
						}
					}
				} else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (左)全力Switch *第一候補 <Done>
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
							DriveForward(kForwardZeroToLeftInnerSwitch, 0.3);
							break;
						case 4:
							DriveRotateAndLiftUp(kTurnRightToInnerSwitch, Lift.kSwitchHigh, 0.3);
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
					} else if (autonomousChooser == 2) {// (左)回り込みScale *第二候補 <Done>
						switch(phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardBitToTurn, 0.3);
							break;
						case 2:
							DriveRotate(kTurnLeftToOuterSwitchAngle, 0.3);
							break;
						case 3:
							DriveForward(kForwardZeroToOuterSwitch, 0.3);
							break;
						case 4:
							DriveRotate(kTurnRightToScaleAngle, 0.3);
							break;
						case 5:
							DriveForward(kForwardOuterToLeftScaleAndLift, 0.3);
							break;
						case 6:
							LiftUp(Lift.kScaleHigh, 0.3);
							break;
						case 7:
							DriveForward(kForwardBitToLeftScaleAndShoot, 0.3);
							break;
						case 8:
							ArmShoot(kShootPower, 0.3);
							break;
						case 9:
							End();
							break;
						}
					}
				} else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (左)全力Switch *第一候補 <Done>
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
							DriveForward(kForwardZeroToLeftInnerSwitch, 0.3);
							break;
						case 4:
							DriveRotateAndLiftUp(kTurnRightToInnerSwitch, Lift.kSwitchHigh, 0.3);
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
					} else if (autonomousChooser == 2) {// (右)回り込みScale *第二候補 <Done>
						switch(phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardBitToTurn, 0.3);
							break;
						case 2:
							DriveRotate(kTurnRightToOuterSwitchAngle, 0.3);
							break;
						case 3:
							DriveForward(kForwardZeroToRightOuterSwitch, 0.3);
							break;
						case 4:
							DriveRotate(kTurnLeftToScaleAngle, 0.3);
							break;
						case 5:
							DriveForward(kForwardOuterToRightScaleAndLift, 0.3);
							break;
						case 6:
							LiftUp(Lift.kScaleHigh, 0.3);
							break;
						case 7:
							DriveForward(kForwardBitToRightScaleAndShoot, 0.3);
							break;
						case 8:
							ArmShoot(kShootPower, 0.3);
							break;
						case 9:
							End();
							break;
						}
					}
				} else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (右)全力Switch *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardBitToTurn, 0.3);
							break;
						case 2:
							DriveRotate(kTurnRightToInnerSwitchAngle, 0.3);
							break;
						case 3:
							DriveForward(kForwardZeroToRightInnerSwitch, 0.3);
							break;
						case 4:
							DriveRotateAndLiftUp(kTurnLeftToInnerSwitch, Lift.kSwitchHigh, 0.3);
							break;
						case 5:
							DriveForward(kForwardToRightShoot, 0.3);
							break;
						case 6:
							ArmShoot(kShootPower, 0.3);
							break;
						case 7:
							End();
							break;
						}
					} else if (autonomousChooser == 2) {// (左)回り込みScale *第二候補 <Done>
						switch(phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardBitToTurn, 0.3);
							break;
						case 2:
							DriveRotate(kTurnLeftToOuterSwitchAngle, 0.3);
							break;
						case 3:
							DriveForward(kForwardZeroToOuterSwitch, 0.3);
							break;
						case 4:
							DriveRotate(kTurnRightToScaleAngle, 0.3);
							break;
						case 5:
							DriveForward(kForwardOuterToLeftScaleAndLift, 0.3);
							break;
						case 6:
							LiftUp(Lift.kScaleHigh, 0.3);
							break;
						case 7:
							DriveForward(kForwardBitToLeftScaleAndShoot, 0.3);
							break;
						case 8:
							ArmShoot(kShootPower, 0.3);
							break;
						case 9:
							End();
							break;
						}
					}
				} else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (右)全力Switch *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardBitToTurn, 0.3);
							break;
						case 2:
							DriveRotate(kTurnRightToInnerSwitchAngle, 0.3);
							break;
						case 3:
							DriveForward(kForwardZeroToRightInnerSwitch, 0.3);
							break;
						case 4:
							DriveRotateAndLiftUp(kTurnLeftToInnerSwitch, Lift.kSwitchHigh, 0.3);
							break;
						case 5:
							DriveForward(kForwardToRightShoot, 0.3);
							break;
						case 6:
							ArmShoot(kShootPower, 0.3);
							break;
						case 7:
							End();
							break;
						}
					} else if (autonomousChooser == 2) {// (右)回り込みScale *第二候補 <Done>
						switch(phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardBitToTurn, 0.3);
							break;
						case 2:
							DriveRotate(kTurnRightToOuterSwitchAngle, 0.3);
							break;
						case 3:
							DriveForward(kForwardZeroToRightOuterSwitch, 0.3);
							break;
						case 4:
							DriveRotate(kTurnLeftToScaleAngle, 0.3);
							break;
						case 5:
							DriveForward(kForwardOuterToRightScaleAndLift, 0.3);
							break;
						case 6:
							LiftUp(Lift.kScaleHigh, 0.3);
							break;
						case 7:
							DriveForward(kForwardBitToRightScaleAndShoot, 0.3);
							break;
						case 8:
							ArmShoot(kShootPower, 0.3);
							break;
						case 9:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (右)フィールド中心へ *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToMiddleOfField, 0.3);
							break;
						case 2:
							DriveRotate(kTurnJustLeft, 0.3);
							break;
						case 3:
							DriveForward(kForwardToMiddle, 0.3);
							break;
						case 4:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (右)全速力Scale *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToScale, 0.3);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnLeftToScale, Lift.kScaleHigh, 0.5);
							break;
						case 3:
							DriveForward(kForwardToScaleToShoot, 0.3);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {// (右)戻ってきてSwitch *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToOverSwitch, 0.3);
							break;
						case 2:
							DriveRotate(kTurnLeftToReturnToSwitchAngle, 0.3);
							break;
						case 3:
							DriveForwardAndLiftUp(kForwardReturnToSwitchAndShoot, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {// (右)戻ってきてSwitch) *第一候補 <Done>
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToOverSwitch, 0.3);
							break;
						case 2:
							DriveRotate(kTurnLeftToReturnToSwitchAngle, 0.3);
							break;
						case 3:
							DriveForwardAndLiftUp(kForwardReturnToSwitchAndShoot, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					} else if (autonomousChooser == 2) {// (右)全速力Scale *第二候補 <Done>
						switch(phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToOverSwitch, 0.3);
							break;
						case 2:
							DriveRotate(kTurnLeftToReturnToSwitchAngle, 0.3);
							break;
						case 3:
							DriveForwardAndLiftUp(kForwardReturnToSwitchAndShoot, Lift.kSwitchHigh, 0.5);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							DriveForward(kForwardZeroToScale, 0.3);
							break;
						case 2:
							DriveRotateAndLiftUp(kTurnLeftToScale, Lift.kScaleHigh, 0.5);
							break;
						case 3:
							DriveForward(kForwardToScaleToShoot, 0.3);
							break;
						case 4:
							ArmShoot(kShootPower, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				}
			}else {// 選択なしの時自動的に第一候補が選択される
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

	void JustGoForward(double power, double time) {
		if (!timerStarted) {
			timerStarted = true;
			timer.reset();
			timer.start();
		} else {
			if (timer.get() < time) {
				drive.leftMotors.set(power);
				drive.rightMotors.set(-(power) * 1.1);
			} else {
				drive.my_arcade_drive.arcadeDrive(0.0, 0.0);
				Timer.delay(0.3);
				timerStarted = false;
				phase++;
			}
		}
	}

	void JustRotateAndLiftUp(double power, double rotateSetpoint, double liftSetpoint,double rotateTolerance) {
		if (drive.gyro.getAngle() > rotateSetpoint + rotateTolerance) {
			drive.my_arcade_drive.arcadeDrive(0.0, - power);
			lift.runPID(liftSetpoint);
		}else if (drive.gyro.getAngle() < rotateSetpoint - rotateTolerance) {
			drive.my_arcade_drive.arcadeDrive(0.0, power);
			lift.runPID(liftSetpoint);
		}else {
			lift.stopPID();
			drive.my_arcade_drive.arcadeDrive(0.0, 0.0);
			Timer.delay(0.3);
			phase++;
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
				drive.stopDrive();
				Timer.delay(delaysec);
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
				drive.stopDrive();
				Timer.delay(delaysec);
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
				timerStarted = false;
				phase++;
			}
		}
	}

	void End() {
		isAutonomousDone = true;
	}
}