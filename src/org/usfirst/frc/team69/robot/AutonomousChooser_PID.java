package org.usfirst.frc.team69.robot;

/*
 * Autonomousは線分と回転角によってなるパス上をロボットが動くように設計。
 * Hawaii Regionalでは、ロボットの試合開始時のLocationを得る仕様が事前の認識と異なっていたこと(手動入力にすることで解決)や、
 * ドライブベース右のエンコーダが亡くなりその結果線分をまっすぐに描けなくなったこと(苦肉の策ながら簡単に変更可能で確実な時間制御で解決)などエラーが重なり、
 * 大会以前に用意したコード通りにロボットが動くことはなかった。
 * 結局最後の一試合になってようやく cubeをswitchにのせる という中難易度のタスクをこなすことに成功した。
 */

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class AutonomousChooser_PID {

	Drive_PID drive_pid;
	Lift lift;
	Arm arm;
	Timer timer;

	public String gameData;
	public int location;

	int phase;
	int autonomousChooser; // 0:線だけ超える 1:第一候補 2:第二候補
	boolean isAutonomousModeChosen;
	boolean isAutonomousDone;

	//距離・角度
	//Turnのために壁から少し移動
	static final int kForwardBitToTurn = 550; //ロボットの対角線の半分の長さ
	//(右)(左)sideから目の前のScale
	static final int kForwardZeroToTurnToScale = 6150;
	 static final int kTurnLeftToScale = -30;
	static final int kTurnRightToScale = 30;
	static final int kForwardToScale = 1000;
	//(右)(左)sideから向こう側のScale
	static final int kForwardZeroToTurnToOtherSide = 5400;
	static final int kTurnLeftToOtherSide = -90;
	static final int kTurnRightToOtherSide = 90;
	static final int kForwardToTurnToOtherSideScale = 5500;
	static final int kTurnRightToOtherSideScale = 15;
	static final int kTurnLeftToOtherSideScale = -15;
	static final int kForwardToOtherSideSclae = 1500;
	//(右)(左)sideからSwitch
	static final int kForwardZeroToTurnToSwitch = 3800;
	static final int kTurnLeftToSwitch = -90;
	static final int kTurnRightToSwitch = 90;
	static final int kForwardToSwitch = 400;
	//sideからcross line <単純時間制御のメソッドで確実に>
	//(右)centerからSwitch
	static final int kTurnRightToRightMidpoint = 55;
	static final int kForwardToRightMidpoint = 940;
	static final int kTurnLeftToRightSwitch = 0;
	static final int kForwardToRightSwitch = 1400;
	//(左)centerからSwitch
	static final int kTurnLeftToLeftMidpoint = -55;
	static final int kForwardToLeftMidpoint = 2300;
	static final int kTurnRightToLeftSwitch = 0;
	static final int kForwardToLeftSwitch = 800;
	//centerからcross line <単純時間制御のメソッドで確実に>


	//使用するXboxController
	XboxController xbox_drive;
	XboxController xbox_ope;

	boolean timerOn;
	boolean patternChosen;
	boolean locationChosen;

	AutonomousChooser_PID(XboxController xbox_ope, Drive_PID drive_pid,
			Lift lift, Arm arm) {
		this.xbox_ope = xbox_ope;
		this.drive_pid = drive_pid;
		this.lift = lift;
		this.arm = arm;

		timer = new Timer();
		timerOn = false;

		phase = 0;
		isAutonomousDone = false;
		isAutonomousModeChosen = false;
		autonomousChooser = 9999;
		location = 9999;
	}

	void chooseAutonomousMode() {
		if (isAutonomousModeChosen == false) {
			if (xbox_ope.getAButton() && xbox_ope.getBButton()) {
				autonomousChooser = 0; //線だけ超えるMode
				patternChosen = true;
			} else if (xbox_ope.getAButton() && xbox_ope.getXButton()) {
				autonomousChooser = 1; //第一候補を選択
				patternChosen = true;
			} else if (xbox_ope.getAButton() && xbox_ope.getYButton()) {
				autonomousChooser = 2; //第二候補を選択
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
			isAutonomousModeChosen = false; //選択やり直し
			patternChosen = false;
			locationChosen = false;
			autonomousChooser = 9999;
			location = 9999;
		}
	}

	void init(String gameData) {
		this.gameData = gameData;

		phase = 0;
		isAutonomousModeChosen = false;
		isAutonomousDone = false;
		timerOn = false;

		lift.lift_pidController.setOutputRange(Lift.kOutputResistingGravity, 0.7);
		lift.liftEncoder.reset();
		drive_pid.init();
		timer.reset();
	}

	void Start() {
		if (phase == 0) {
			phase = 1;
		}
	}

	void GoForward(double setpoint) {
		drive_pid.runStraightPID(setpoint);

		if(drive_pid.getStraightPIDSetpoint() != 0) {
			if (drive_pid.getStraightPIDOnTarget()) {
				Timer.delay(0.2);
				drive_pid.stopDrive();
				phase++;
			}
		}
	}



	void GoForwardAndLiftUp(double driveSetpoint, double liftSetpoint) {
		drive_pid.runStraightPID(driveSetpoint);
		lift.lift_pidController.setSetpoint(liftSetpoint);
		lift.lift_pidController.enable();

		if(drive_pid.getStraightPIDSetpoint() != 0) {
			if (drive_pid.getStraightPIDOnTarget() && lift.lift_pidController.onTarget()) {
				drive_pid.stopDrive();
				Timer.delay(0.2);
				phase++;
			}
		}
	}

	void Rotate(double setpoint) {
		drive_pid.runRotatePID(setpoint);

		if(drive_pid.getRotatePIDSetpoint() != 0) {
			if (drive_pid.getRotatePIDOnTarget()) {
				drive_pid.stopDrive();
				Timer.delay(0.2);
				phase++;
			}
		}
	}

	void RotateAndLiftUp(double rotateSetpoint, double liftSetpoint) {
		drive_pid.runRotatePID(rotateSetpoint);
		lift.lift_pidController.setSetpoint(liftSetpoint);
		lift.lift_pidController.enable();

		if(drive_pid.getRotatePIDSetpoint() != 0) {
			if (drive_pid.getRotatePIDOnTarget() && lift.lift_pidController.onTarget()) {
				drive_pid.stopDrive();
				Timer.delay(0.2);
				phase++;
			}
		}
	}

	void LiftUp(double setpoint, double delaysec) {
		drive_pid.stopDrive();

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
		drive_pid.stopDrive();

		if (timerOn == false) {
			timerOn = true;
			timer.reset();
			timer.start();
		} else {
			if (timer.get() < 3) {
				arm.my_arms.set(setpoint);
			} else {
				Timer.delay(delaysec);
				arm.my_arms.set(0.0);
				timerOn = false;
				phase++;
			}
		}
	}

	void End() {
		isAutonomousDone = true;
	}

	void periodic() {
		if (isAutonomousDone == false) {
			if (autonomousChooser == 0) {
				//Just go aross the line and get 5 points
				if (location == 1 || location == 3) {
					switch (phase) {
					case 0:
						Start();
						break;
					case 1:
						GoForward(5000);
						break;
					case 2:
						End();
						break;
					}
				}else if (location == 2){
					if (gameData.charAt(0) == 'L') {
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardBitToTurn);
							break;
						case 2:
							Rotate(kTurnLeftToLeftMidpoint);
							break;
						case 3:
							GoForward(kForwardToLeftMidpoint);
							break;
						case 4:
							Rotate(kTurnRightToLeftSwitch);
							break;
						case 5:
							GoForward(kForwardToLeftSwitch);
							break;
						case 6:
							End();
							break;
						}
					}else if (gameData.charAt(0) == 'R') {
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardBitToTurn);
							break;
						case 2:
							Rotate(kTurnRightToRightMidpoint);
							break;
						case 3:
							GoForward(kForwardToRightMidpoint);
							break;
						case 4:
							Rotate(kTurnLeftToRightSwitch);
							break;
						case 5:
							GoForward(kForwardToRightSwitch);
							break;
						case 6:
							End();
							break;
						}
					}
				}
			}

			if (isAutonomousModeChosen == true) {
				if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) {  // (左)sideから目の前のscale
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardZeroToTurnToScale);
							break;
						case 2:
							RotateAndLiftUp(kTurnRightToScale, lift.kScaleHigh);
							break;
						case 3:
							GoForward(kForwardToScale);
							break;
						case 4:
							ArmShoot(1.0, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}else if (autonomousChooser == 2) { // (左)sideからswitch
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardToSwitch);
							break;
						case 2:
							RotateAndLiftUp( kTurnRightToSwitch, lift.kSwitchHigh);
							break;
						case 3:
							GoForward( kForwardToSwitch);
							break;
						case 4:
							ArmShoot(1.0, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 1 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) { //(左)sideから向こう側のscale
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardZeroToTurnToOtherSide);
							break;
						case 2:
							Rotate(kTurnRightToOtherSide);
							break;
						case 3:
							GoForward(kForwardToTurnToOtherSideScale);
							break;
						case 4:
							RotateAndLiftUp(kTurnLeftToOtherSideScale, lift.kScaleHigh);
							break;
						case 5:
							GoForward( kForwardToOtherSideSclae);
							break;
						case 6:
							ArmShoot(1.0, 0.3);
							break;
						case 7:
							End();
							break;
						}
					}else if (autonomousChooser == 2) { //(左)sideからswitch
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardToSwitch);
							break;
						case 2:
							RotateAndLiftUp(kTurnRightToSwitch, lift.kSwitchHigh);
							break;
						case 3:
							GoForward(kForwardToSwitch);
							break;
						case 4:
							ArmShoot(1.0, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) { // (左)sideから目の前のscale
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardZeroToTurnToScale);
							break;
						case 2:
							RotateAndLiftUp(kTurnRightToScale, lift.kScaleHigh);
							break;
						case 3:
							GoForward(kForwardToScale);
							break;
						case 4:
							ArmShoot(1.0, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 1 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) { // (左)sideから向こう側のscale
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardZeroToTurnToOtherSide);
							break;
						case 2:
							Rotate(kTurnRightToOtherSide);
							break;
						case 3:
							GoForward(kForwardToTurnToOtherSideScale);
							break;
						case 4:
							RotateAndLiftUp(kTurnLeftToOtherSideScale, lift.kScaleHigh);
							break;
						case 5:
							GoForward(kForwardToOtherSideSclae);
							break;
						case 6:
							ArmShoot(1.0, 0.3);
							break;
						case 7:
							End();
							break;
						}
					}
				} else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) { // (左)centerからswitch
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardBitToTurn);
							break;
						case 2:
							Rotate(kTurnLeftToLeftMidpoint);
							break;
						case 3:
							GoForward(kForwardToLeftMidpoint);
							break;
						case 4:
							RotateAndLiftUp(kTurnRightToLeftSwitch, lift.kSwitchHigh);
							break;
						case 5:
							GoForward(kForwardToLeftSwitch);
							break;
						case 6:
							ArmShoot(1.0, 0.3);
							break;
						case 7:
							End();
							break;
						}
					}
				} else if (location == 2 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) { // (左)centerからswitch
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardBitToTurn);
							break;
						case 2:
							Rotate(kTurnLeftToLeftMidpoint);
							break;
						case 3:
							GoForward(kForwardToLeftMidpoint);
							break;
						case 4:
							RotateAndLiftUp(kTurnRightToLeftSwitch, lift.kSwitchHigh);
							break;
						case 5:
							GoForward(kForwardToLeftSwitch);
							break;
						case 6:
							ArmShoot(1.0, 0.3);
							break;
						case 7:
							End();
							break;
						}
					}
				} else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) { // (右)centerからswitch
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardBitToTurn);
							break;
						case 2:
							Rotate(kTurnRightToRightMidpoint);
							break;
						case 3:
							GoForward(kForwardToRightMidpoint);
							break;
						case 4:
							RotateAndLiftUp(kTurnLeftToRightSwitch, lift.kSwitchHigh);
							break;
						case 5:
							GoForward(kForwardToRightSwitch);
							break;
						case 6:
							ArmShoot(1.0, 0.3);
							break;
						case 7:
							End();
							break;
						}
					}
				} else if (location == 2 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) { // (右)centerからswitch
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardBitToTurn);
							break;
						case 2:
							Rotate(kTurnRightToRightMidpoint);
							break;
						case 3:
							GoForward(kForwardToRightMidpoint);
							break;
						case 4:
							RotateAndLiftUp(kTurnLeftToRightSwitch, lift.kSwitchHigh);
							break;
						case 5:
							GoForward(kForwardToRightSwitch);
							break;
						case 6:
							ArmShoot(1.0, 0.3);
							break;
						case 7:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) { // (右)sideから向こう側のscale
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardZeroToTurnToOtherSide);
							break;
						case 2:
							Rotate(kTurnLeftToOtherSide);
							break;
						case 3:
							GoForward(kForwardToTurnToOtherSideScale);
							break;
						case 4:
							RotateAndLiftUp(kTurnRightToOtherSideScale, lift.kScaleHigh);
							break;
						case 5:
							GoForward(kForwardToOtherSideSclae);
							break;
						case 6:
							ArmShoot(1.0, 0.3);
							break;
						case 7:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'L' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) { // (右)sideから目の前のscale
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardZeroToTurnToScale);
							break;
						case 2:
							RotateAndLiftUp(kTurnLeftToScale, lift.kScaleHigh);
							break;
						case 3:
							GoForward(kForwardToScale);
							break;
						case 4:
							ArmShoot(1.0, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'L') {
					if (autonomousChooser == 1) { //(右)sideから向こう側のscale
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardZeroToTurnToOtherSide);
							break;
						case 2:
							Rotate(kTurnLeftToOtherSide);
							break;
						case 3:
							GoForward(kForwardToTurnToOtherSideScale);
							break;
						case 4:
							RotateAndLiftUp(kTurnRightToOtherSideScale, lift.kScaleHigh);
							break;
						case 5:
							GoForward(kForwardToOtherSideSclae);
							break;
						case 6:
							ArmShoot(1.0, 0.3);
							break;
						case 7:
							End();
							break;
						}
					}else if (autonomousChooser == 2) { //(右)sideからswitch
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardToSwitch);
							break;
						case 2:
							RotateAndLiftUp(kTurnLeftToSwitch, lift.kSwitchHigh);
							break;
						case 3:
							GoForward(kForwardToSwitch);
							break;
						case 4:
							ArmShoot(1.0, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				} else if (location == 3 && gameData.charAt(0) == 'R' && gameData.charAt(1) == 'R') {
					if (autonomousChooser == 1) {  // (右)sideから目の前のscale
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardZeroToTurnToScale);
							break;
						case 2:
							RotateAndLiftUp(kTurnLeftToScale, lift.kScaleHigh);
							break;
						case 3:
							GoForward(kForwardToScale);
							break;
						case 4:
							ArmShoot(1.0, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}else if (autonomousChooser == 2) { // (右)sideからswitch
						switch (phase) {
						case 0:
							Start();
							break;
						case 1:
							GoForward(kForwardToSwitch);
							break;
						case 2:
							RotateAndLiftUp(kTurnLeftToSwitch, lift.kSwitchHigh);
							break;
						case 3:
							GoForward(kForwardToSwitch);
							break;
						case 4:
							ArmShoot(1.0, 0.3);
							break;
						case 5:
							End();
							break;
						}
					}
				}
			}
		}
	}
}