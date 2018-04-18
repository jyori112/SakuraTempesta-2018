package org.usfirst.frc.team69.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

public class AutonomousChooser_TIME {

	//ベースになるDrive機能
	Drive_BASE drive_base;
	//Lift
	Lift lift;
	//Arm
	Arm arm;
	//Timer
	Timer timer;
	boolean timerOn;
	//コントローラー
	XboxController xbox_ope;
	//マッチのセッティング
	public String gameData;
	public int location;
	//Autonomousのセッティング
	int phase;
	int autonomousChooser; // 0:線だけ超える 1:第一候補 2:第二候補
	boolean isAutonomousModeChosen;
	boolean isAutonomousDone;
	boolean patternChosen;
	boolean locationChosen;
	//RobotのMAX速度 [mm/s] (Hawaii Regionalでの動きから測定)
	final int MAX_VEL = 2920;
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

	//回転の許容誤差 [degree]
	int rotateTolerance = 5;


	AutonomousChooser_TIME (XboxController xbox_ope, Drive_BASE drive_base, Lift lift, Arm arm) {
		this.drive_base = drive_base;
		this.lift = lift;
		this.arm = arm;
		this.timer = new Timer();
		this.xbox_ope = xbox_ope;
		this.patternChosen = false;
		this.locationChosen = false;


		phase = 0;
		isAutonomousDone = false;
		isAutonomousModeChosen = false;
		autonomousChooser = 9999;
		location = 9999;
	}

	void init(String gameData) {
		this.gameData = gameData;

		phase = 0;
		isAutonomousModeChosen = false;
		isAutonomousDone = false;
		timerOn = false;

		lift.lift_pidController.setOutputRange(lift.kOutputResistingGravity, 0.7);
		drive_base.gyro.reset();
		lift.liftEncoder.reset();
		timer.reset();
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


	void Start() {
		if (phase == 0) {
			phase = 1;
		}
	}

	void GoForward(double power, double distance) {
		double time = distance / (MAX_VEL * power);

		if (timerOn == false) {
			timerOn = true;
			timer.reset();
			timer.start();
		} else {
			if (timer.get() < time) {
				drive_base.leftMotors.set(power * drive_base.leftModifier);
				drive_base.rightMotors.set(-(power) * drive_base.rightModifier);
			} else {
				drive_base.my_arcade_drive.arcadeDrive(0.0, 0.0);
				Timer.delay(0.2);
				timerOn = false;
				phase++;
			}
		}
	}

	void GoForwardAndLiftUp(double power, double time,  double liftSetpoint) {
		if (timerOn == false) {
			timerOn = true;
			timer.reset();
			timer.start();
		} else {
			if (timer.get() < time) {
				drive_base.leftMotors.set(power * drive_base.leftModifier);
				drive_base.rightMotors.set(-(power) * drive_base.rightModifier);
				lift.runPID(liftSetpoint);
			} else {
				lift.stopPID();
				drive_base.my_arcade_drive.arcadeDrive(0.0, 0.0);
				Timer.delay(0.2);
				timerOn = false;
				phase++;
			}
		}
	}

	void Rotate(double power, double rotateSetpoint) {
		if (drive_base.gyro.getAngle() > rotateSetpoint + rotateTolerance) {
			drive_base.my_arcade_drive.arcadeDrive(0.0, - power);
		}else if (drive_base.gyro.getAngle() < rotateSetpoint - rotateTolerance) {
			drive_base.my_arcade_drive.arcadeDrive(0.0, power);
		}else {
			drive_base.my_arcade_drive.arcadeDrive(0.0, 0.0);
			Timer.delay(0.2);
			phase++;
		}
	}

	void RotateAndLiftUp(double power, double rotateSetpoint, double liftSetpoint) {
		if (drive_base.gyro.getAngle() > rotateSetpoint + rotateTolerance) {
			drive_base.my_arcade_drive.arcadeDrive(0.0, - power);
			lift.runPID(liftSetpoint);
		}else if (drive_base.gyro.getAngle() < rotateSetpoint - rotateTolerance) {
			drive_base.my_arcade_drive.arcadeDrive(0.0, power);
			lift.runPID(liftSetpoint);
		}else {
			lift.stopPID();
			drive_base.my_arcade_drive.arcadeDrive(0.0, 0.0);
			Timer.delay(0.2);
			phase++;
		}
	}

	void LiftUp(double setpoint, double delaysec) {

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
						GoForward(0.5, 5000);
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
							GoForward(0.5, kForwardBitToTurn);
							break;
						case 2:
							Rotate(0.7, kTurnLeftToLeftMidpoint);
							break;
						case 3:
							GoForward(1.0, kForwardToLeftMidpoint);
							break;
						case 4:
							Rotate(0.7, kTurnRightToLeftSwitch);
							break;
						case 5:
							GoForward(0.5, kForwardToLeftSwitch);
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
							GoForward(0.5, kForwardBitToTurn);
							break;
						case 2:
							Rotate(0.7, kTurnRightToRightMidpoint);
							break;
						case 3:
							GoForward(1.0, kForwardToRightMidpoint);
							break;
						case 4:
							Rotate(0.7, kTurnLeftToRightSwitch);
							break;
						case 5:
							GoForward(0.5, kForwardToRightSwitch);
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
							GoForward(1.0, kForwardZeroToTurnToScale);
							break;
						case 2:
							RotateAndLiftUp(0.7, kTurnRightToScale, lift.kScaleHigh);
							break;
						case 3:
							GoForward(1.0, kForwardToScale);
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
							GoForward(1.0, kForwardToSwitch);
							break;
						case 2:
							RotateAndLiftUp(0.7, kTurnRightToSwitch, lift.kSwitchHigh);
							break;
						case 3:
							GoForward(0.5, kForwardToSwitch);
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
							GoForward(1.0, kForwardZeroToTurnToOtherSide);
							break;
						case 2:
							Rotate(0.7, kTurnRightToOtherSide);
							break;
						case 3:
							GoForward(1.0, kForwardToTurnToOtherSideScale);
							break;
						case 4:
							RotateAndLiftUp(0.7, kTurnLeftToOtherSideScale, lift.kScaleHigh);
							break;
						case 5:
							GoForward(0.5, kForwardToOtherSideSclae);
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
							GoForward(1.0, kForwardToSwitch);
							break;
						case 2:
							RotateAndLiftUp(0.7, kTurnRightToSwitch, lift.kSwitchHigh);
							break;
						case 3:
							GoForward(0.5, kForwardToSwitch);
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
							GoForward(1.0, kForwardZeroToTurnToScale);
							break;
						case 2:
							RotateAndLiftUp(0.7, kTurnRightToScale, lift.kScaleHigh);
							break;
						case 3:
							GoForward(1.0, kForwardToScale);
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
							GoForward(1.0, kForwardZeroToTurnToOtherSide);
							break;
						case 2:
							Rotate(0.7, kTurnRightToOtherSide);
							break;
						case 3:
							GoForward(1.0, kForwardToTurnToOtherSideScale);
							break;
						case 4:
							RotateAndLiftUp(0.7, kTurnLeftToOtherSideScale, lift.kScaleHigh);
							break;
						case 5:
							GoForward(0.5, kForwardToOtherSideSclae);
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
							GoForward(0.5, kForwardBitToTurn);
							break;
						case 2:
							Rotate(0.7, kTurnLeftToLeftMidpoint);
							break;
						case 3:
							GoForward(1.0, kForwardToLeftMidpoint);
							break;
						case 4:
							RotateAndLiftUp(0.7, kTurnRightToLeftSwitch, lift.kSwitchHigh);
							break;
						case 5:
							GoForward(0.5, kForwardToLeftSwitch);
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
							GoForward(0.5, kForwardBitToTurn);
							break;
						case 2:
							Rotate(0.7, kTurnLeftToLeftMidpoint);
							break;
						case 3:
							GoForward(1.0, kForwardToLeftMidpoint);
							break;
						case 4:
							RotateAndLiftUp(0.7, kTurnRightToLeftSwitch, lift.kSwitchHigh);
							break;
						case 5:
							GoForward(0.5, kForwardToLeftSwitch);
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
							GoForward(0.5, kForwardBitToTurn);
							break;
						case 2:
							Rotate(0.7, kTurnRightToRightMidpoint);
							break;
						case 3:
							GoForward(1.0, kForwardToRightMidpoint);
							break;
						case 4:
							RotateAndLiftUp(0.7, kTurnLeftToRightSwitch, lift.kSwitchHigh);
							break;
						case 5:
							GoForward(0.5, kForwardToRightSwitch);
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
							GoForward(0.5, kForwardBitToTurn);
							break;
						case 2:
							Rotate(0.7, kTurnRightToRightMidpoint);
							break;
						case 3:
							GoForward(1.0, kForwardToRightMidpoint);
							break;
						case 4:
							RotateAndLiftUp(0.7, kTurnLeftToRightSwitch, lift.kSwitchHigh);
							break;
						case 5:
							GoForward(0.5, kForwardToRightSwitch);
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
							GoForward(1.0, kForwardZeroToTurnToOtherSide);
							break;
						case 2:
							Rotate(0.7, kTurnLeftToOtherSide);
							break;
						case 3:
							GoForward(1.0, kForwardToTurnToOtherSideScale);
							break;
						case 4:
							RotateAndLiftUp(0.7, kTurnRightToOtherSideScale, lift.kScaleHigh);
							break;
						case 5:
							GoForward(0.5, kForwardToOtherSideSclae);
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
							GoForward(1.0, kForwardZeroToTurnToScale);
							break;
						case 2:
							RotateAndLiftUp(0.7, kTurnLeftToScale, lift.kScaleHigh);
							break;
						case 3:
							GoForward(1.0, kForwardToScale);
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
							GoForward(1.0, kForwardZeroToTurnToOtherSide);
							break;
						case 2:
							Rotate(0.7, kTurnLeftToOtherSide);
							break;
						case 3:
							GoForward(1.0, kForwardToTurnToOtherSideScale);
							break;
						case 4:
							RotateAndLiftUp(0.7, kTurnRightToOtherSideScale, lift.kScaleHigh);
							break;
						case 5:
							GoForward(0.5, kForwardToOtherSideSclae);
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
							GoForward(1.0, kForwardToSwitch);
							break;
						case 2:
							RotateAndLiftUp(0.7, kTurnLeftToSwitch, lift.kSwitchHigh);
							break;
						case 3:
							GoForward(0.5, kForwardToSwitch);
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
							GoForward(1.0, kForwardZeroToTurnToScale);
							break;
						case 2:
							RotateAndLiftUp(0.7, kTurnLeftToScale, lift.kScaleHigh);
							break;
						case 3:
							GoForward(1.0, kForwardToScale);
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
							GoForward(1.0, kForwardToSwitch);
							break;
						case 2:
							RotateAndLiftUp(0.7, kTurnLeftToSwitch, lift.kSwitchHigh);
							break;
						case 3:
							GoForward(0.5, kForwardToSwitch);
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
