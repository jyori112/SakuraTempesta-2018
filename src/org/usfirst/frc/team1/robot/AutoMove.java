package org.usfirst.frc.team1.robot;

import edu.wpi.first.wpilibj.Timer;

public class AutoMove {
	Drive drive;

	Lift lift;

	Arm Arm;

	public int changer;

	AutoMove(double percentage1, double percentage2, double percentage3) {
		drive.SpeedTolerance(percentage1);
		drive.RotationTolerance(percentage2);
		lift.setLiftTolerance(percentage3);
	}

	void changerSet(int next) {
		changer = next;
	}

	void changer0(int next, double setpoint, double seconds) {
		if (changer == 0) {
			if (drive.isEnabledSpeed() == false) {
				drive.runSpeedPID(setpoint);
			}
			if (drive.onSpeedTarget()) {
				drive.stopSpeedPID();
				Timer.delay(seconds);
				changer = next;
			}
		}
	}

	void changer1(int next, double setpoint, double seconds) {
		if (changer == 1) {
			if (drive.isEnabledRotation() == false) {
				drive.runRotationPID(setpoint);
			}
			if (drive.onRotationTarget()) {
				drive.stopRotationPID();
				Timer.delay(seconds);
				changer = next;
			}
		}
	}

	void changer2(int next, double setpoint, double seconds) {
		if (changer == 2) {
			if (lift.isEnabled() == false) {
				lift.runPID(setpoint);
			}
			if (drive.onRotationTarget()) {
				lift.stopPID();
				Timer.delay(seconds);
				changer = next;
			}
		}
	}

	void changer3() {
		if (changer == 3) {
			Arm.ArmStartOuting();
		}
	}

}