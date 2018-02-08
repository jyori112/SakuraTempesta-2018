package org.usfirst.frc.team1.robot;

import edu.wpi.first.wpilibj.Timer;

public class Autonomous {
	Drive drive;

	public int changer;

	Autonomous(double percentage1, double percentage2) {
		drive.SpeedTolerance(percentage1);
		drive.RotationTolerance(percentage2);
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
}
