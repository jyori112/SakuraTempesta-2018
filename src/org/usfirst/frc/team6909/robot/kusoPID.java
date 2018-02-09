package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.Timer;

public class kusoPID {
	Drive drive;
	Lift lift;
	Arm arm;

	String gameData;
	int location;

	int phase;

	kusoPID(String gameData, int location, Drive drive, Lift lift, Arm arm) {
		this.gameData = gameData;
		this.location = location;

		this.drive = drive;
		this.lift = lift;
		this.arm = arm;

		drive.driveSpeed_pidController.setEnabled(false);
		drive.driveSpeed_pidController.setPercentTolerance(1);
		drive.driveRotation_pidController.setEnabled(false);
		drive.driveRotation_pidController.setPercentTolerance(1);

		phase = 0;
	}

	void letsGoPID() {
		switch (phase) {
		case 0:
			trySpeedPID(7000, 3);
			break;
		case 1:
			tryRotatePID(90, 3);
			break;
		}
	}

	void trySpeedPID(double setpoint, double delaysec) {
		if (drive.driveSpeed_pidController.isEnabled() == false) {
			drive.driveLeftEncoder.reset();
			drive.driveSpeed_pidController.setInputRange(0, setpoint);
			drive.runSpeedPID(setpoint);
		}

		if (drive.driveSpeed_pidController.onTarget()) {
			drive.driveSpeed_pidController.disable();
			Timer.delay(delaysec);
			phase++;
		}
	}

	void tryRotatePID(double setpoint, double delaysec) {
		if (drive.driveRotation_pidController.isEnabled() == false) {
			drive.driveRotation_pidController.setInputRange(0, setpoint);
			drive.runRotationPID(setpoint);
		}

		if (drive.driveRotation_pidController.onTarget()) {
			drive.driveRotation_pidController.disable();
			Timer.delay(delaysec);
			phase++;
		}
	}

}
