package org.usfirst.frc.team1.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.XboxController;

public class Robot extends IterativeRobot {

	int kXboxDrivePort = 0;

	int kXboxOpePort = 1;

	XboxController xbox_ope;

	XboxController xbox_drive;

	Arm arm;

	Drive drive;

	Lift lift;

	Autonomous Auto;

	AutoMove AutoMove;

	@Override

	public void robotInit() {

		xbox_drive = new XboxController(kXboxDrivePort);

		xbox_ope = new XboxController(kXboxOpePort);

		drive = new Drive(xbox_drive);

		arm = new Arm(xbox_ope);

		lift = new Lift(xbox_ope);

		AutoMove = new AutoMove(3, 3, 3);

	}

	@Override

	public void autonomousInit() {
		Auto = new Autonomous(DriverStation.getInstance().getGameSpecificMessage(),
				DriverStation.getInstance().getLocation());
		Auto.changerSet();
	}

	@Override

	public void autonomousPeriodic() {
		Auto.move();
	}

	@Override

	public void teleopInit() {

	}

	@Override

	public void teleopPeriodic() {

		drive.teleopPeriodic();

		lift.teleopPeriodic();

		arm.teleopPeriodic();

	}

	@Override

	public void testPeriodic() {

	}

}