package org.usfirst.frc.team1.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;

public class AutonChooser {

	private String gameData;
	private int location;
	private int status;
	private int changer;
	private static final double DriveDistance1 = 36660;
	private static final double DriveDistance2 = 38660;
	private static final double DistanceFromSwitch = 100;
	private static final double Angle1 = 90;
	private static final double Angle2 = -30;
	private static final double Angle3 = 130;
	private static final double Angle4 = 30;
	private static final double Angle5 = -130;
	private static final double Angle6 = -90;

	Arm arm;
	Drive drive;
	Lift lift;
	EncoderWithNewFuncs liftEncoder;
	Encoder DriveEncoder;



	AutonChooser(XboxController xbox_ope, Arm arm, Drive drive, Lift lift, EncoderWithNewFuncs liftEncoder,String gamedata, int location) {
		this.arm = arm;
		this.lift = lift;
		this.drive = drive;
		this.liftEncoder = liftEncoder;
		this.gameData = gamedata;
		this.location = location;

	}

	void chooser() {

	}
}
