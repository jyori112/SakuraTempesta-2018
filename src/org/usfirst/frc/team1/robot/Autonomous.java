package org.usfirst.frc.team1.robot;

public class Autonomous {
	AutoMove AutoMove;

	public String gamedata;

	public int location;

	Autonomous(String gamedata0, int location0) {
		gamedata0 = gamedata;
		location0 = location;
	}

	void changerSet() {
		if (gamedata == "L" && location == 1) {
			AutoMove.changerSet(1);
		}
	}

	void move() {
		if (gamedata == "L" && location == 1) {
			AutoMove.changer0(1, 3000, 5);
			AutoMove.changer1(2, 90, 5);
		}
	}

}
