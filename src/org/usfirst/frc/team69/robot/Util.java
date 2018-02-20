package org.usfirst.frc.team69.robot;

public class Util {

	static double outputCalc(double kNoReact, double input) {
		if (Math.abs(input) < kNoReact) {
			return 0.0;
		}else {
			return 1 / (1 - kNoReact) * input - input / Math.abs(input) * kNoReact / (1 - kNoReact);
		}
	}

}
