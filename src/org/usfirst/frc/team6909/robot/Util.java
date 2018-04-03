package org.usfirst.frc.team6909.robot;

/*
 * コントローラの意図しない入力のブレに対する不感帯を設けた場合でも、出力の範囲を変えないための関数
 */

public class Util {

	static double outputCalc(double kNoReact, double input) {
		if (Math.abs(input) < kNoReact) {
			return 0.0;
		}else {
			return 1 / (1 - kNoReact) * input - input / Math.abs(input) * kNoReact / (1 - kNoReact);
		}
	}

}
