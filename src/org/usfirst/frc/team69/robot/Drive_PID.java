package org.usfirst.frc.team69.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class Drive_PID {
	String DriveAutonomousMode;
	Drive_BASE  drive_base;
	//エンコーダー関連
	private static final int kDriveRightEncoderChannelAPort = 6;
	private static final int kDriveRightEncoderChannelBPort = 7;
	private static final int kDriveLeftEncoderChannelAPort = 8;
	private static final int kDriveLeftEncoderChannelBPort = 9;
	private static final double kDriveEncoderMMPerPulse = 7.7 * Math.PI / 10.71;
	public Encoder driveLeftEncoder;
	public Encoder driveRightEncoder;
	//PID
	public PIDController straightL;
	public double straightL_output;
	public PIDController straightR;
	public double straightR_output;
	public PIDController rotateL;
	public double rotateL_output;
	public PIDController rotateR;
	public double rotateR_output;

	private String drivePIDMode = "stop";

	static final double kStraight_P = 0.012;
	static final double kStraight_I = 0.00;
	static final double kStraight_D = 0.07;
	static final double kRotate_P = 0.035;
	static final double kRotate_I = 0.002;
	static final double kRotate_D = 0.25;

	Drive_PID (String DriveAutonomousMode, Drive_BASE drive_base) {
		this.DriveAutonomousMode = DriveAutonomousMode;
		this.drive_base = drive_base;

		if (DriveAutonomousMode == "SinglEncoder") {
			driveLeftEncoder = new Encoder(kDriveRightEncoderChannelAPort, kDriveRightEncoderChannelBPort, false);
			driveLeftEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);
			driveLeftEncoder.reset();

			drive_base.gyro.reset();

			straightR = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveLeftEncoder, new StraightROutput(drive_base,this)); //pidソースを同じにするが、出力の補正は異なるようにする
			straightL = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveLeftEncoder, new StraightLOutput(drive_base, this));
			rotateR = new PIDController(kRotate_P, kRotate_I,kRotate_D, drive_base.gyro, new RotateROutput(drive_base, this));
			rotateL = new PIDController(kRotate_P, kRotate_I, kRotate_D, drive_base.gyro, new RotateLOutput(drive_base, this));

			straightR.setAbsoluteTolerance(100);
			straightL.setAbsoluteTolerance(100);
			rotateR.setAbsoluteTolerance(10.0);
			rotateL.setAbsoluteTolerance(10.0);
			straightR.setOutputRange(-0.7, 0.7);
			straightL.setOutputRange(-0.7, 0.7);
			rotateR.setOutputRange(-0.6, 0.6);
			rotateL.setOutputRange(-0.6, 0.6);
			straightR.disable();
			straightL.disable();
			rotateR.disable();
			rotateL.disable();
		}else if (DriveAutonomousMode == "DualEncoder") {
			driveRightEncoder = new Encoder(kDriveLeftEncoderChannelAPort, kDriveLeftEncoderChannelBPort, true);
			driveRightEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);
			driveLeftEncoder = new Encoder(kDriveRightEncoderChannelAPort, kDriveRightEncoderChannelBPort, false);
			driveLeftEncoder.setDistancePerPulse(kDriveEncoderMMPerPulse);
			driveRightEncoder.reset();
			driveLeftEncoder.reset();

			drive_base.gyro.reset();

			straightR = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveRightEncoder, new StraightROutput(drive_base,this));
			straightL = new PIDController(kStraight_P, kStraight_I, kStraight_D, driveLeftEncoder, new StraightLOutput(drive_base, this));
			rotateR = new PIDController(kRotate_P, kRotate_I,kRotate_D, drive_base.gyro, new RotateROutput(drive_base, this));
			rotateL = new PIDController(kRotate_P, kRotate_I, kRotate_D, drive_base.gyro, new RotateLOutput(drive_base, this));

			straightR.setAbsoluteTolerance(100);
			straightL.setAbsoluteTolerance(100);
			rotateR.setAbsoluteTolerance(10.0);
			rotateL.setAbsoluteTolerance(10.0);
			straightR.setOutputRange(-0.7, 0.7);
			straightL.setOutputRange(-0.7, 0.7);
			rotateR.setOutputRange(-0.6, 0.6);
			rotateL.setOutputRange(-0.6, 0.6);
			straightR.disable();
			straightL.disable();
			rotateR.disable();
			rotateL.disable();
		}
	}

	public void init() {
		if (DriveAutonomousMode == "SinglEncoder") {
			driveLeftEncoder.reset();
			drive_base.gyro.reset();
		}else if (DriveAutonomousMode == "DualEncoder") {
			driveRightEncoder.reset();
			driveLeftEncoder.reset();
			drive_base.gyro.reset();
		}
	}

	public void setDrivePIDMode(String drivePIDMode) {
		this.drivePIDMode = drivePIDMode;
	}

	public String getDrivePIDMode() {
		return this.drivePIDMode;
	}

	public double getStraightPIDSetpoint() {
		return straightR.getSetpoint(); //RとLは常に同じsetpoint
	}

	public double getRotatePIDSetpoint() {
		return rotateR.getSetpoint(); //RとLは常に同じsetpoint
	}

	public boolean getStraightPIDOnTarget() {
		return straightR.onTarget() && straightL.onTarget();
	}

	public boolean getRotatePIDOnTarget() {
		return rotateL.onTarget() && rotateR.onTarget();
	}

	void excuteOutput() {
		if(drivePIDMode == "stop") {
			drive_base.leftMotors.set(0.0);
			drive_base.rightMotors.set(0.0);
		}else if(drivePIDMode == "straight") {
			drive_base.leftMotors.set(straightL_output);
			drive_base.rightMotors.set(straightR_output);
		}else if(drivePIDMode == "rotate") {
			drive_base.leftMotors.set(rotateL_output);
			drive_base.rightMotors.set(rotateR_output);
		}
	}

	void runRotatePID(double setpoint) {
		if (drivePIDMode != "rotate") {
			drivePIDMode = "rotate";
		}else {
			rotateR.setSetpoint(setpoint);
			rotateL.setSetpoint(setpoint);
			rotateR.enable();
			rotateL.enable();
			excuteOutput();
		}
	}

	void runStraightPID(double setpoint) {

		if (drivePIDMode != "straight") {
			drivePIDMode = "straight";
			if (DriveAutonomousMode == "SinglEncoder") {
				driveLeftEncoder.reset();
			}else if (DriveAutonomousMode == "DualEncoder") {
				driveRightEncoder.reset(); //encoderをリセット
				driveLeftEncoder.reset();
			}
		}else {
			if(drive_base.liftEncoder.getArmsHeight() > 200) {
				straightR.setOutputRange(-0.6, 0.6);
				straightL.setOutputRange(-0.6, 0.6);
				straightR.setSetpoint(setpoint);
				straightL.setSetpoint(setpoint);
				straightR.enable();
				straightL.enable();

				excuteOutput();
			}else {
				straightR.setOutputRange(-0.7, 0.7);
				straightL.setOutputRange(-0.7, 0.7);
				straightR.setSetpoint(setpoint);
				straightL.setSetpoint(setpoint);
				straightR.enable();
				straightL.enable();

				excuteOutput();
			}
		}
	}

	void stopDrive() {
		if (drivePIDMode != "stop") {
			drivePIDMode = "stop";
		}else {
			excuteOutput();
		}
	}
}

class StraightROutput implements PIDOutput {
	Drive_BASE drive_base;
	Drive_PID drive_pid;

	StraightROutput(Drive_BASE drive_base, Drive_PID drive_pid) {
		this.drive_base = drive_base;
		this.drive_pid = drive_pid;
	}

	@Override
	public void pidWrite(double output) {
		drive_pid.straightR_output = - drive_base.rightModifier * output;
	}

}


class StraightLOutput implements PIDOutput {
	Drive_BASE drive_base;
	Drive_PID drive_pid;

	StraightLOutput(Drive_BASE drive_base, Drive_PID drive_pid) {
		this.drive_base = drive_base;
		this.drive_pid = drive_pid;
	}

	@Override
	public void pidWrite(double output) {
		drive_pid.straightL_output = drive_base.leftModifier * output;
	}

}

class RotateROutput implements PIDOutput {
	Drive_BASE drive_base;
	Drive_PID drive_pid;

	RotateROutput(Drive_BASE drive_base, Drive_PID drive_pid) {
		this.drive_base = drive_base;
		this.drive_pid = drive_pid;
	}

	@Override
	public void pidWrite(double output) {
		drive_pid.rotateR_output = drive_base.rightModifier * output; //反転させる マイナスのマイナス
	}

}

class RotateLOutput implements PIDOutput {
	Drive_BASE drive_base;
	Drive_PID drive_pid;

	RotateLOutput(Drive_BASE drive_base, Drive_PID drive_pid) {
		this.drive_base = drive_base;
		this.drive_pid = drive_pid;
	}

	@Override
	public void pidWrite(double output) {
		drive_pid.rotateL_output = drive_base.leftModifier * output;
	}

}