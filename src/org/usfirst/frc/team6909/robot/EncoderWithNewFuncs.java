package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderWithNewFuncs extends Encoder {

	private double aOHFE1;
	private double E2OHFG;
	private double E2L;
	private double aHoI;
	private double strL;
	private double strLLoss;
	private double Const;

	private double armsCurrentHeightFromGround;
	private double armsCurrentSpeed;

	public EncoderWithNewFuncs(int channelA, int channelB, boolean reverseDirection, double armsOriginalHeightFromE1, double E2OriginalHeightFromGround, double E2LengthMM, double armsHeightOfItselfMM, double stringLengthMM,  double stringLengthLossMM){
		super(channelA, channelB, reverseDirection);
		this.aOHFE1 = armsOriginalHeightFromE1;
		this.E2OHFG = E2OriginalHeightFromGround;
		this.E2L = E2LengthMM;
		this.aHoI = armsHeightOfItselfMM;
		this.strL = stringLengthMM;
		this.strLLoss = stringLengthLossMM;
		this.Const = aOHFE1 + E2OHFG + E2L + strLLoss - strL - aHoI;
	}
	public double getArmsHeight(){
		armsCurrentHeightFromGround = (2 * this.getDistance()) + Const;
		return armsCurrentHeightFromGround;
	}

	public double getArmsSpeed() {
		armsCurrentSpeed = (2 * this.getRate());
		return armsCurrentSpeed;
	}

	@Override
	 public double pidGet() {
		    switch (super.getPIDSourceType()) {
		      case kDisplacement:
		        return getArmsHeight();
		      case kRate:
		        return getArmsSpeed();
		      default:
		        return 0.0;
		    }
		  }

}

