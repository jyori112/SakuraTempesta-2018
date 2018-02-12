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

	public EncoderWithNewFuncs(final int channelA, final int channelB, final double armsOriginalHeightFromE1, final double E2OriginalHeightFromGround,final double E2LengthMM, final double armsHeightOfItselfMM, final double stringLengthMM,  final double stringLengthLossMM){
		super(channelA, channelB);
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

