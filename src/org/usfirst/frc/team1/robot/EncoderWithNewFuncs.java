package org.usfirst.frc.team1.robot;

import edu.wpi.first.wpilibj.Encoder;

public class EncoderWithNewFuncs extends Encoder {

	private double aOHFG;
	private double sCL;
	private double aHoI;
	private double strL;
	private double strLLoss;
	private double Const;

	private double armsCurrentHeightFromGround;
	private double armsCurrentSpeed;

	public EncoderWithNewFuncs(final int channelA, final int channelB, final double armsOriginalHeightFromGround, final double secondndColumnLengthMM, final double armsHeightOfItselfMM, final double stringLengthMM,  final double stringLengthLossMM){
		super(channelA, channelB);
		this.aOHFG = armsOriginalHeightFromGround;
		this.sCL = secondndColumnLengthMM;
		this.aHoI = armsHeightOfItselfMM;
		this.strL = stringLengthMM;
		this.strLLoss = stringLengthLossMM;
		this.Const = aOHFG + sCL + strLLoss - strL - aHoI;
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

