package org.usfirst.frc.team6909.robot;

import edu.wpi.first.wpilibj.Encoder;

public class Encoder_withF extends Encoder {

	 private double aOHFG;
	 private double sCL;
	 private double aHoI;
	 private double strL;
	 private double strLLoss;
	 private double Const;
	 private double encoderDistance;

	 private double armsCurrentHeightFromGround;

	 public Encoder_withF(final int channelA, final int channelB, final double armsOriginalHeightFromGround, final double secondndColumnLengthMM, final double armsHeightOfItselfMM, final double stringLengthMM,  final double stringLengthLossMM){
		super(channelA, channelB);
		this.aOHFG = armsOriginalHeightFromGround;
		this.sCL = secondndColumnLengthMM;
		this.aHoI = armsHeightOfItselfMM;
		this.strL = stringLengthMM;
		this.strLLoss = stringLengthLossMM;
		this.Const = aOHFG + sCL + strLLoss - strL - aHoI;
	}

	public double getArmsHeight(){
		encoderDistance = this.getDistance();
		armsCurrentHeightFromGround =(2 * encoderDistance) + Const;
		return armsCurrentHeightFromGround;
	}

	@Override
	 public double pidGet() {
		    switch (super.getPIDSourceType()) {
		      case kDisplacement:
		        return getArmsHeight();
		      case kRate:
		        return getRate();
		      default:
		        return 0.0;
		    }
		  }

}