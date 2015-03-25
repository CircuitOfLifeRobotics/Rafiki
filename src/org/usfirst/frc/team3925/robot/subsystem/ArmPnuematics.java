package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ArmPnuematics {
	
	DoubleSolenoid leftArm;
	DoubleSolenoid rightArm;
	
	public ArmPnuematics(int leftA, int leftB, int rightA, int rightB) {
		leftArm = new DoubleSolenoid(leftA, leftB);
		rightArm = new DoubleSolenoid(rightA, rightB);
	}
	
	public void setArms(boolean left, boolean right) {
		leftArm.set(left ? DoubleSolenoid.Value.kForward:DoubleSolenoid.Value.kReverse);
		rightArm.set(right ? DoubleSolenoid.Value.kForward:DoubleSolenoid.Value.kReverse);
	}
	
}
