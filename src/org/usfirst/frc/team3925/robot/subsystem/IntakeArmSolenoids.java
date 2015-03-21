package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class IntakeArmSolenoids {
	
	DoubleSolenoid leftSolenoid;
	DoubleSolenoid rightSolenoid;
	
	public IntakeArmSolenoids(int leftSolenoidPortA, int leftSolenoidPortB, int rightSolenoidPortA, int rightSolenoidPortB) {
		leftSolenoid = new DoubleSolenoid(leftSolenoidPortA, leftSolenoidPortB);
		rightSolenoid = new DoubleSolenoid(rightSolenoidPortA, rightSolenoidPortB);
	}
	
	public void setSolenoids(boolean leftEngaged, boolean rightEngaged) {
		leftSolenoid.set(leftEngaged ? DoubleSolenoid.Value.kForward:DoubleSolenoid.Value.kReverse);
		rightSolenoid.set(rightEngaged ? DoubleSolenoid.Value.kForward:DoubleSolenoid.Value.kReverse);
	}
	
}
