package org.usfirst.frc.team3925.robot.subsystem;


public class Intake {
	
	IntakeRollers rollers;
	IntakeArms frontArm, backArm;
	IntakeArmSolenoids intakeArmSolenoids;
	
	public Intake(int rollerport, int frontLeftPort, int frontRightPort, int backLeftPort, int backRightPort, int leftSolenoidPortA, int leftSolenoidPortB, int rightSolenoidPortA, int rightSolenoidPortB) {
		rollers = new IntakeRollers(rollerport);
		frontArm = new IntakeArms(frontLeftPort, frontRightPort);
		backArm = new IntakeArms(backLeftPort, backRightPort);
		intakeArmSolenoids = new IntakeArmSolenoids(leftSolenoidPortA, leftSolenoidPortB, rightSolenoidPortA, rightSolenoidPortB);
	}
	
	public void setRollerSpeed(double speed) {
		rollers.setSpeed(speed);
	}
	
	public void setFrontArmSpeeds(double speed) {
		frontArm.setSpeed(speed);;
	}
	
	public void setBackArmSpeeds(double speed) {
		backArm.setSpeed(speed);
	}
	
	public void setArmSolenoids(boolean left, boolean right) {
		intakeArmSolenoids.setSolenoids(left, right);
	}
	
}
