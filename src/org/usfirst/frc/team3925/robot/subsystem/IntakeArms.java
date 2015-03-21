package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.Victor;

public class IntakeArms {
	
	Victor rightArm;
	Victor leftArm;
	
	public IntakeArms(int leftPort, int rightPort) {
		leftArm = new Victor(leftPort);
		rightArm = new Victor(rightPort);
	}
	
	public void setSpeed(double speed) {
		leftArm.set(speed);
		rightArm.set(-speed);
	}
}
