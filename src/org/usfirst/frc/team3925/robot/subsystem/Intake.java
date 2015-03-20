package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;

public class Intake {
	
	Talon rollers;
	Victor frontRightArm;
	Victor frontLeftArm;
	Victor backRightArm;
	Victor backLeftArm;
	
	public Intake(int rollerport, int frontLeftPort, int frontRightPort, int backLeftPort, int backRightPort) {
		rollers = new Talon(rollerport);
		frontLeftArm = new Victor(frontLeftPort);
		frontRightArm = new Victor(frontRightPort);
		backLeftArm = new Victor(backLeftPort);
		backRightArm = new Victor(backRightPort);
	}
	
	public void setRollerSpeed(double speed) {
		rollers.set(speed);
	}
	
	public void setFrontArmSpeeds(double speed) {
		frontLeftArm.set(speed);
		frontRightArm.set(-speed);
	}
	
	public void setBackArmSpeeds(double speed) {
		backLeftArm.set(speed);
		backRightArm.set(-speed);
	}
	
}
