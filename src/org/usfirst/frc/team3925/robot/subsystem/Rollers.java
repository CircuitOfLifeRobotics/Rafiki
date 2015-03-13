package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.Talon;

public class Rollers {
	
	Talon rollers;
	
	public Rollers(int rollerport) {
		rollers = new Talon(rollerport);
	}
	
	public void setSpeed(double speed) {
		rollers.set(speed);
	}
	
}
