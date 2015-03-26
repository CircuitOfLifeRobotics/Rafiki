package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.AnalogInput;

public class ToteRangeFinder {

	AnalogInput channel;
	
	public ToteRangeFinder(int port) {
		channel = new AnalogInput(port);
	}
	
	public boolean get() {
		return channel.getVoltage() > 3;
	}
	
	public double getVolts() {
		return channel.getVoltage();
	}
	
}
