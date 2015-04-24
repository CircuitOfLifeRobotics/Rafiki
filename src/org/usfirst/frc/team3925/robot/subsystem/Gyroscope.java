package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.Gyro;

public class Gyroscope {
	
	Gyro gyro;
	
	public Gyroscope(int port) {
		gyro = new Gyro(port);
	}
	
	public double getAngle() {
		return gyro.getAngle();
	}
	
	public void reset() {
		gyro.reset();
	}
	
	public void initGyro() {
		gyro.initGyro();
	}
	
}
