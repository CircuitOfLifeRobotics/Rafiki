package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;

public class Drive {
	
	public static final double WHEELBASE_SIZE = 27; // inches
	
	RobotDrive drive;
	Encoder leftEncoder;
	Encoder rightEncoder;
	
	public Drive(int leftmotorport, int rightmotorport, int leftencoderportA, int leftencoderportB, int rightencoderportA, int rightencoderportB) {
		drive = new RobotDrive(leftmotorport, rightmotorport);
		leftEncoder = new Encoder(leftencoderportA, leftencoderportB);
		rightEncoder = new Encoder(rightencoderportA, rightencoderportB);
		leftEncoder.setDistancePerPulse(120/9127d);
		rightEncoder.setDistancePerPulse(120/9127d); // determined by pushing the robot 10 feet.
	}
	
	public void drive(double fwd, double turn, double maxOutput) {
		if (maxOutput > 1)
			maxOutput = 1;
		if (maxOutput < 0)
			maxOutput = 0;
		drive.setMaxOutput(maxOutput);
		
		drive.arcadeDrive(fwd, turn);
	}
	
	public void setMotorOutputs(double leftSpeed, double rightSpeed) {
		drive.setLeftRightMotorOutputs(leftSpeed, rightSpeed);
	}
	
	public double getLeftDistance() {
		return leftEncoder.getDistance();
	}
	
	public double getRightDistance() {
		return rightEncoder.getDistance();
	}
	
	public void resetLeftEncoder() {
		leftEncoder.reset();
	}
	
	public void resetRightEncoder() {
		rightEncoder.reset();
	}
}
