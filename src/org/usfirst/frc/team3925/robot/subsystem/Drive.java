package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;

public class Drive {
	
	RobotDrive drive;
	DoubleSolenoid shifter;
	Encoder leftEncoder;
	Encoder rightEncoder;
	
	public Drive(int leftmotorport, int rightmotorport, int pcmCanId, int solenoidportA, int solenoidportB, int leftencoderportA, int leftencoderportB, int rightencoderportA, int rightencoderportB) {
		drive = new RobotDrive(leftmotorport, rightmotorport);
		shifter = new DoubleSolenoid(pcmCanId, solenoidportA, solenoidportB);
		leftEncoder = new Encoder(leftencoderportA, leftencoderportB);
		rightEncoder = new Encoder(rightencoderportA, rightencoderportB);
		leftEncoder.setDistancePerPulse(4*Math.PI / 3d / 128d);
		rightEncoder.setDistancePerPulse(4*Math.PI / 3d / 128d);
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
	
	public double getLeftEncoder() {
		return leftEncoder.getDistance();
	}
	
	public double getRightEncoder() {
		return rightEncoder.getDistance();
	}
	
	public void resetLeftEncoder() {
		leftEncoder.reset();
	}
	
	public void resetRightEncoder() {
		rightEncoder.reset();
	}
}