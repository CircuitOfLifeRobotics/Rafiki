package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class SwagDrive {
	
	RobotDrive drive;
	Encoder leftEncoder, rightEncoder;
	Timer timer;
	
	int swagPoints = 0;
	int unloadSwagTime = 0;
	
	public SwagDrive(int leftmotorport, int rightmotorport, int leftencoderportA, int leftencoderportB, int rightencoderportA, int rightencoderportB) {
		drive = new RobotDrive(leftmotorport, rightmotorport);
		leftEncoder = new Encoder(leftencoderportA, leftencoderportB);
		rightEncoder = new Encoder(rightencoderportA, rightencoderportB);
		leftEncoder.setDistancePerPulse(120/9127d);
		rightEncoder.setDistancePerPulse(120/9127d); // determined by pushing the robot 10 feet.
		timer = new Timer();
	}
	
	public void drive(double fwd, double turn, double maxOutput) {
		if (swagPoints < 50) {
			if (maxOutput > 1)
				maxOutput = 1;
			if (maxOutput < 0)
				maxOutput = 0;
			drive.setMaxOutput(maxOutput);
			
			drive.arcadeDrive(fwd, turn);
		}else {
			
		}
	}
	
	public double getRightEncoder() {
		return rightEncoder.getDistance();
	}
	
	public double getLeftEncoder() {
		return leftEncoder.getDistance();
	}
	
	public int getSwagPoints() {
		return swagPoints;
	}
	
}
