package org.usfirst.frc.team3925.robot.command;

import org.usfirst.frc.team3925.robot.subsystem.Drive;

public class DriveDistance implements Command<Drive> {

	public static final double TOLERANCE = 3;

	public final double targetDistance, speed;
	
	private double currentDistance;
	
	public DriveDistance(double distance, double speed) {
		this.targetDistance = distance;
		this.speed = speed;
	}

	@Override
	public void start(Drive drive) {
		drive.resetLeftEncoder();
		drive.resetRightEncoder();
	}

	@Override
	public void execute(Drive drive) {
		currentDistance = getDistance(drive);
		if (currentDistance < targetDistance) {
			drive.setMotorOutputs(speed, speed);
		} else{
			drive.setMotorOutputs(-speed, -speed);
		}
	}

	public double getDistance(Drive drive) {
		return (drive.getLeftDistance() + drive.getRightDistance()) / 2;
	}
	
	@Override
	public boolean finished(Drive sub) {
		return Math.abs(currentDistance - targetDistance) < TOLERANCE;
	}

	@Override
	public void done(Drive drive) {
		drive.setMotorOutputs(0, 0);
	}

}
