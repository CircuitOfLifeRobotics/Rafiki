package org.usfirst.frc.team3925.robot.command;

import org.usfirst.frc.team3925.robot.subsystem.Drive;

public class TurnDriveEncoder implements Command<Drive> {

	public static final double TOLERANCE = 5;
	
	public final double targetAngle;
	public final double speed;
	
	private double currentAngle; // just so that we don't have to calculate twice per execution
	
	public TurnDriveEncoder(double angleDeg, double speed) {
		this.targetAngle = angleDeg;
		this.speed = speed;
	}

	@Override
	public void start(Drive drive) {
		drive.resetLeftEncoder();
		drive.resetRightEncoder();
	}
	
	@Override
	public void execute(Drive drive) {
		currentAngle = getCurrentAngle(drive);
		if (currentAngle < targetAngle) {
			drive.setMotorOutputs(speed, -speed);
		} else {
			drive.setMotorOutputs(-speed, speed);
		}
	}

	private double getCurrentAngle(Drive drive) {
		double angleLeft = Math.toDegrees(drive.getLeftDistance() / (Drive.WHEELBASE_SIZE / 2));
		double angleRight = -Math.toDegrees(drive.getRightDistance() / (Drive.WHEELBASE_SIZE / 2));
		return (angleLeft + angleRight) / 2;
	}
	
	@Override
	public boolean finished(Drive drive) {
		return Math.abs(currentAngle - targetAngle) < TOLERANCE;
	}

	@Override
	public void done(Drive drive) {
		drive.setMotorOutputs(0, 0);
	}
	
}
