package org.usfirst.frc.team3925.robot.command;

import org.usfirst.frc.team3925.robot.subsystem.Drive;
import org.usfirst.frc.team3925.robot.subsystem.Gyroscope;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnGyro implements Command<Drive> {
	
	public static final double TOLERANCE = 1;
	
	public final double targetAngle;
	private double currentAngle;
	public final double slowDownPoint;
	public final double speed;
	
	public Gyroscope gyro;
	
	public TurnGyro(double spid, double angle, double SDP) {
		targetAngle = angle;
		speed = spid;
		slowDownPoint = SDP;
	}
	
	@Override
	public void start(Drive drive) {
		drive.resetGyro();
		drive.drive(0, speed, 1);
	}

	@Override
	public void execute(Drive drive) {
		currentAngle = drive.getAngle();
		SmartDashboard.putNumber("Distance left", Math.abs(currentAngle-targetAngle));
		if (Math.abs(currentAngle-targetAngle) < slowDownPoint) 
			drive.drive(0, speed*Math.abs(currentAngle-targetAngle)/slowDownPoint, 1);
		else
			drive.drive(0, speed, 1);
	}

	@Override
	public boolean finished(Drive drive) {
		return Math.abs(currentAngle-targetAngle) < TOLERANCE;
	}

	@Override
	public void done(Drive drive) {
		SmartDashboard.putBoolean("YO I DID THE THING! <3", true);
	}

}
