package org.usfirst.frc.team3925.robot.command;

import org.usfirst.frc.team3925.robot.subsystem.Drive;

import edu.wpi.first.wpilibj.Timer;

public class TimedDrive implements Command<Drive> {
	
	final double speed, targetTime;
	Timer timer;
	
	public TimedDrive(double speed, double time) {
		this.speed = speed;
		targetTime = time;
		timer = new Timer();
	}
	
	@Override
	public void start(Drive sub) {
		timer.start();
		timer.reset();
	}

	@Override
	public void execute(Drive sub) {
		sub.setMotorOutputs(speed, speed);
	}

	@Override
	public boolean finished(Drive sub) {
		return timer.get()>=targetTime;
	}

	@Override
	public void done(Drive sub) {
		sub.setMotorOutputs(0, 0);
		timer.stop();
	}
	
}
