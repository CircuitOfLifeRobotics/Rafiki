package org.usfirst.frc.team3925.robot.command;

import org.usfirst.frc.team3925.robot.subsystem.Drive;

import edu.wpi.first.wpilibj.Timer;

public class Wait implements Command<Drive> {
	
	Timer timer;
	
	double waitTime;
	
	public Wait(double time) {
		//time in seconds
		waitTime = time;
		timer = new Timer();
	}

	@Override
	public void start(Drive sub) {
		timer.reset();
		timer.start();
	}

	@Override
	public void execute(Drive sub) {
		
	}

	@Override
	public boolean finished(Drive sub) {
		return timer.get() >= waitTime;
	}

	@Override
	public void done(Drive sub) {
		timer.stop();
	}
	
}
