package org.usfirst.frc.team3925.robot.command;

import edu.wpi.first.wpilibj.Timer;

public class Wait implements Command<Object> {
	
	double time;
	Timer timer;
	
	public Wait(double time) {
		this.time = time;
		timer = new Timer();
	}

	@Override
	public void start(Object sub) {
		timer.start();
		timer.reset();
	}

	@Override
	public void execute(Object sub) {
		
	}

	@Override
	public boolean finished(Object sub) {
		return timer.get() >= time;
	}

	@Override
	public void done(Object sub) {
		timer.stop();
	}
	
	
	
}
