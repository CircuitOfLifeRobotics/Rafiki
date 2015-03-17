package org.usfirst.frc.team3925.robot.command;

public interface Command<Subsystem> {

	public void start(Subsystem sub);
	public void execute(Subsystem sub);
	public boolean finished(Subsystem sub);
	public void done(Subsystem sub);
	
}
