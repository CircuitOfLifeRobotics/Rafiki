package org.usfirst.frc.team3925.robot.command;

import java.util.Arrays;


public class CommandListExecutor<Subsystem> {
	private Command<Subsystem>[] commandList;
	private int currentCommandIndex;
	private boolean started, done;
	
	@SafeVarargs
	public CommandListExecutor(Command<Subsystem> ... commands) {
		if (commands.length < 1)
			throw new IllegalArgumentException("CommandListExecutor requires at least one command");
		commandList = Arrays.copyOf(commands, commands.length);
		reset();
	}
	
	public void reset() {
		currentCommandIndex = 0;
		started = false;
		done = false;
	}
	
	public void execute(Subsystem sub) {
		if (done) return;
		
		Command<Subsystem> currentCommand = commandList[currentCommandIndex];
		
		if (!started) {
			commandList[currentCommandIndex].start(sub);
			started = true;
		}
		
		currentCommand.execute(sub);
		
		if (currentCommand.finished(sub)) {
			currentCommand.done(sub);
			currentCommandIndex++;
			if (currentCommandIndex == commandList.length) {
				done = true;
			} else {
				started = false;
			}
		}
	}
	
}
