//THIS WHOLE CLASS WAS RENDERED USELESS WITH NEW DESIGN WITHOUT LATCHES

package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.Relay;

public class Latches {
	
	Relay latches;
	
	public Latches(int latchesport) {
		latches = new Relay(latchesport);
	}
	
	public void setLatches(boolean out) {
		latches.set(out ? Relay.Value.kForward : Relay.Value.kOff);
	}
	
}
