package org.usfirst.frc.team3925.robot.util;

import edu.wpi.first.wpilibj.Joystick;

public class AwesomeButton {
	
	public static enum ButtonType {
		RAW, PRESS, TOGGLE
	}
	
	final ButtonType type;
	final Joystick stick;
	final int btn;
	
	private boolean last, toggle;
	
	public AwesomeButton(Joystick stick, int btn, ButtonType type) {
		this.stick = stick;
		this.btn = btn;
		this.type = type;
		reset();
	}
	
	public AwesomeButton(Joystick stick, int btn) {
		this(stick, btn, ButtonType.RAW);
	}
	
	public void reset() {
		last = false;
		toggle = false;
	}
	
	public boolean get() {
		boolean state = stick.getRawButton(btn);
		boolean ret = false;
		
		switch (type) {
		case RAW:
			ret = state;
			break;
		case PRESS:
			ret = state && !last;
			break;
		case TOGGLE:
			if (state && !last)
				toggle = !toggle;
			ret = toggle;
			break;
		}
		
		last = state;
		return ret;
	}

}
