package org.usfirst.frc.team3925.robot.util;

import edu.wpi.first.wpilibj.Joystick;

public class Rumble {
	public static void setRumble(Joystick joystick, float leftValue, float rightValue) {
		joystick.setRumble(Joystick.RumbleType.kLeftRumble, leftValue);
		joystick.setRumble(Joystick.RumbleType.kRightRumble, rightValue);
	}
}
