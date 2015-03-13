package org.usfirst.frc.team3925.robot;

public final class RobotMap {
	// cannot create instances of this class
	private RobotMap() {}
	
	public static final String CAMERA_IP = "10.39.25.11";
	
	public static final int 
			DRIVE_LEFT_MOTOR = 0,
			DRIVE_RIGHT_MOTOR = 1,
			LEFT_DRIVE_ENCODER_A = 0,
			LEFT_DRIVE_ENCODER_B = 1,
			RIGHT_DRIVE_ENCODER_A = 2,
			RIGHT_DRIVE_ENCODER_B = 3,
			ELEVATOR_LEFT_TALON = 2,
			ELEVATOR_RIGHT_TALON = 3,
			ELEVATOR_ENCODER_A = 4,
			ELEVATOR_ENCODER_B = 5,
			ELEVATOR_SWITCH_1 = 9,
			ELEVATOR_SWITCH_2 = 8,
			INTAKE_VICTOR_LEFT = 4,
			INTAKE_VICTOR_RIGHT = 5,
			INTAKE_ROLLER = 6,
			JOYSTICK_XBOX_DRIVER = 0,
			JOYSTICK_XBOX_SHOOTER = 1;
	
}
