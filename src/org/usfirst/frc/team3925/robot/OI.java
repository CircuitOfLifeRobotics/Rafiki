package org.usfirst.frc.team3925.robot;

import static org.usfirst.frc.team3925.robot.RobotMap.JOYSTICK_XBOX_DRIVER;
import static org.usfirst.frc.team3925.robot.RobotMap.JOYSTICK_XBOX_SHOOTER;

import org.usfirst.frc.team3925.robot.util.AwesomeButton;
import org.usfirst.frc.team3925.robot.util.ToggleButton;

import edu.wpi.first.wpilibj.Joystick;

public class OI {
	
	public Joystick driverXbox;
	public Joystick shooterXbox;
	
	public AwesomeButton zeroElevatorBtn;
	public AwesomeButton liftToteBtn;
	public AwesomeButton lowerToteBtn;
	public AwesomeButton stopElevatorBtn;
	public AwesomeButton manualElevatorBtn;
	
	public AwesomeButton rightArmBtn;
	public AwesomeButton leftArmBtn;	
	
	public AwesomeButton turboBtn;
	
	public OI () {
		shooterXbox = new Joystick(JOYSTICK_XBOX_SHOOTER);
    	driverXbox = new Joystick(JOYSTICK_XBOX_DRIVER);
		
		zeroElevatorBtn = new AwesomeButton(shooterXbox, 7, AwesomeButton.ButtonType.PRESS);
    	liftToteBtn = new AwesomeButton(shooterXbox, 4, AwesomeButton.ButtonType.PRESS);
    	lowerToteBtn = new AwesomeButton(shooterXbox, 1, AwesomeButton.ButtonType.PRESS);
    	stopElevatorBtn = new AwesomeButton(shooterXbox, 2, AwesomeButton.ButtonType.PRESS);
    	manualElevatorBtn = new AwesomeButton(shooterXbox, 8, AwesomeButton.ButtonType.RAW);
    	
    	rightArmBtn = new AwesomeButton(driverXbox, 5, AwesomeButton.ButtonType.RAW);
    	leftArmBtn = new AwesomeButton(driverXbox, 6, AwesomeButton.ButtonType.RAW);
    	
    	turboBtn = new AwesomeButton(driverXbox, 10, AwesomeButton.ButtonType.TOGGLE);
	}
	
}
