//         .----------------.  .----------------.  .----------------.  .----------------. 
//        | .--------------. || .--------------. || .--------------. || .--------------. |
//        | |    ______    | || |    ______    | || |    _____     | || |   _______    | |
//        | |   / ____ `.  | || |  .' ____ '.  | || |   / ___ `.   | || |  |  _____|   | |
//        | |   `'  __) |  | || |  | (____) |  | || |  |_/___) |   | || |  | |____     | |
//        | |   _  |__ '.  | || |  '_.____. |  | || |   .'____.'   | || |  '_.____''.  | |
//        | |  | \____) |  | || |  | \____| |  | || |  / /____     | || |  | \____) |  | |
//        | |   \______.'  | || |   \______,'  | || |  |_______|   | || |   \______.'  | |
//        | |              | || |              | || |              | || |              | |
//        | '--------------' || '--------------' || '--------------' || '--------------' |
//         '----------------'  '----------------'  '----------------'  '----------------' 
//2015 FRC Robotics competition

package org.usfirst.frc.team3925.robot;

import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_LEFT_ENCODER_A;
import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_LEFT_ENCODER_B;
import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_LEFT_MOTOR;
import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_RIGHT_ENCODER_A;
import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_RIGHT_ENCODER_B;
import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_RIGHT_MOTOR;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_ENCODER_A;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_ENCODER_B;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_LEFT_TALON;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_RIGHT_TALON;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_SWITCH_1;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_SWITCH_2;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_SOLENOID_LEFT_A;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_SOLENOID_LEFT_B;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_SOLENOID_RIGHT_A;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_SOLENOID_RIGHT_B;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_ROLLER;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_UPPER_LEFT;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_UPPER_RIGHT;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_LOWER_LEFT;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_LOWER_RIGHT;
import static org.usfirst.frc.team3925.robot.RobotMap.JOYSTICK_XBOX_DRIVER;
import static org.usfirst.frc.team3925.robot.RobotMap.JOYSTICK_XBOX_SHOOTER;

import org.usfirst.frc.team3925.robot.subsystem.ArmPnuematics;
import org.usfirst.frc.team3925.robot.subsystem.Drive;
import org.usfirst.frc.team3925.robot.subsystem.Elevator;
import org.usfirst.frc.team3925.robot.subsystem.IntakeArms;
import org.usfirst.frc.team3925.robot.subsystem.Rollers;
import org.usfirst.frc.team3925.robot.util.Button;
import org.usfirst.frc.team3925.robot.util.Rumble;
import org.usfirst.frc.team3925.robot.util.ToggleButton;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	int wait = 0;
	
	public static final double AUTONOMOUS_DISTANCE = 50;
	
	Timer timer;
	Drive drive;
	Elevator elevator;
	Rollers rollers;
	IntakeArms upperArms;
	IntakeArms lowerArms;
	Rumble rumble;
	ArmPnuematics arms;
	
	Joystick driverXbox;
	Joystick shooterXbox;
	ToggleButton manualElevatorToggle;
	Button zeroElevatorBtn;
	Button liftToteBtn;
	Button lowerToteBtn;
	Button stopElevatorBtn;
	
	private final double DEADZONE = 0.1;
	double leftDistanceDriven;
	double rightDistanceDriven;
	double leftSpeed;
	double rightSpeed;
	
	private static double GEAR_COEFFICIENT = 0.4;
	
	/*
	 * ShooterXbox:
	 * elevator position 1:		A (1)
	 * elevator position 2:		B (2)
	 * elevator position 3:		X (3)
	 * elevator position 4:		Y (4)
	 * 
	 * tote in initialize:		RT (5)
	 * tote in sequence:		LT (analog 2)
	 * 
	 * tote out initialize:		LT (6)
	 * tote out rollers:		RT (analog 3)
	 */
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	timer = new Timer();
    	drive = new Drive(DRIVE_LEFT_MOTOR, DRIVE_RIGHT_MOTOR, DRIVE_LEFT_ENCODER_A, DRIVE_LEFT_ENCODER_B, DRIVE_RIGHT_ENCODER_A, DRIVE_RIGHT_ENCODER_B);
    	elevator = new Elevator(ELEVATOR_LEFT_TALON, ELEVATOR_RIGHT_TALON, ELEVATOR_ENCODER_A, ELEVATOR_ENCODER_B, ELEVATOR_SWITCH_1, ELEVATOR_SWITCH_2);
    	rollers = new Rollers(INTAKE_ROLLER);
    	upperArms = new IntakeArms(INTAKE_VICTOR_UPPER_LEFT, INTAKE_VICTOR_UPPER_RIGHT);
    	lowerArms = new IntakeArms(INTAKE_VICTOR_LOWER_LEFT, INTAKE_VICTOR_LOWER_RIGHT);
    	arms = new ArmPnuematics(INTAKE_SOLENOID_LEFT_A, INTAKE_SOLENOID_LEFT_B, INTAKE_SOLENOID_RIGHT_A, INTAKE_SOLENOID_RIGHT_B);
    	
    	shooterXbox = new Joystick(JOYSTICK_XBOX_SHOOTER);
    	driverXbox = new Joystick(JOYSTICK_XBOX_DRIVER);
    	manualElevatorToggle = new ToggleButton(shooterXbox, 8 /* start */);
    	zeroElevatorBtn = new Button(shooterXbox, 7);
    	liftToteBtn = new Button(shooterXbox, 4);
    	lowerToteBtn = new Button(shooterXbox, 1);
    	stopElevatorBtn = new Button(shooterXbox, 2);
    	
    	leftDistanceDriven = 0;
    	rightDistanceDriven = 0;
    }

    public void autonomousInit() {
    	//elevator.zeroElevator();
    	drive.resetLeftEncoder();
    	drive.resetRightEncoder();
    	timer.reset();
    	timer.start();
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    	if (timer.get() < 3) {
    		drive.setMotorOutputs(0.75, 0.75);
    	} else {
    		drive.setMotorOutputs(0, 0);
    	}
    	
    	//elevator.elevatorRun();
    }
    
    /**
     * This function is called at the beginning of teleop
     */
    public void teleopInit() {
    	//elevator.zeroElevator();
    	manualElevatorToggle.reset();
    	
    	drive.resetLeftEncoder();
    	drive.resetRightEncoder();
    	
    	elevator.idle();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	drivePeriodic();
		SmartDashboard.putNumber("elevator height", elevator.getCurrentHeight());
    	elevatorPeriodic();
    	intakePeriodic();
    	
    }
    
	/**
	 * Controls elevator based on user input
	 */
	private void elevatorPeriodic() {
		
		manualElevatorToggle.update();
		
		if (!manualElevatorToggle.get()) {
			elevator.idle();
			
			double elevatorSpeed = -shooterXbox.getRawAxis(1);
			elevator.setElevatorSpeed(elevatorSpeed, true);
			SmartDashboard.putNumber("elevator speed", elevatorSpeed);
		} else {
			if (lowerToteBtn.get()) {
				SmartDashboard.putBoolean("lowerTote", true);
				elevator.lowerStack();
			}
			if (liftToteBtn.get()) {
				SmartDashboard.putBoolean("liftTote", true);
				elevator.liftStack();
			}
			if (zeroElevatorBtn.get()) {
				SmartDashboard.putBoolean("zero'd", true);
				elevator.zeroElevator();
			}
			if (stopElevatorBtn.get()) {
				SmartDashboard.putBoolean("stopElevator", true);
				elevator.idle();
			}
			elevator.elevatorRun();
		}
	}
	
	private void intakePeriodic() {
		double rollersSpeed = shooterXbox.getRawAxis(2) - shooterXbox.getRawAxis(3);
		rollers.setSpeed(rollersSpeed);
		/* if (driverXbox.getRawButton(5)) {
			upperArms.setSpeeds(1, -1);
			lowerArms.setSpeeds(1, -1);
		}
		if (driverXbox.getRawButton(6)) {
			upperArms.setSpeeds(1, -1);
			lowerArms.setSpeeds(-1, 1);
		}*/
		double upperIntakeSpeed = (driverXbox.getRawButton(4) ? 0.5:0) - (driverXbox.getRawButton(3) ? 0.5:0);
		double lowerIntakeSpeed = (driverXbox.getRawButton(1) ? 0.5:0) - (driverXbox.getRawButton(2) ? 0.5:0);
		SmartDashboard.putNumber("upper arm speed", upperIntakeSpeed);
		SmartDashboard.putNumber("lower arm speed", lowerIntakeSpeed);
		lowerArms.setSpeeds(lowerIntakeSpeed, -lowerIntakeSpeed);
		upperArms.setSpeeds(upperIntakeSpeed, -upperIntakeSpeed);
		boolean leftArmState = driverXbox.getRawButton(5);
		boolean rightArmState = driverXbox.getRawButton(6);
		arms.setArms(leftArmState, rightArmState);
	}
	
	private void drivePeriodic() {
		double moveValue = driverXbox.getRawAxis(1);
    	double rotateValue = driverXbox.getRawAxis(4);
    	
    	if (Math.abs(moveValue) < DEADZONE)
    		moveValue = 0;
    	if (Math.abs(rotateValue) < DEADZONE)
    		rotateValue = 0;
    	
    	double maxOutput;
    	maxOutput = GEAR_COEFFICIENT;
    	
		drive.drive(moveValue, rotateValue, maxOutput);
	}
    
    @Override
    public void disabledInit() {
    	drive.drive(0, 0, GEAR_COEFFICIENT);
    }
    
    @Override
    public void disabledPeriodic() {
    }
}