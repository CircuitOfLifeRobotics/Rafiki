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

import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_LEFT_MOTOR;
import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_RIGHT_MOTOR;
import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_SOLENOID_A;
import static org.usfirst.frc.team3925.robot.RobotMap.DRIVE_SOLENOID_B;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_ENCODER_A;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_ENCODER_B;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_LEFT_TALON;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_RIGHT_TALON;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_SWITCH_1;
import static org.usfirst.frc.team3925.robot.RobotMap.ELEVATOR_SWITCH_2;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_ROLLER;
import static org.usfirst.frc.team3925.robot.RobotMap.JOYSTICK_XBOX_DRIVER;
import static org.usfirst.frc.team3925.robot.RobotMap.JOYSTICK_XBOX_SHOOTER;
import static org.usfirst.frc.team3925.robot.RobotMap.LATCH_PORT;
import static org.usfirst.frc.team3925.robot.RobotMap.LEFT_DRIVE_ENCODER_A;
import static org.usfirst.frc.team3925.robot.RobotMap.LEFT_DRIVE_ENCODER_B;
import static org.usfirst.frc.team3925.robot.RobotMap.PCM_CAN_ID;
import static org.usfirst.frc.team3925.robot.RobotMap.RIGHT_DRIVE_ENCODER_A;
import static org.usfirst.frc.team3925.robot.RobotMap.RIGHT_DRIVE_ENCODER_B;

import org.usfirst.frc.team3925.robot.subsystem.Drive;
import org.usfirst.frc.team3925.robot.subsystem.Elevator;
import org.usfirst.frc.team3925.robot.subsystem.Latches;
import org.usfirst.frc.team3925.robot.subsystem.Rollers;
import org.usfirst.frc.team3925.robot.subsystem.Rumble;
import org.usfirst.frc.team3925.robot.util.Button;
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
	Rollers intake;
	Latches latches;
	Rumble rumble;
	
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
	
	private static double LOW_GEAR_COEFFICIENT = 0.4;
	private static double HIGH_GEAR_COEFFICIENT = 1;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	timer = new Timer();
    	drive = new Drive(DRIVE_LEFT_MOTOR, DRIVE_RIGHT_MOTOR, PCM_CAN_ID, DRIVE_SOLENOID_A, DRIVE_SOLENOID_B, LEFT_DRIVE_ENCODER_A, LEFT_DRIVE_ENCODER_B, RIGHT_DRIVE_ENCODER_A, RIGHT_DRIVE_ENCODER_B);
    	latches = new Latches(LATCH_PORT);
    	elevator = new Elevator(ELEVATOR_LEFT_TALON, ELEVATOR_RIGHT_TALON, ELEVATOR_ENCODER_A, ELEVATOR_ENCODER_B, ELEVATOR_SWITCH_1, ELEVATOR_SWITCH_2, latches);
    	intake = new Rollers(INTAKE_ROLLER);
    	
    	shooterXbox = new Joystick(JOYSTICK_XBOX_SHOOTER);
    	driverXbox = new Joystick(JOYSTICK_XBOX_DRIVER);
    	manualElevatorToggle = new ToggleButton(shooterXbox, 8 /* start */);
    	zeroElevatorBtn = new Button(shooterXbox, 2);
    	liftToteBtn = new Button(shooterXbox, 1);
    	lowerToteBtn = new Button(shooterXbox, 4);
    	stopElevatorBtn = new Button(shooterXbox, 3);
    	
    	leftDistanceDriven = 0;
    	rightDistanceDriven = 0;
    }

    public void autonomousInit() {
    	elevator.zeroElevator();
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
    	
    	elevator.elevatorRun();
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
		
		if (manualElevatorToggle.get()) {
			elevator.idle();
			
			double elevatorSpeed = -shooterXbox.getRawAxis(1);
			elevator.setElevatorSpeed(elevatorSpeed, false);
		} else {
			if (lowerToteBtn.get()) {
				elevator.lowerStack();
			}
			if (liftToteBtn.get()) {
				elevator.liftStack();
			}
			if (zeroElevatorBtn.get()) {
				SmartDashboard.putBoolean("zero'd", true);
				elevator.zeroElevator();
			}
			if (stopElevatorBtn.get()) {
				elevator.idle();
			}
			elevator.elevatorRun();
		}
	}
	
	private void intakePeriodic() {
		double intakeSpeed = shooterXbox.getRawAxis(2) - shooterXbox.getRawAxis(3);
		intake.setSpeed(intakeSpeed);
	}
	
	private void drivePeriodic() {
		double moveValue = driverXbox.getRawAxis(1);
    	double rotateValue = driverXbox.getRawAxis(4);
    	
    	if (Math.abs(moveValue) < DEADZONE)
    		moveValue = 0;
    	if (Math.abs(rotateValue) < DEADZONE)
    		rotateValue = 0;
    	
    	double maxOutput;
    	boolean gear1 = driverXbox.getRawButton(5);
    	boolean gear2 = driverXbox.getRawButton(6);
    	
    	if (gear1 || gear2) {
    		maxOutput = HIGH_GEAR_COEFFICIENT;
    	} else {
    		maxOutput = LOW_GEAR_COEFFICIENT;
    	}
    	
		drive.drive(moveValue, rotateValue, maxOutput);
	}
    
    @Override
    public void disabledInit() {
    	drive.drive(0, 0, LOW_GEAR_COEFFICIENT);
    }
    
    @Override
    public void disabledPeriodic() {
    }
}