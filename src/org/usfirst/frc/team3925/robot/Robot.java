/*         .----------------.  .----------------.  .----------------.  .----------------. 
*         | .--------------. || .--------------. || .--------------. || .--------------. |
*         | |    ______    | || |    ______    | || |    _____     | || |   _______    | |
*         | |   / ____ `.  | || |  .' ____ '.  | || |   / ___ `.   | || |  |  _____|   | |
*         | |   `'  __) |  | || |  | (____) |  | || |  |_/___) |   | || |  | |____     | |
*         | |   _  |__ '.  | || |  '_.____. |  | || |   .'____.'   | || |  '_.____''.  | |
*         | |  | \____) |  | || |  | \____| |  | || |  / /____     | || |  | \____) |  | |
*         | |   \______.'  | || |   \______,'  | || |  |_______|   | || |   \______.'  | |
*         | |              | || |              | || |              | || |              | |
*         | '--------------' || '--------------' || '--------------' || '--------------' |
*          '----------------'  '----------------'  '----------------'  '----------------' 
*		  2015 FRC Robotics competition
*/

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
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_ROLLER;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_LEFT_DOUBLE_SOLENOID_A;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_LEFT_DOUBLE_SOLENOID_B;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_RIGHT_DOUBLE_SOLENOID_A;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_RIGHT_DOUBLE_SOLENOID_B;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_LEFT_BACK;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_LEFT_FRONT;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_RIGHT_BACK;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_RIGHT_FRONT;
import static org.usfirst.frc.team3925.robot.RobotMap.JOYSTICK_XBOX_DRIVER;
import static org.usfirst.frc.team3925.robot.RobotMap.JOYSTICK_XBOX_SHOOTER;

import org.usfirst.frc.team3925.robot.command.CommandListExecutor;
import org.usfirst.frc.team3925.robot.command.DriveDistance;
import org.usfirst.frc.team3925.robot.command.TurnDriveEncoder;
import org.usfirst.frc.team3925.robot.subsystem.Drive;
import org.usfirst.frc.team3925.robot.subsystem.Elevator;
import org.usfirst.frc.team3925.robot.subsystem.Intake;
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
	Intake intake;
	Rumble rumble;
	
	CommandListExecutor<Drive> autonomousDriveCommandList;
	
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
	
	private static double LOW_GEAR_COEFFICIENT = 0.5;
	private static double HIGH_GEAR_COEFFICIENT = 1;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	timer = new Timer();
    	drive = new Drive(DRIVE_LEFT_MOTOR, DRIVE_RIGHT_MOTOR, DRIVE_LEFT_ENCODER_A, DRIVE_LEFT_ENCODER_B, DRIVE_RIGHT_ENCODER_A, DRIVE_RIGHT_ENCODER_B);
    	elevator = new Elevator(ELEVATOR_LEFT_TALON, ELEVATOR_RIGHT_TALON, ELEVATOR_ENCODER_A, ELEVATOR_ENCODER_B, ELEVATOR_SWITCH_1, ELEVATOR_SWITCH_2);
    	intake = new Intake(INTAKE_ROLLER, INTAKE_VICTOR_LEFT_FRONT, INTAKE_VICTOR_RIGHT_FRONT, INTAKE_VICTOR_LEFT_BACK, INTAKE_VICTOR_RIGHT_BACK, INTAKE_LEFT_DOUBLE_SOLENOID_A, INTAKE_LEFT_DOUBLE_SOLENOID_B, INTAKE_RIGHT_DOUBLE_SOLENOID_A, INTAKE_RIGHT_DOUBLE_SOLENOID_B);
    	
    	shooterXbox = new Joystick(JOYSTICK_XBOX_SHOOTER);
    	driverXbox = new Joystick(JOYSTICK_XBOX_DRIVER);
    	manualElevatorToggle = new ToggleButton(shooterXbox, 8 /* start */);
    	zeroElevatorBtn = new Button(shooterXbox, 2);
    	liftToteBtn = new Button(shooterXbox, 1);
    	lowerToteBtn = new Button(shooterXbox, 4);
    	stopElevatorBtn = new Button(shooterXbox, 3);
    	
    	autonomousDriveCommandList = new CommandListExecutor<Drive>(
    			new DriveDistance(72, 1),
    			new TurnDriveEncoder(45, 1));
    }

    public void autonomousInit() {
    	//elevator.zeroElevator();
    	autonomousDriveCommandList.reset();
    	timer.reset();
    	timer.start();
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    	autonomousDriveCommandList.execute(drive);
    	
    	//elevator.elevatorRun();
    }
    
    /**
     * This function is called at the beginning of teleop
     */
    public void teleopInit() {
    	//elevator.zeroElevator();
    	//manualElevatorToggle.reset();
    	
    	drive.resetLeftEncoder();
    	drive.resetRightEncoder();
    	
    	//elevator.idle();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	
    	drivePeriodic();
		SmartDashboard.putNumber("elevator height", elevator.getCurrentHeight());
    	//elevatorPeriodic();
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
		double rollerSpeed = shooterXbox.getRawAxis(2) - shooterXbox.getRawAxis(3);
		intake.setRollerSpeed(rollerSpeed);
		
		double frontArmSpeed = shooterXbox.getRawAxis(1);
		intake.setFrontArmSpeeds(frontArmSpeed);
		
		double backArmSpeed = shooterXbox.getRawAxis(5);
		intake.setBackArmSpeeds(backArmSpeed);
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
		
		SmartDashboard.putNumber("left drive encoder", drive.getLeftDistance());
		SmartDashboard.putNumber("right drive encoder", drive.getRightDistance());
	}
    
    @Override
    public void disabledInit() {
    	drive.drive(0, 0, LOW_GEAR_COEFFICIENT);
    }
    
    @Override
    public void disabledPeriodic() {
    }
}