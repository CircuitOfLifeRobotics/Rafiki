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

import static org.usfirst.frc.team3925.robot.RobotMap.*;

import org.usfirst.frc.team3925.robot.subsystem.ArmPnuematics;
import org.usfirst.frc.team3925.robot.subsystem.Drive;
import org.usfirst.frc.team3925.robot.subsystem.Elevator;
import org.usfirst.frc.team3925.robot.subsystem.IntakeArms;
import org.usfirst.frc.team3925.robot.subsystem.Rollers;
import org.usfirst.frc.team3925.robot.subsystem.ToteRangeFinder;
import org.usfirst.frc.team3925.robot.util.AwesomeButton;
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
	ToteRangeFinder toteRangeFinder;
	ToteRangeFinder feederStationToteFinder;
	
	OI oi;
	
	private final double DEADZONE = 0.1;
	double leftDistanceDriven;
	double rightDistanceDriven;
	double leftSpeed;
	double rightSpeed;
	
	private static double GEAR_COEFFICIENT = 0.8;
	
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
    	toteRangeFinder = new ToteRangeFinder(RANGE_FINDER_TOTE);
    	feederStationToteFinder = new ToteRangeFinder(RANGE_FINDER_STATION);
    	
    	oi = new OI();
    	
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
		
		if (oi.manualElevatorBtn.get()) {
			elevator.idle();
			
			double elevatorSpeed = -oi.shooterXbox.getRawAxis(1);
			elevator.setElevatorSpeed(elevatorSpeed, true);
			SmartDashboard.putNumber("elevator speed", elevatorSpeed);
		} else {
			if (oi.lowerToteBtn.get()) {
				SmartDashboard.putBoolean("lowerTote", true);
				elevator.lowerStack();
			}
			if (oi.liftToteBtn.get()) {
				SmartDashboard.putBoolean("liftTote", true);
				elevator.liftStackUpper();
			}
			if (oi.zeroElevatorBtn.get()) {
				SmartDashboard.putBoolean("zero'd", true);
				elevator.zeroElevator();
			}
			if (oi.stopElevatorBtn.get()) {
				SmartDashboard.putBoolean("stopElevator", true);
				elevator.idle();
			}
			elevator.elevatorRun();
		}
	}
	
	private void intakePeriodic() {
		SmartDashboard.putNumber("tote range", toteRangeFinder.getVolts());
		SmartDashboard.putNumber("station range", feederStationToteFinder.getVolts());
		
		/*
		if (manualIntakeToggle.get()) {
			double rollersSpeed = shooterXbox.getRawAxis(2) - shooterXbox.getRawAxis(3);
			rollers.setSpeed(rollersSpeed);
			double upperIntakeSpeed = (driverXbox.getRawButton(4) ? 0.5:0) - (driverXbox.getRawButton(3) ? 0.5:0);
			double lowerIntakeSpeed = (driverXbox.getRawButton(1) ? 0.5:0) - (driverXbox.getRawButton(2) ? 0.5:0);
			SmartDashboard.putNumber("upper arm speed", upperIntakeSpeed);
			SmartDashboard.putNumber("lower arm speed", lowerIntakeSpeed);
			lowerArms.setSpeeds(lowerIntakeSpeed, -lowerIntakeSpeed);
			upperArms.setSpeeds(upperIntakeSpeed, -upperIntakeSpeed);
			boolean armState = driverXbox.getRawButton(5);
			SmartDashboard.putBoolean("armPnuematics", armState);
			arms.setArms(armState, armState);
		}else {
			boolean leftArmState = leftArmBtn.get();
			boolean rightArmState = rightArmBtn.get();
			arms.setArms(leftArmState, rightArmState);
			
			double binSpeed = driverXbox.getRawAxis(5);
			double toteSpeed = driverXbox.getRawAxis(6);
			if (binSpeed != 0) {
				upperArms.setSpeeds(-binSpeed, binSpeed);
				lowerArms.setSpeeds(binSpeed, -binSpeed);
			}else {
				upperArms.setSpeeds(toteSpeed, -toteSpeed);
				lowerArms.setSpeeds(toteSpeed, -toteSpeed);
			}
			
		}
		*/
	}
	
	private void drivePeriodic() {
		double moveValue = oi.driverXbox.getRawAxis(1);
    	double rotateValue = oi.driverXbox.getRawAxis(4);
    	
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