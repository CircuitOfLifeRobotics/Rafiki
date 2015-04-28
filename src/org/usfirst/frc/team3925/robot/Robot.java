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
import static org.usfirst.frc.team3925.robot.RobotMap.GYRO;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_ROLLER;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_SOLENOID_LEFT_A;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_SOLENOID_LEFT_B;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_SOLENOID_RIGHT_A;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_SOLENOID_RIGHT_B;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_LOWER_LEFT;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_LOWER_RIGHT;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_UPPER_LEFT;
import static org.usfirst.frc.team3925.robot.RobotMap.INTAKE_VICTOR_UPPER_RIGHT;
import static org.usfirst.frc.team3925.robot.RobotMap.RANGE_FINDER_STATION;
import static org.usfirst.frc.team3925.robot.RobotMap.RANGE_FINDER_TOTE;

import org.usfirst.frc.team3925.robot.command.CommandListExecutor;
import org.usfirst.frc.team3925.robot.command.DriveDistance;
import org.usfirst.frc.team3925.robot.command.TurnGyro;
import org.usfirst.frc.team3925.robot.subsystem.ArmPnuematics;
import org.usfirst.frc.team3925.robot.subsystem.Drive;
import org.usfirst.frc.team3925.robot.subsystem.Elevator;
import org.usfirst.frc.team3925.robot.subsystem.Gyroscope;
import org.usfirst.frc.team3925.robot.subsystem.IntakeArms;
import org.usfirst.frc.team3925.robot.subsystem.Rollers;
import org.usfirst.frc.team3925.robot.subsystem.ToteRangeFinder;
import org.usfirst.frc.team3925.robot.util.Rumble;

import edu.wpi.first.wpilibj.IterativeRobot;
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
	
	DriveDistance forwardAutonomous;
	TurnGyro turn;
	
	CommandListExecutor<Drive> autonomousDrive;
	
	OI oi;
	
	public static final double AUTONOMOUS_DISTANCE = 50;
	
	private final double DEADZONE = 0.1;
	
	private static double LOW_GEAR_COEFFICIENT = 0.45;
	private static double HIGH_GEAR_COEFFICIENT = 1;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	
    	timer = new Timer();
    	drive = new Drive(DRIVE_LEFT_MOTOR, DRIVE_RIGHT_MOTOR, DRIVE_LEFT_ENCODER_A, DRIVE_LEFT_ENCODER_B, DRIVE_RIGHT_ENCODER_A, DRIVE_RIGHT_ENCODER_B, GYRO);
    	elevator = new Elevator(ELEVATOR_LEFT_TALON, ELEVATOR_RIGHT_TALON, ELEVATOR_ENCODER_A, ELEVATOR_ENCODER_B, ELEVATOR_SWITCH_1, ELEVATOR_SWITCH_2);
    	rollers = new Rollers(INTAKE_ROLLER);
    	upperArms = new IntakeArms(INTAKE_VICTOR_UPPER_LEFT, INTAKE_VICTOR_UPPER_RIGHT);
    	lowerArms = new IntakeArms(INTAKE_VICTOR_LOWER_LEFT, INTAKE_VICTOR_LOWER_RIGHT);
    	arms = new ArmPnuematics(INTAKE_SOLENOID_LEFT_A, INTAKE_SOLENOID_LEFT_B, INTAKE_SOLENOID_RIGHT_A, INTAKE_SOLENOID_RIGHT_B);
    	toteRangeFinder = new ToteRangeFinder(RANGE_FINDER_TOTE);
    	feederStationToteFinder = new ToteRangeFinder(RANGE_FINDER_STATION);
    	
    	forwardAutonomous = new DriveDistance(20, 1);
    	turn = new TurnGyro(0.5, 360, 15);
    	autonomousDrive = new CommandListExecutor<Drive>(forwardAutonomous, turn);
    	
    	drive.initGyro();
    	
    	oi = new OI();
    }

    public void autonomousInit() {
    	elevator.zeroElevator();
    	drive.resetLeftEncoder();
    	drive.resetRightEncoder();
    	timer.reset();
    	timer.start();
    	autonomousDrive.reset();
    	/*
    	upperArms.setSpeeds(-1, 1);
    	lowerArms.setSpeeds(1, -1);
    	*/
    }
    
    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
    	
    	autonomousDrive.execute(drive);
    	
    	/*
    	if (timer.get() < 1.5) {
    		drive.setMotorOutputs(0.75, 0.75);
    	} else {
    		drive.setMotorOutputs(0, 0);
    	}
    	//*/
    	
    	/*
    	if (timer.get() < 0.7+5) {}else if (timer.get() < 3+5) {
    		drive.setMotorOutputs(-0.75, -0.75);
    	}else if (timer.get() < 4+5){
    		drive.setMotorOutputs(-0.4, -0.4);
    	}else if (timer.get() < 7+5) {
    		drive.setMotorOutputs(1, 1);
    	}else {
    		drive.setMotorOutputs(0, 0);
    	}
    	
    	if (timer.get() < 3+5 && timer.get() > 5) {
    		arms.setArms(true, true);
    		upperArms.setSpeeds(-1, 1);
    		lowerArms.setSpeeds(1, -1);
    	}else if (timer.get() < 4+5) {
    		arms.setArms(false, false);
    		upperArms.setSpeeds(0, 0);
    		lowerArms.setSpeeds(0, 0);
    	}else if (timer.get() < 6.5+5) {
    		arms.setArms(true, true);
    		upperArms.setSpeeds(0, 0);
    		lowerArms.setSpeeds(0, 0);
    	} else if (timer.get() > 5) {
    		arms.setArms(true, true);
    		upperArms.setSpeeds(0, 0);
    		lowerArms.setSpeeds(0, 0);
    	}
    	rollers.setSpeed(1);
    	*/
    	
    	//autonomousDrive.execute(drive);
    	elevator.elevatorRun();
    }
    
    /**
     * This function is called at the beginning of teleop
     */
    public void teleopInit() {
    	elevator.zeroElevator();
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
    	SmartDashboard.putNumber("Gyro", drive.getAngle());
    	
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
				elevator.gotoBottom();
			}
			if (oi.liftToteBtn.get()) {
				SmartDashboard.putBoolean("liftTote", true);
				elevator.intakeTote();
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
		
		//rollers
		double driverRollerSpeed = oi.driverXbox.getRawAxis(2) - oi.driverXbox.getRawAxis(3);
		double shooterRollerSpeed = oi.shooterXbox.getRawAxis(2) - oi.shooterXbox.getRawAxis(3);
		if (Math.abs(driverRollerSpeed) > Math.abs(shooterRollerSpeed)) {
			rollers.setSpeed(driverRollerSpeed);
		}else {
			rollers.setSpeed(shooterRollerSpeed);
		}
		
		//arms
		
		boolean botIn = oi.driverXbox.getRawButton(1);
		boolean botOut = oi.driverXbox.getRawButton(2);
		boolean topIn = oi.driverXbox.getRawButton(4);
		boolean topOut = oi.driverXbox.getRawButton(3);

		boolean driverToteIntake = oi.driverXbox.getRawButton(5);
		boolean driverContainerIntake = oi.driverXbox.getRawButton(6);
		
		boolean shooterTopIn = oi.shooterXbox.getRawButton(6);
		boolean shooterTopOut = oi.shooterXbox.getRawButton(5);
		
		double upperArmSpeed;
		double lowerArmSpeed;
		boolean leftArm, rightArm;
		
		// manual controls
		if (!(botIn || botOut || topIn || topOut)) {
			
			
			topIn = driverToteIntake || shooterTopIn;
			topOut = (driverContainerIntake && !driverToteIntake) || shooterTopOut;
			
			botIn = driverToteIntake || driverContainerIntake;
			
		}
		
		if (botIn != botOut) {
			lowerArmSpeed = botIn ? -1 : 1;
		} else {
			lowerArmSpeed = 0;
		}
		
		if (topIn != topOut) {
			upperArmSpeed = topIn ? -1 : 1;
		} else {
			upperArmSpeed = 0;
		}
		
		if (driverContainerIntake || driverToteIntake) {
			leftArm = rightArm = true;
		} else {
			double shooterArmX = oi.shooterXbox.getRawAxis(4);
			double shooterArmY = oi.shooterXbox.getRawAxis(5);
			
			double threshhold = .3;
			
			leftArm = shooterArmX > threshhold || shooterArmY > threshhold;
			rightArm = shooterArmX < -threshhold || shooterArmY > threshhold;
		}
		
//		double driverLowerArmSpeed = Math.min(1, (oi.driverXbox.getRawButton(5) ? 1:0) + (oi.driverXbox.getRawButton(6) ? 1:0));	//if left or right bumper is pressed, lower arms spin inward
//		double driverUpperArmSpeed = (oi.driverXbox.getRawButton(5) ? 1:0) + (oi.driverXbox.getRawButton(6) ? -1:0);				//if left bumper is pressed, upper arms spin inward, if right, outward
//		
//		double manualLowerArmSpeed = (oi.driverXbox.getRawButton(1) ? 1:0) + (oi.driverXbox.getRawButton(2) ? -1:0);				//A and B correspond to out and in for lower arms
//		double manualUpperArmSpeed = (oi.driverXbox.getRawButton(4) ? 1:0) + (oi.driverXbox.getRawButton(3) ? -1:0);				//Y and X correspond to out and in for upper arms
//		
//		double shooterUpperArmSpeed = (oi.shooterXbox.getRawButton(5) ? 1:0) + (oi.shooterXbox.getRawButton(6) ? -1:0);			//if left bumper is pressed, upper arms spin inward, if right
//		
//		boolean armState = (oi.driverXbox.getRawButton(5) || oi.driverXbox.getRawButton(6)) ? true:false;							//if either of the bumpers is pressed, arms are in
//		
//		if (manualLowerArmSpeed == 0 && manualUpperArmSpeed == 0) {
//			if (Math.abs(shooterUpperArmSpeed) > Math.abs(driverUpperArmSpeed)) {
//				upperArmSpeed = shooterUpperArmSpeed;
//			}else {
//				upperArmSpeed = driverUpperArmSpeed;
//			}
//			lowerArmSpeed = driverLowerArmSpeed;
//		}else {
//			upperArmSpeed = manualUpperArmSpeed;
//			lowerArmSpeed = manualLowerArmSpeed;
//		}
		
		upperArms.setSpeeds(upperArmSpeed, -upperArmSpeed);
		lowerArms.setSpeeds(-lowerArmSpeed, lowerArmSpeed);
		arms.setArms(leftArm, rightArm);
		
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
    	maxOutput = oi.turboBtn.get() ? HIGH_GEAR_COEFFICIENT:LOW_GEAR_COEFFICIENT;
    	
		drive.drive(moveValue, rotateValue, maxOutput);
	}
    
    @Override
    public void disabledInit() {
    	drive.drive(0, 0, LOW_GEAR_COEFFICIENT);
    	elevator.idle();
    }
    
    @Override
    public void disabledPeriodic() {
    }
}