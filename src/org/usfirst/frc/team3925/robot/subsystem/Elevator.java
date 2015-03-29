//UPDATED ELVATOR CODE FOR NEW DESIGN WITHOUT LATCHES REFER TO "OldElevator.java" FOR OLD CODE WITH LATCHES

package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {
	
	public enum State {
		IDLE, RESET,
		LIFT_INIT_1, LIFT_WAIT_DOWN_1, LIFT_WAIT_UP_1,
		LIFT_INIT_2, LIFT_WAIT_DOWN_2, LIFT_WAIT_UP_2,
		LOWER_INIT, LOWER_WAIT_DOWN;
	}
	
	//sets constants
	private static final double ELEVATOR_POS_BOTTOM = 0;
	private static final double ELEVATOR_POS_GRAB_TOTE = 12.5;
	private static final double ELEVATOR_POS_3 = 25;
	private static final double ELEVATOR_POS_TOP = 32;
	private static final double TOLERANCE = 1d;
	private static final double DOWN_SPEED = -1d;
	private static final double UP_SPEED = 0.5d;
	private static double elevatorEncoderOffset = 0;
	
	//min (bottom) 19.672
	//max (top) -14.405
	//difference (total travel) 34.077
	
	//vital variables
	double speed = 0;
	double targetHeight = 0;
	State state = State.IDLE;
	
	Talon leftElevatorMotor;
	Talon rightElevatorMotor;
	Encoder elevatorEncoder;
	DigitalInput limitSwitch1;
	
	public Elevator(int leftelevatormotor, int rightelevatormotor, int encoderportA, int encoderportB, int limitswitchport1, int limitswitchport2) {
		leftElevatorMotor = new Talon(leftelevatormotor);
		rightElevatorMotor = new Talon(rightelevatormotor);
		elevatorEncoder = new Encoder(encoderportA, encoderportB);
		limitSwitch1 = new DigitalInput(limitswitchport1);
		//inches per revolution
		elevatorEncoder.setDistancePerPulse(4.25/2048);
	}
	
	//returns current elevator height (based on encoder position which should be zeroed befor calling this)
	public double getCurrentHeight() {
		return elevatorEncoder.getDistance() + elevatorEncoderOffset;
	}
	
	//sets the raw elevator speed, limits speed based on maximum height
	public void setElevatorSpeed(double speed, boolean doLimits) {
		//doLimits = false; //TODO: DEBUG
		if (doLimits) {
			if (getCurrentHeight() >= ELEVATOR_POS_TOP && speed > 0) {
				speed = 0;
			}else if (getCurrentHeight() <= 0 && speed < 0) {
				speed = 0;
			}
		}
		SmartDashboard.putNumber("speed", -speed);
		speed = -speed; // motors on the robot are inverted
		if (state == State.RESET) {
			leftElevatorMotor.set(speed/2);
			rightElevatorMotor.set(speed/2);
		}else {
			leftElevatorMotor.set(speed);
			rightElevatorMotor.set(speed);
		}
	}
	
	//sets elevator speed based on target height, always constant, stops if it is at target within tolerance
	private double updateHeight(double height, boolean doLimits) {
		double current = getCurrentHeight();
		if (Math.abs(height-current) > TOLERANCE) {
			if (current > height) {
				setElevatorSpeed(DOWN_SPEED, doLimits);
				return DOWN_SPEED;
			}else {
				setElevatorSpeed(UP_SPEED, doLimits);
				return UP_SPEED;
			}
		}else {
			setElevatorSpeed(0, doLimits);
			return 0;
		}
	}
	
	//state machine that runs the elevator
	public void elevatorRun() {// no activity
		double height = getCurrentHeight();
		double absError = Math.abs(height-targetHeight);
		switch (state) {
		case IDLE:
			targetHeight = getCurrentHeight();
			break;
		case LIFT_INIT_1:
			targetHeight = ELEVATOR_POS_GRAB_TOTE;
			state = State.LIFT_WAIT_DOWN_1;
			break;
		case LIFT_WAIT_DOWN_1:
			if (absError < TOLERANCE) {
				targetHeight = ELEVATOR_POS_TOP;
				state = State.LIFT_WAIT_UP_1;
			}
			break;
		case LIFT_WAIT_UP_1:
			if (absError < TOLERANCE) {
				state = State.IDLE;
			}
			break;
		case LIFT_INIT_2:
			targetHeight = ELEVATOR_POS_BOTTOM;
			state = State.LIFT_WAIT_DOWN_1;
			break;
		case LIFT_WAIT_DOWN_2:
			if (absError < TOLERANCE) {
				targetHeight = ELEVATOR_POS_3;
				state = State.LIFT_WAIT_UP_1;
			}
			break;
		case LIFT_WAIT_UP_2:
			if (absError < TOLERANCE) {
				state = State.IDLE;
			}
			break;
		case LOWER_INIT:
			targetHeight = ELEVATOR_POS_BOTTOM;
			state = State.LOWER_WAIT_DOWN;
			break;
		case LOWER_WAIT_DOWN:
			if (absError < TOLERANCE) {
				state = State.IDLE;
			}
			break;
		case RESET:
			if (absError < TOLERANCE) {
				targetHeight = ELEVATOR_POS_BOTTOM;
				elevatorEncoderOffset = 0;
				state = State.IDLE;
				targetHeight = ELEVATOR_POS_TOP;
			}
			if (!limitSwitch1.get()/* || !limitSwitch2.get()*/) {
				elevatorEncoder.reset();
				targetHeight = ELEVATOR_POS_BOTTOM;
				state = State.IDLE;
				targetHeight = ELEVATOR_POS_TOP;
			}
			
		}
		SmartDashboard.putBoolean("trueOrFalse", absError<TOLERANCE);
		SmartDashboard.putNumber("target", targetHeight);
		SmartDashboard.putNumber("difference", absError);
		SmartDashboard.putNumber("tolerance", TOLERANCE);
		speed = updateHeight(targetHeight, state != State.RESET);
	}
	
	//resets the height to 0
	public void zeroElevator() {
		targetHeight = -1.2 * ELEVATOR_POS_TOP;
    	state = State.RESET;
	}
	
	//lifts the elevator to tote height
	public void intakeTote() {
		state = State.LIFT_INIT_1;
	}
	
	public void liftStackLower() {
		state = State.LIFT_INIT_2;
	}
	
	//lowers the elevator to 0
	public void gotoBottom() {
		state = State.LOWER_INIT;
	}

	public void idle() {
		state = State.IDLE;
	}
	
}