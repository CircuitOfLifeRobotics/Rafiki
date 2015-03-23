//UPDATED ELVATOR CODE FOR NEW DESIGN WITHOUT LATCHES REFER TO "OldElevator.java" FOR OLD CODE WITH LATCHES

package org.usfirst.frc.team3925.robot.subsystem;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Talon;

public class Elevator {
	
	public enum State {
		IDLE, RESET,
		LIFT_INIT, LIFT_WAIT_DOWN, LIFT_WAIT_UP,
		LOWER_INIT, LOWER_WAIT_DOWN;
	}
	
	
	//sets constants
	private static final double MAX_ELEVATOR_HEIGHT = 13.25;
	private static final double TOLERANCE = 0.1;
	private static final double UP_SPEED = -0.5d;
	private static final double DOWN_SPEED = 0.5d;
	private static final double TOTE_HEIGHT = 13.500001;
	private static double elevatorEncoderOffset = 0;

	//vital variables
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
		if (doLimits) {
			if (getCurrentHeight() >= MAX_ELEVATOR_HEIGHT && speed > 0) {
				speed = 0;
			}else if (getCurrentHeight() <= 0 && speed < 0) {
				speed = 0;
			}
		}
		speed = -speed; // motors on the robot are inverted
		leftElevatorMotor.set(speed);
		rightElevatorMotor.set(speed);
	}
	
	//sets elevator speed based on target height, always constant, stops if it is at target within tolerance
	private void updateHeight(double height, boolean doLimits) {
		double current = getCurrentHeight();
		if (Math.abs(height-current) > TOLERANCE) {
			if (current > height) {
				setElevatorSpeed(UP_SPEED, doLimits);
			}else {
				setElevatorSpeed(DOWN_SPEED, doLimits);
			}
		}else {
			setElevatorSpeed(0, doLimits);
		}
	}
	
	//state machine that runs the elevator
	public void elevatorRun() {// no activity
		switch (state) {
		case IDLE:
			targetHeight = getCurrentHeight();
			break;
		case LIFT_INIT:
			targetHeight = 0;
			state = State.LIFT_WAIT_DOWN;
		case LIFT_WAIT_DOWN:
			if (Math.abs(getCurrentHeight()-targetHeight) < TOLERANCE) {
				targetHeight = TOTE_HEIGHT;
				state = State.LIFT_WAIT_UP;
			}
			break;
		case LIFT_WAIT_UP:
			if (Math.abs(getCurrentHeight()-targetHeight) < TOLERANCE) {
				state = State.IDLE;
			}
			break;
		case LOWER_INIT:
			targetHeight = 0;
			state = State.LOWER_WAIT_DOWN;
		case LOWER_WAIT_DOWN:
			if (Math.abs(getCurrentHeight()-targetHeight) < TOLERANCE) {
				state = State.IDLE;
			}
			break;
		case RESET:
			if (Math.abs(getCurrentHeight()-targetHeight) < TOLERANCE) {
				targetHeight = 0;
				elevatorEncoderOffset = 0;
				state = State.LIFT_WAIT_UP;
			}
			if (!limitSwitch1.get()/* || !limitSwitch2.get()*/) {
				elevatorEncoder.reset();
				targetHeight = 0;
				state = State.LIFT_WAIT_UP;
			}
			
		}
		updateHeight(targetHeight, state != State.RESET);
	}
	
	//resets the height to 0
	public void zeroElevator() {
		targetHeight = -1.2 * MAX_ELEVATOR_HEIGHT;
    	state = State.RESET;
	}
	
	//lifts the elevator to tote height
	public void liftStack() {
		state = State.LIFT_INIT;
	}
	
	//lowers the elevator to 0
	public void lowerStack() {
		state = State.LOWER_INIT;
	}

	public void idle() {
		state = State.IDLE;
	}
	
}