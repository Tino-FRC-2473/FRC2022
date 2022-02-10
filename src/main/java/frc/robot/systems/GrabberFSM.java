package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;

// Third Party Hardware Imports
import com.revrobotics.CANSparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.HardwareMap;

public class GrabberFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		LOW_IDLE,
		ASCENDING,
    	CONTROL,
    	DESCENDING,
    	MID_IDLE
	}


	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private Joystick joystick;
	private CANSparkMax motor;
	private static final int THRESHOLD = 100;
	private double power = 0.3; 

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public GrabberFSM() {
		// Perform hardware init
		joystick = new Joystick(HardwareMap.CAN_ID_JOYSTICK);

        motor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_SHOOTER, CANSparkMax.MotorType.kBrushless);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.LOW_IDLE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case LOW_IDLE:
				handleLowIdleState(input);
				break;

			case ASCENDING:
				handleAscendingState(input);
				break;
				
			case DESCENDING:
                handleDescendingState(input);
				break;
				
			case CONTROL:
				handleControlState(input);
				break;

			case MID_IDLE:
				handleMidIdleState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input) {
    	boolean isAtLowerThresh = motor.getEncoder().getPosition() < 0;
    	boolean isAtUpperThresh = motor.getEncoder().getPosition() > THRESHOLD;


		switch (currentState) {
			case LOW_IDLE:
				if (input != null){
                	if (input.isAscendingButtonPressed() && !input.isDescendingButtonPressed()) {
                    	return FSMState.ASCENDING;
                	} else {
                    	return FSMState.LOW_IDLE;
                	}
            	}
				return currentState;
				
			case ASCENDING:
				if (input != null){
                	if (isAtUpperThresh) {
                    	return FSMState.MID_IDLE;
                	} else if (!input.isAscendingButtonPressed() && input.isDescendingButtonPressed()) {
                    	return FSMState.DESCENDING;
                    }else if (input.isAscendingButtonPressed() && !input.isDescendingButtonPressed()) {
                    	return FSMState.ASCENDING;
                    } else {
                    	return FSMState.CONTROL;
                	}
            	}
				return currentState;
            
            case DESCENDING:
                if (input != null) {
                    if (isAtLowerThresh) {
                        return FSMState.LOW_IDLE;
                    } else if (!input.isAscendingButtonPressed() && input.isDescendingButtonPressed()) {
                        return FSMState.DESCENDING;
                    } else if (input.isAscendingButtonPressed() && !input.isDescendingButtonPressed()) {
                        return FSMState.ASCENDING;
                    } else {
                        return FSMState.CONTROL;
                    }
                }
                return currentState;
            
            case CONTROL: 
                if (input != null) {
                    if (input.isAscendingButtonPressed() && !input.isDescendingButtonPressed()) {
                        return FSMState.ASCENDING;
                    } else if (!input.isAscendingButtonPressed() && input.isDescendingButtonPressed()) {
                        return FSMState.DESCENDING;
                    } else {
                        return FSMState.CONTROL;
                    }
                }
                return currentState;
            
            case MID_IDLE:
                if (input != null) {
                    if (!input.isAscendingButtonPressed() && input.isDescendingButtonPressed()) {
                        return  FSMState.DESCENDING;
                    } else {
                        return  FSMState.MID_IDLE;
                    }
                }
                return currentState;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
        
		}

        
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in LOW_IDLE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleLowIdleState(TeleopInput input) {
		motor.set(0);
	}

	/**
	 * Handle behavior in MID_IDLE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleMidIdleState(TeleopInput input) {
		motor.set(0);
	}

    /**
	 * Handle behavior in CONTROL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleControlState(TeleopInput input) {
		motor.set(0);
	}

    /**
	 * Handle behavior in ASCENDING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleAscendingState(TeleopInput input) {
		motor.set(power);
	}

    /**
	 * Handle behavior in DESCENDING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleDescendingState(TeleopInput input) {
		motor.set(-power);
	}
}
