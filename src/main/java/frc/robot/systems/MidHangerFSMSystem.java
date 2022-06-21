package frc.robot.systems;

// // WPILib Imports
// import edu.wpi.first.wpilibj.DoubleSolenoid;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// wpilib imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
// Robot Imports
import frc.robot.TeleopInput;

public class MidHangerFSMSystem {
	/* ======================== Constants ======================== */

	// FSM state definitions
	public enum FSMState {
		IDLE,
		AT_BOTTOM_STATE,
		FULLY_EXTENDED_STATE,
		EXTENDED_BELOW_MID_RUNG_HEIGHT_STATE,
		PISTON_LOCK_ENGAGED_STATE,
		EXTENDED_AT_MID_RUNG_HEIGHT_STATE
		// DESCEND_TO_GROUND_STATE
	}

	/* ======================== Private variables ======================== */

	private FSMState currentState;

	private CANSparkMax magicMotor;

	private DoubleSolenoid switchSolenoid;

	private double magicMotorSpeed;
	private double startingPosTicks;

	private double posTicksForBottomSwitchRetract;
	private double posTicksFullyExtended;
	private double posTicksBelowClip;
	private double posTicksRightAtClip;

	private boolean isAscendingButtonPressed;
	// private boolean isDescendingButtonPressed;
	private boolean isArmDescendingButtonPressed;
	private boolean isHangerAtBottom;
	private boolean isHangerFullyExtended;
	private boolean isHangerBelowMidRungHeight;
	private boolean isHangerAtMidRungHeight;
	private boolean isPistonSwitchFired;

	private Timer timerForOff;
	// private boolean readyToPressDescendButton;
	// private boolean isRobotAtGroundLevel;
	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public MidHangerFSMSystem() {
		// hardware init
		magicMotor = new CANSparkMax(5, MotorType.kBrushless);

		magicMotorSpeed = -0.4;

		magicMotor.getEncoder().setPosition(0);

		switchSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

		startingPosTicks = magicMotor.getEncoder().getPosition();
		/*
		 * Value of 33.428 is from testing encoder value on Wed
		 */
		posTicksForBottomSwitchRetract = startingPosTicks - 30;

		/*
		 * FIND ENCODER VALUE
		 */
		posTicksFullyExtended = startingPosTicks + 647;

		/*
		 * FIND ENOCDER VALUE
		 */
		posTicksBelowClip = startingPosTicks + 343;

		/*
		 * FIND ENOCDER VALUE
		 */
		posTicksRightAtClip = startingPosTicks + 500;

		isAscendingButtonPressed = false;
		// isDescendingButtonPressed = false;
		isArmDescendingButtonPressed = false;
		isHangerAtBottom = false;
		isHangerFullyExtended = false;
		isHangerBelowMidRungHeight = false;
		isHangerAtMidRungHeight = false;
		isPistonSwitchFired = false;
		// readyToPressDescendButton = false;
		// isRobotAtGroundLevel = false;

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
		currentState = FSMState.IDLE;
		startingPosTicks = magicMotor.getEncoder().getPosition();
		switchSolenoid.set(DoubleSolenoid.Value.kReverse);
		timerForOff = new Timer();
		timerForOff.reset();
        magicMotor.getEncoder().setPosition(0);
	}
	/**
	 * Update FSM based on new inputs in Autonomous. This function only calls
	 * the FSM state specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		System.out.println(magicMotor.getEncoder().getPosition() - startingPosTicks);
		switch (currentState) {

			case IDLE:
				handleIdleState(input);
				break;

			case AT_BOTTOM_STATE:
				handleAtBottomState(input);
				break;

			case FULLY_EXTENDED_STATE:
				handleFullyExtendedState(input);
				break;

			case EXTENDED_BELOW_MID_RUNG_HEIGHT_STATE:
				handleExtendedBelowMidRunHeightState(input);
				break;

			case PISTON_LOCK_ENGAGED_STATE:
				handlePistonLockEngagedState(input);
				break;

			case EXTENDED_AT_MID_RUNG_HEIGHT_STATE:
				handleExtendedAtMidRunHeightState(input);
				break;

			// case DESCEND_TO_GROUND_STATE:
			// 	handleDescendToGroundState(input);
			// 	break;

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
		switch (currentState) {
			case IDLE:
				if (isAscendingButtonPressed) {
					isAscendingButtonPressed = false;
					return FSMState.AT_BOTTOM_STATE;
				} else if (isArmDescendingButtonPressed) {
					isArmDescendingButtonPressed = false;
					return FSMState.EXTENDED_BELOW_MID_RUNG_HEIGHT_STATE;
				}

				return FSMState.IDLE;

			case AT_BOTTOM_STATE:
				if (isHangerAtBottom) {
					isHangerAtBottom = false;
					return FSMState.FULLY_EXTENDED_STATE;
				} else {
					return FSMState.AT_BOTTOM_STATE;
				}

			case FULLY_EXTENDED_STATE:
				if (isHangerFullyExtended) {
					isHangerFullyExtended = false;
					return FSMState.IDLE;
				} else {
					return FSMState.FULLY_EXTENDED_STATE;
				}

			case EXTENDED_BELOW_MID_RUNG_HEIGHT_STATE:
				if (isHangerBelowMidRungHeight) {
					isHangerBelowMidRungHeight = false;
					// readyToPressDescendButton = true;
                    timerForOff.reset();
                    timerForOff.start();
					return FSMState.PISTON_LOCK_ENGAGED_STATE;
				} else {
					return FSMState.EXTENDED_BELOW_MID_RUNG_HEIGHT_STATE;
				}

			case PISTON_LOCK_ENGAGED_STATE:
				if (isPistonSwitchFired) {
					isPistonSwitchFired = false;
					return FSMState.EXTENDED_AT_MID_RUNG_HEIGHT_STATE;
				} else {
					return FSMState.PISTON_LOCK_ENGAGED_STATE;
				}

			case EXTENDED_AT_MID_RUNG_HEIGHT_STATE:
				if (isHangerAtMidRungHeight) {
					isHangerAtMidRungHeight = false;
					return FSMState.IDLE;
				} else {
					return FSMState.EXTENDED_AT_MID_RUNG_HEIGHT_STATE;
				}


			// case DESCEND_TO_GROUND_STATE:
			// 	if (isRobotAtGroundLevel) {
			// 		isRobotAtGroundLevel = false;
			// 		return FSMState.IDLE;
			// 	} else {
			// 		return FSMState.DESCEND_TO_GROUND_STATE;
			// 	}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in IDLE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIdleState(TeleopInput input) {
		magicMotor.set(0);

		if (input.isAscendingButtonPressed()) {
			isAscendingButtonPressed = true;
			return;
		}

		if (input.isRawDescendingPressed()) {
			magicMotor.set(-0.15);
		}

		if (input.isRawAscendingPressed()) {
			magicMotor.set(0.15);
		}

		if (input.isDescendingButtonPressed()) {
			isArmDescendingButtonPressed = true;
			return;
		}

		// if (readyToPressDescendButton && input.isDescendingButtonPressed()) {
		// 	readyToPressDescendButton = false;
		// 	isDescendingButtonPressed = true;
		// 	return;
		// }
	}

	/**
	 * Handle behavior in for the AT_BOTTOM_STATE.
	 * Arm will move down to a specific encoder value so the bottom switch
	 * can retract
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleAtBottomState(TeleopInput input) {
		if (magicMotor.getEncoder().getPosition() <= posTicksForBottomSwitchRetract) {
			isHangerAtBottom = true;
			return;
		}

		magicMotor.set(magicMotorSpeed);

	}

	/**
	 * Handle behavior in for the FULLY_EXTENDED_STATE.
	 * Arm will move up so it is fully extended.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFullyExtendedState(TeleopInput input) {
		if (magicMotor.getEncoder().getPosition() >= posTicksFullyExtended) {
			isHangerFullyExtended = true;
			return;
		}

		magicMotor.set(-magicMotorSpeed * Constants.POWER_MULTIPLIER_FOR_ACCEND_STATE);

	}

	/**
	 * Handle behavior in for the EXTEND_BELOW_MID_RUNG_HEIGHT_STATE.
	 * Arm will move down from fully extended to right below the
	 * latch.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleExtendedBelowMidRunHeightState(TeleopInput input) {
		if (magicMotor.getEncoder().getPosition() <= posTicksBelowClip) {
			isHangerBelowMidRungHeight = true;
			return;
		}

		magicMotor.set(magicMotorSpeed);
	}

	/**
	 * Handle behavior in for the PISTON_LOCK_ENGAGED_STATE.
	 * Fires piston so the latch holding the arm in place is
	 * engaged.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handlePistonLockEngagedState(TeleopInput input) {
		switchSolenoid.set(DoubleSolenoid.Value.kForward);
        magicMotor.set(0);
		if (timerForOff.hasElapsed(0.03)) {
			switchSolenoid.set(DoubleSolenoid.Value.kOff);
			isPistonSwitchFired = true;
			return;
		}
	}

	/**
	 * Handle behavior in for the EXTEND_AT_MID_RUNG_HEIGHT_STATE.
	 * Arm will move down from right below the latch to exactly
	 * the height of the latch. The height of the arm will be
	 * the same as the mid-rung.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleExtendedAtMidRunHeightState(TeleopInput input) {
		if (magicMotor.getEncoder().getPosition() >= posTicksRightAtClip) {
			isHangerAtMidRungHeight = true;
			return;
		}

		magicMotor.set(-magicMotorSpeed);
	}

	// private void handleDescendToGroundState(TeleopInput input) {
	// 	if (magicMotor.getEncoder().getPosition() <= startingPosTicks) {
	// 		isRobotAtGroundLevel = true;
	// 		return;
	// 	}

	// 	magicMotor.set(0.1);
	// }
}
