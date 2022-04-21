// package frc.robot.systems;

// // // WPILib Imports
// // import edu.wpi.first.wpilibj.DoubleSolenoid;
// // import edu.wpi.first.wpilibj.I2C;
// // import edu.wpi.first.wpilibj.PneumaticsModuleType;
// // import edu.wpi.first.wpilibj.PowerDistribution;
// // import edu.wpi.first.wpilibj.Timer;
// // import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
// // import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// // Third party Hardware Imports
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// // Robot Imports
// import frc.robot.TeleopInput;

// public class MidHangerFSMSystem {
// 	/* ======================== Constants ======================== */
// 	// FSM state definitions
// 	public enum FSMState {
// 		IDLE,
// 		AT_BOTTOM_STATE,
// 		FULLY_EXTENDED_STATE,
// 		EXTENDED_TO_MID_RUNG_HEIGHT_STATE,
// 		DESCEND_TO_GROUND_STATE
// 	}

// 	/* ======================== Private variables ======================== */
// 	private FSMState currentState;

// 	private CANSparkMax magicMotor;

// 	private double magicMotorSpeed;
// 	private double startingPosTicks;

// 	private double posTicksForBottomSwitchRetract;
// 	private double posTicksFullyExtended;
// 	private double posTicksBelowClip;

// 	private boolean isAscendingButtonPressed;
// 	private boolean isDescendingButtonPressed;
// 	private boolean isHangerAtBottom;
// 	private boolean isHangerFullyExtended;
// 	private boolean isHangerAtMidRungHeight;
// 	private boolean readyToPressDescendButton;
// 	private boolean isRobotAtGroundLevel;
// 	/* ======================== Constructor ======================== */
// 	/**
// 	 * Create FSMSystem and initialize to starting state. Also perform any
// 	 * one-time initialization or configuration of hardware required. Note
// 	 * the constructor is called only once when the robot boots.
// 	 */
// 	public MidHangerFSMSystem() {
// 		// hardware init
// 		magicMotor = new CANSparkMax(7, MotorType.kBrushless);

// 		magicMotorSpeed = 0.3;

// 		startingPosTicks = magicMotor.getEncoder().getPosition();
// 		posTicksForBottomSwitchRetract = startingPosTicks - 5;
// 		posTicksFullyExtended = startingPosTicks + 30;
// 		posTicksBelowClip = startingPosTicks + 20;

// 		isAscendingButtonPressed = false;
// 		isDescendingButtonPressed = false;
// 		isHangerAtBottom = false;
// 		isHangerFullyExtended = false;
// 		isHangerAtMidRungHeight = false;
// 		readyToPressDescendButton = false;
// 		isRobotAtGroundLevel = false;

// 		reset();
// 	}

// 	/* ======================== Public methods ======================== */
// 	/**
// 	 * Return current FSM state.
// 	 * @return Current FSM state
// 	 */
// 	public FSMState getCurrentState() {
// 		return currentState;
// 	}
// 	/**
// 	 * Reset this system to its start state. This may be called from mode init
// 	 * when the robot is enabled.
// 	 *
// 	 * Note this is distinct from the one-time initialization in the constructor
// 	 * as it may be called multiple times in a boot cycle,
// 	 * Ex. if the robot is enabled, disabled, then reenabled.
// 	 */
// 	public void reset() {
// 		currentState = FSMState.IDLE;
// 	}
// 	/**
// 	 * Update FSM based on new inputs in Autonomous. This function only calls
// 	 * the FSM state specific handlers.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 * @param driveState Current FSMState of DriveFSMSystem.
// 	 */
// 	public void update(TeleopInput input, DriveFSMSystem.FSMState driveState) {

// 		switch (currentState) {

// 			case IDLE:
// 				handleIdleState(input);
// 				break;

// 			case AT_BOTTOM_STATE:
// 				handleAtBottomState(input);
// 				break;

// 			case FULLY_EXTENDED_STATE:
// 				handleFullyExtendedState(input);
// 				break;

// 			case EXTENDED_TO_MID_RUNG_HEIGHT_STATE:
// 				handleExtendedToMidRunHeightState(input);
// 				break;

// 			case DESCEND_TO_GROUND_STATE:
// 				handleDescendToGroundState(input);
// 				break;

// 			default:
// 				throw new IllegalStateException("Invalid state: " + currentState.toString());
// 		}
// 		currentState = nextState(input, driveState);
// 	}

// 	/* ======================== Private methods ======================== */
// 	/**
// 	 * Decide the next state to transition to. This is a function of the inputs
// 	 * and the current state of this FSM. This method should not have any side
// 	 * effects on outputs. In other words, this method should only read or get
// 	 * values to decide what state to go to.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 * @param driveState Current FSMState of DriveFSMSystem.
// 	 * @return FSM state for the next iteration
// 	 */
// 	private FSMState nextState(TeleopInput input, DriveFSMSystem.FSMState driveState) {
// 		switch (currentState) {
// 			case IDLE:
// 				if (isAscendingButtonPressed) {
// 					isAscendingButtonPressed = false;
// 					return FSMState.AT_BOTTOM_STATE;
// 				} else if (isDescendingButtonPressed) {
// 					isDescendingButtonPressed = false;
// 					return FSMState.DESCEND_TO_GROUND_STATE;
// 				}

// 				return FSMState.IDLE;

// 			case AT_BOTTOM_STATE:
// 				if (isHangerAtBottom) {
// 					isHangerAtBottom = false;
// 					return FSMState.FULLY_EXTENDED_STATE;
// 				} else {
// 					return FSMState.AT_BOTTOM_STATE;
// 				}

// 			case FULLY_EXTENDED_STATE:
// 				if (isHangerFullyExtended) {
// 					isHangerFullyExtended = false;
// 					return FSMState.EXTENDED_TO_MID_RUNG_HEIGHT_STATE;
// 				} else {
// 					return FSMState.FULLY_EXTENDED_STATE;
// 				}

// 			case EXTENDED_TO_MID_RUNG_HEIGHT_STATE:
// 				if (isHangerAtMidRungHeight) {
// 					isHangerAtMidRungHeight = false;
// 					readyToPressDescendButton = true;
// 					return FSMState.IDLE;
// 				} else {
// 					return FSMState.EXTENDED_TO_MID_RUNG_HEIGHT_STATE;
// 				}

// 			case DESCEND_TO_GROUND_STATE:
// 				if (isRobotAtGroundLevel) {
// 					isRobotAtGroundLevel = false;
// 					return FSMState.IDLE;
// 				} else {
// 					return FSMState.DESCEND_TO_GROUND_STATE;
// 				}

// 			default:
// 				throw new IllegalStateException("Invalid state: " + currentState.toString());
// 		}
// 	}

// 	/* ------------------------ FSM state handlers ------------------------ */
// 	/**
// 	 * Handle behavior in IDLE.
// 	 * @param input Global TeleopInput if robot in teleop mode or null if
// 	 *        the robot is in autonomous mode.
// 	 */
// 	private void handleIdleState(TeleopInput input) {
// 		magicMotor.set(0);

// 		if (input.isAscendingButtonPressed()) {
// 			isAscendingButtonPressed = true;
// 			return;
// 		}

// 		if (readyToPressDescendButton && input.isDescendingButtonPressed()) {
// 			readyToPressDescendButton = false;
// 			isDescendingButtonPressed = true;
// 			return;
// 		}
// 	}

// 	private void handleAtBottomState(TeleopInput input) {
// 		if (magicMotor.getEncoder().getPosition() <= posTicksForBottomSwitchRetract) {
// 			isHangerAtBottom = true;
// 			return;
// 		}

// 		magicMotor.set(-magicMotorSpeed);

// 	}


// 	private void handleFullyExtendedState(TeleopInput input) {
// 		if (magicMotor.getEncoder().getPosition() >= posTicksFullyExtended) {
// 			isHangerFullyExtended = true;
// 			return;
// 		}

// 		magicMotor.set(magicMotorSpeed);

// 	}

// 	private void handleExtendedToMidRunHeightState(TeleopInput input) {
// 		if (magicMotor.getEncoder().getPosition() <= posTicksBelowClip) {
// 			isHangerAtMidRungHeight = true;
// 			return;
// 		}

// 		magicMotor.set(-magicMotorSpeed);
// 	}

// 	private void handleDescendToGroundState(TeleopInput input) {
// 		if (magicMotor.getEncoder().getPosition() <= startingPosTicks) {
// 			isRobotAtGroundLevel = true;
// 			return;
// 		}

// 		magicMotor.set(0.1);
// 	}
// }
