package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.Constants;
import frc.robot.HardwareMap;

public class BallHandlingFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		START_STATE,
		FIRE_SHOOTER,
		RETRACT_SHOOTER,
		DEPRESSURIZE_SHOOTER,
		RETRACT_INTAKE_MECH,
		RELEASE_INTAKE_MECH,
		BALL_INTO_TERMINAL,
		MOVE_MOTOR_RED_MECH
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private DoubleSolenoid pushSolenoid;
	private DoubleSolenoid pullSolenoid;
	private DoubleSolenoid intakeDeploySolenoid;

	private CANSparkMax intakeMotor;

	private PowerDistribution pDH;

	private Timer stateTimer;

	private boolean isShooterPistonExtended;
	private boolean isShooterPistonPressurized;
	private boolean isIntakeMechRetracted;

	private boolean[] hasDepositedBall;

	private ColorSensorV3 ballDetector;

	public enum IntakeMechBallStates {
		NONE,
		RED,
		BLUE
	}

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public BallHandlingFSM() {
		// Perform hardware init
		pushSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			HardwareMap.PCM_CHANNEL_SHOOTER_SOLENOID_EXTEND,
			HardwareMap.PCM_CHANNEL_SHOOTER_SOLENOID_EXTEND_RELEASE);
		pullSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			HardwareMap.PCM_CHANNEL_SHOOTER_SOLENOID_RETRACT,
			HardwareMap.PCM_CHANNEL_SHOOTER_SOLENOID_RETRACT_RELEASE);

		intakeDeploySolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			HardwareMap.PCM_CHANNEL_INTAKE_RELEASE_SOLENOID,
			HardwareMap.PCM_CHANNEL_INTAKE_RETRACT_SOLENOID);

		intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTAKE,
			MotorType.kBrushless);

		ballDetector = new ColorSensorV3(I2C.Port.kOnboard);
		pDH = new PowerDistribution(1, ModuleType.kRev);

		// Reset state machine
		stateTimer = new Timer();
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
		stateTimer.reset();
		hasDepositedBall = new boolean[Constants.MAX_NUMBER_OF_BALLS];
		currentState = FSMState.START_STATE;

		isShooterPistonExtended = false;
		isShooterPistonPressurized = false;

		isIntakeMechRetracted = true;
		// Call one tick of update to ensure outputs reflect start state
		update(null, DriveFSMSystem.FSMState.PURE_PURSUIT);
		currentState = FSMState.START_STATE;
	}
	/**
	 * Update FSM based on new inputs in Autonomous. This function only calls
	 * the FSM state specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param driveState Current FSMState of DriveFSMSystem.
	 */
	public void update(TeleopInput input, DriveFSMSystem.FSMState driveState) {
		updateIsInShootingPositionIndicator(false);
		SmartDashboard.putBoolean("Red Ball", getBallInMech() == IntakeMechBallStates.RED);
		SmartDashboard.putBoolean("Blue Ball", getBallInMech() == IntakeMechBallStates.BLUE);

		switch (currentState) {
			case START_STATE:
				handleStartState(input);
				break;

			case IDLE:
				handleIdleState(input);
				break;

			case FIRE_SHOOTER:
				handleFireShooterState(input);
				break;

			case RETRACT_SHOOTER:
				handleRetractShooterState(input);
				break;

			case RETRACT_INTAKE_MECH:
				handleRetractIntakeMechState(input);
				break;

			case RELEASE_INTAKE_MECH:
				handleReleaseIntakeMechState(input);
				break;

			case BALL_INTO_TERMINAL:
				handleBallIntoTerminalState(input);
				break;

			case DEPRESSURIZE_SHOOTER:
				handleDepressurizeShooterState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input, driveState);
	}

	/* ======================== Private methods ======================== */
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @param driveState Current FSMState of DriveFSMSystem.
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input, DriveFSMSystem.FSMState driveState) {
		if (input == null) { //auto
			if (currentState == FSMState.START_STATE) {
				return FSMState.DEPRESSURIZE_SHOOTER;
			}
			if (!isShooterPistonExtended && !isShooterPistonPressurized
					&& driveState.getBallIndex() < Constants.MAX_NUMBER_OF_BALLS
					&& !hasDepositedBall[driveState.getBallIndex()]) {
				hasDepositedBall[driveState.getBallIndex()] = true;
				restartTimer();

				return FSMState.FIRE_SHOOTER;
			} else if (isShooterPistonExtended) {
				if (stateTimer.hasElapsed(Constants.TIME_FOR_FULL_SHOT)) {
					restartTimer();

					return FSMState.RETRACT_SHOOTER;
				} else if (stateTimer.hasElapsed(Constants.TIME_FOR_PISTON_EXTENSION)
						&& isShooterPistonPressurized) {
					return FSMState.DEPRESSURIZE_SHOOTER;
				}
			} else if (stateTimer.hasElapsed(Constants.TIME_TO_DEPRESSURIZATION)
						&& isShooterPistonPressurized) {
				return FSMState.DEPRESSURIZE_SHOOTER;
			}
			return FSMState.IDLE;
		}

		switch (currentState) {
			case START_STATE:
				return FSMState.DEPRESSURIZE_SHOOTER;

			case IDLE:
				if (!isShooterPistonExtended
						&& !isShooterPistonPressurized
						&& input.isShooterButtonPressed()) {
					restartTimer();

					return FSMState.FIRE_SHOOTER;
				} else if (isShooterPistonExtended) {
					if (stateTimer.hasElapsed(Constants.TIME_FOR_FULL_SHOT)) {
						restartTimer();

						return FSMState.RETRACT_SHOOTER;
					} else if (stateTimer.hasElapsed(Constants.TIME_FOR_PISTON_EXTENSION)
							&& isShooterPistonPressurized) {
						return FSMState.DEPRESSURIZE_SHOOTER;
					}
				} else if (stateTimer.hasElapsed(Constants.TIME_TO_DEPRESSURIZATION)
						&& isShooterPistonPressurized) {
					return FSMState.DEPRESSURIZE_SHOOTER;
				} else if (input.wasToggleIntakeButtonPressed()) {
					if (isIntakeMechRetracted) {
						return FSMState.RELEASE_INTAKE_MECH;
					} else {
						return FSMState.RETRACT_INTAKE_MECH;
					}
				} else if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.BALL_INTO_TERMINAL;
				} else {
					return FSMState.IDLE;
				}

			case BALL_INTO_TERMINAL:
				if (!isShooterPistonExtended
						&& !isShooterPistonPressurized
						&& input.isShooterButtonPressed()) {
					restartTimer();

					return FSMState.FIRE_SHOOTER;
				} else if (isShooterPistonExtended) {
					if (stateTimer.hasElapsed(Constants.TIME_FOR_FULL_SHOT)) {
						restartTimer();

						return FSMState.RETRACT_SHOOTER;
					} else if (stateTimer.hasElapsed(Constants.TIME_FOR_PISTON_EXTENSION)
							&& isShooterPistonPressurized) {
						return FSMState.DEPRESSURIZE_SHOOTER;
					}
				} else if (stateTimer.hasElapsed(Constants.TIME_TO_DEPRESSURIZATION)
						&& !isShooterPistonExtended && isShooterPistonPressurized) {
					return FSMState.DEPRESSURIZE_SHOOTER;
				} else if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.BALL_INTO_TERMINAL;
				} else {
					return FSMState.IDLE;
				}

			case RETRACT_INTAKE_MECH:
				return FSMState.IDLE;

			case RELEASE_INTAKE_MECH:
				return FSMState.IDLE;

			case FIRE_SHOOTER:
				if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.BALL_INTO_TERMINAL;
				} else {
					return FSMState.IDLE;
				}

			case RETRACT_SHOOTER:
				return FSMState.IDLE;

			case DEPRESSURIZE_SHOOTER:
				if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.BALL_INTO_TERMINAL;
				} else {
					return FSMState.IDLE;
				}

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
		pushSolenoid.set(DoubleSolenoid.Value.kOff);
		pullSolenoid.set(DoubleSolenoid.Value.kOff);
		intakeDeploySolenoid.set(DoubleSolenoid.Value.kOff);
		intakeMotor.setVoltage(
			getBallInMech() == IntakeMechBallStates.NONE
			|| input.isForwardIntakeButtonPressed()
				? -Constants.INTAKE_MOTOR_VOLTAGE : 0);
	}
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartState(TeleopInput input) {
		handleRetractIntakeMechState(input);
		handleRetractShooterState(input);
	}
	/**
	 * Handle behavior in FIRE_SHOOTER.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFireShooterState(TeleopInput input) {
		pushSolenoid.set(DoubleSolenoid.Value.kForward);

		isShooterPistonExtended = true;
		isShooterPistonPressurized = true;
	}
	/**
	 * Handle behavior in RETRACT_SHOOTER.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRetractShooterState(TeleopInput input) {
		pullSolenoid.set(DoubleSolenoid.Value.kForward);

		isShooterPistonExtended = false;
		isShooterPistonPressurized = true;
	}
	/**
	 * Handle behavior in RETRACT_INTAKE_MECH.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRetractIntakeMechState(TeleopInput input) {
		isIntakeMechRetracted = true;
		intakeDeploySolenoid.set(DoubleSolenoid.Value.kReverse);
	}
	/**
	 * Handle behavior in RELEASE_INTAKE_MECH.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleReleaseIntakeMechState(TeleopInput input) {
		isIntakeMechRetracted = false;
		intakeDeploySolenoid.set(DoubleSolenoid.Value.kForward);
	}
	/**
	 * Handle behavior in BALL_INTO_TERMINAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleBallIntoTerminalState(TeleopInput input) {
		pushSolenoid.set(DoubleSolenoid.Value.kOff);
		pullSolenoid.set(DoubleSolenoid.Value.kOff);
		intakeDeploySolenoid.set(DoubleSolenoid.Value.kOff);
		intakeMotor.setVoltage(Constants.INTAKE_MOTOR_VOLTAGE);
	}
	/**
	 * Handle behavior in DEPRESSURIZE_SHOOTER.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleDepressurizeShooterState(TeleopInput input) {
		pushSolenoid.set(DoubleSolenoid.Value.kReverse);
		pullSolenoid.set(DoubleSolenoid.Value.kReverse);
		isShooterPistonPressurized = false;
	}

	// private void moveMotorForHanger(TeleopInput input,
	// 	double inches) {

	// 	double currentPosTicks = -magicMotor.getEncoder().getPosition();


	// 	// Checks if either the encoder value is equal to the inches required (reached destination)
	// 	// or checks if the robot has hit a wall (encoder value is not changing and motor power is
	// 	// not zero)
	// 	if ((inches > 0 && error < Constants.ERR_THRESHOLD_STRAIGHT_IN)
	// 		|| (inches < 0 && error > -Constants.ERR_THRESHOLD_STRAIGHT_IN)) {

	// 		magicMotor.set(0);
	// 		return;
	// 	}

	// 	double speed = 0.1;

	// 	magicMotor.set(speed);
	// }

	/**
	 * Get relavent Spark Max instances for simulation.
	 * @return All used Spark Max instances to be tested
	 */
	public CANSparkMax[] getSparkMaxs() {
		return new CANSparkMax[]{
			intakeMotor
		};
	}

	//SYSTEM IDENTIFIERS

	/**
	 * Determines whether or not a ball is in the intake mech.
	 * @return Whether or not a ball is in the intake mech
	 */
	public boolean isBallInIntake() {
		return ballDetector.getProximity() > Constants.BALL_PROXIMITY_THRESHOLD;
	}

	/**
	 * Determines the presence and/or color of the ball in the intake mech.
	 * @return The type of ball in the intake mechanism
	 */
	public IntakeMechBallStates getBallInMech() {
		if (!isBallInIntake()) {
			return IntakeMechBallStates.NONE;
		} else if (ballDetector.getRed() > ballDetector.getBlue()) {
			return IntakeMechBallStates.RED;
		} else {
			return IntakeMechBallStates.BLUE;
		}
	}

	private void updateIsInShootingPositionIndicator(boolean isInShootingPosition) {
		pDH.setSwitchableChannel(isInShootingPosition);
	}

	private void restartTimer() {
		stateTimer.reset();
		stateTimer.start();
	}
}
