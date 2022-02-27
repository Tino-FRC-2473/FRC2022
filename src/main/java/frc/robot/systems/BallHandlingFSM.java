package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

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
		FIRING,
		RETRACTING,
		RETRACT_INTAKE_MECH,
		RELEASE_INTAKE_MECH,
		INTAKING,
		RELEASING
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private DoubleSolenoid pushSolenoid;
	private DoubleSolenoid intakeDeploySolenoid;

	private CANSparkMax intakeMotor;

	private PowerDistribution pDH;

	private double pushCommandTimeStamp;

	private boolean isShooterSolenoidExtended;
	private boolean isIntakeMechRetracted;

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
			HardwareMap.PCM_CHANNEL_PUSH_BOT_SOLENOID,
			HardwareMap.PCM_CHANNEL_PULL_BOT_SOLENOID);

		intakeDeploySolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH,
			HardwareMap.PCM_CHANNEL_INTAKE_RELEASE_SOLENOID,
			HardwareMap.PCM_CHANNEL_INTAKE_RETRACT_SOLENOID);

		intakeMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_INTAKE,
			MotorType.kBrushless);

		intakeMotor.getForwardLimitSwitch(Type.kNormallyClosed);
		intakeMotor.getReverseLimitSwitch(Type.kNormallyClosed);
		ballDetector = new ColorSensorV3(I2C.Port.kOnboard);
		pDH = new PowerDistribution(1, ModuleType.kRev);

		// Reset state machine
		pushCommandTimeStamp = Timer.getFPGATimestamp() - Constants.PUSH_TIME_SECONDS;
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
		currentState = FSMState.START_STATE;

		isShooterSolenoidExtended = false;
		isIntakeMechRetracted = true;
		// Call one tick of update to ensure outputs reflect start state
		update(null, DriveFSMSystem.FSMState.PURE_PURSUIT);
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

		switch (currentState) {
			case START_STATE:
				handleStartState(input);

			case IDLE:
				handleIdleState(input);
				break;

			case FIRING:
				handleFiringState(input);
				break;

			case RETRACTING:
				handleRetractingState(input);
				break;

			case RETRACT_INTAKE_MECH:
				handleRetractIntakeMechState(input);
				break;

			case RELEASE_INTAKE_MECH:
				handleReleaseIntakeMechState(input);
				break;

			case INTAKING:
				handleIntakingState(input);
				break;

			case RELEASING:
				handleReleasingState(input);
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
		if (input == null) {
			if (!isShooterSolenoidExtended
					&& driveState == DriveFSMSystem.FSMState.DEPOSIT_BALL_IDLE) {
				pushCommandTimeStamp = Timer.getFPGATimestamp();

				return FSMState.FIRING;
			} else if (isShooterSolenoidExtended
					&& Timer.getFPGATimestamp() - pushCommandTimeStamp
					> Constants.PUSH_TIME_SECONDS) {
				return FSMState.RETRACTING;
			}
			return FSMState.IDLE;
		}

		switch (currentState) {
			case START_STATE:
				return FSMState.IDLE;

			case IDLE:
				if (!isShooterSolenoidExtended && input.isShooterButtonPressed()) {
					pushCommandTimeStamp = Timer.getFPGATimestamp();

					return FSMState.FIRING;
				} else if (isShooterSolenoidExtended
						&& Timer.getFPGATimestamp() - pushCommandTimeStamp
						> Constants.PUSH_TIME_SECONDS) {
					return FSMState.RETRACTING;
				} else if (input.wasToggleIntakeButtonPressed()) {
					if (isIntakeMechRetracted) {
						return FSMState.RELEASE_INTAKE_MECH;
					} else {
						return FSMState.RETRACT_INTAKE_MECH;
					}
				} else if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKING;
				} else if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.RELEASING;
				} else {
					return FSMState.IDLE;
				}

			case INTAKING:
				if (isShooterSolenoidExtended
						&& Timer.getFPGATimestamp() - pushCommandTimeStamp
						> Constants.PUSH_TIME_SECONDS) {
					return FSMState.RETRACTING;
				} else if (!isShooterSolenoidExtended && input.isShooterButtonPressed()) {
					pushCommandTimeStamp = Timer.getFPGATimestamp();

					return FSMState.FIRING;
				} else if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKING;
				} else {
					return FSMState.IDLE;
				}

			case RELEASING:
				if (isShooterSolenoidExtended
						&& Timer.getFPGATimestamp() - pushCommandTimeStamp
						> Constants.PUSH_TIME_SECONDS) {
					return FSMState.RETRACTING;
				} else if (!isShooterSolenoidExtended && input.isShooterButtonPressed()) {
					pushCommandTimeStamp = Timer.getFPGATimestamp();

					return FSMState.FIRING;
				} else if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.RELEASING;
				} else {
					return FSMState.IDLE;
				}

			case RETRACT_INTAKE_MECH:
				return FSMState.IDLE;

			case RELEASE_INTAKE_MECH:
				return FSMState.IDLE;

			case FIRING:
				if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKING;
				} else if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.RELEASING;
				} else {
					return FSMState.IDLE;
				}

			case RETRACTING:
				if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKING;
				} else if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.RELEASING;
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
		intakeDeploySolenoid.set(DoubleSolenoid.Value.kOff);
		intakeMotor.setVoltage(0);
	}
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartState(TeleopInput input) {
		handleRetractIntakeMechState(input);
		handleRetractingState(input);
	}
	/**
	 * Handle behavior in FIRING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFiringState(TeleopInput input) {
		pushSolenoid.set(DoubleSolenoid.Value.kForward);

		isShooterSolenoidExtended = true;
	}
	/**
	 * Handle behavior in RETRACTING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRetractingState(TeleopInput input) {
		pushSolenoid.set(DoubleSolenoid.Value.kReverse);

		isShooterSolenoidExtended = false;
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
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		pushSolenoid.set(DoubleSolenoid.Value.kOff);
		intakeMotor.setVoltage(
			getBallInMech() == IntakeMechBallStates.NONE
				? -Constants.INTAKE_MOTOR_VOLTAGE : 0);
	}
	/**
	 * Handle behavior in RELEASING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleReleasingState(TeleopInput input) {
		pushSolenoid.set(DoubleSolenoid.Value.kOff);
		intakeMotor.setVoltage(Constants.INTAKE_MOTOR_VOLTAGE);
	}

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
}
