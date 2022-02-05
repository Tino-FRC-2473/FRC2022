package frc.robot.systems;

import com.revrobotics.CANSparkMax;

// WPILib Imports

// Third party Hardware Imports
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.wrappers.NeoSparkMaxPid;
import frc.robot.HardwareMap;

public class BallHandlingFSM {
	/* ======================== Constants ======================== */
	// FSM state definitions
	public enum FSMState {
		IDLE,
		FIRING,
		RETRACTING,
		INTAKING,
		RELEASING
	}

	private static final double PUSH_TIME_SECONDS = 3;

	private static final double INTAKE_MOTOR_RPM = 300;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	private Solenoid pushSolenoid;
	private Solenoid pullSolenoid;

	private NeoSparkMaxPid intakeMotor;

	private double pushCommandTime;

	private static final double P = 0;
	private static final double I = 0;
	private static final double D = 0;
	private static final double F = 0;
	private static final double IZ = 0;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public BallHandlingFSM() {
		// Perform hardware init
		pushSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
		HardwareMap.PCM_CHANNEL_PUSH_BOT_SOLENOID);
		pullSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
		HardwareMap.PCM_CHANNEL_PULL_BOT_SOLENOID);

		intakeMotor = new NeoSparkMaxPid(HardwareMap.CAN_ID_SPARK_INTAKE,
						P,
						I,
						D,
						F,
						IZ);

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
		currentState = FSMState.IDLE;

		pushCommandTime = -1;
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
			case IDLE:
				handleIdleState(input);
				break;

			case FIRING:
				handleFiringState(input);
				break;

			case RETRACTING:
				handleRetractingState(input);
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
		if (input == null) {
			return FSMState.IDLE;
		}

		switch (currentState) {
			case IDLE:
				if (pushCommandTime == -1 && input.isShooterButtonPressed()) {
					pushCommandTime = Timer.getFPGATimestamp();

					return FSMState.FIRING;
				} else if (pushCommandTime != -1
					&& Timer.getFPGATimestamp() - pushCommandTime > PUSH_TIME_SECONDS) {
					pushCommandTime = -1;

					return FSMState.RETRACTING;
				} else if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKING;
				} else if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.RELEASING;
				} else {
					return FSMState.IDLE;
				}

			case INTAKING:
				if (pushCommandTime != -1
					&& Timer.getFPGATimestamp() - pushCommandTime > PUSH_TIME_SECONDS) {
					pushCommandTime = -1;

					return FSMState.RETRACTING;
				} else if (input.isIntakeButtonPressed()) {
					return FSMState.INTAKING;
				} else {
					return FSMState.IDLE;
				}

			case RELEASING:
				if (pushCommandTime != -1
					&& Timer.getFPGATimestamp() - pushCommandTime > PUSH_TIME_SECONDS) {
					pushCommandTime = -1;

					return FSMState.RETRACTING;
				} else if (input.isTerminalReleaseButtonPressed()) {
					return FSMState.RELEASING;
				} else {
					return FSMState.IDLE;
				}

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
		pushSolenoid.set(false);
		pullSolenoid.set(false);
		intakeMotor.setVelocity(0);
	}
	/**
	 * Handle behavior in FIRING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleFiringState(TeleopInput input) {
		pushSolenoid.set(true);
		pullSolenoid.set(false);
	}
	/**
	 * Handle behavior in RETRACTING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleRetractingState(TeleopInput input) {
		pushSolenoid.set(false);
		pullSolenoid.set(true);
	}
	/**
	 * Handle behavior in INTAKING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleIntakingState(TeleopInput input) {
		pushSolenoid.set(false);
		pullSolenoid.set(false);
		intakeMotor.setVelocity(INTAKE_MOTOR_RPM);
		System.out.println(intakeMotor.getVelocity());
	}
	/**
	 * Handle behavior in RELEASING.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleReleasingState(TeleopInput input) {
		pushSolenoid.set(false);
		pullSolenoid.set(false);
		intakeMotor.setVelocity(-INTAKE_MOTOR_RPM);
	}

	/**
	 * Get relavent Spark Max instances for simulation.
	 * @return All used Spark Max instances to be tested
	 */
	public CANSparkMax[] getSparkMaxs() {
		return new CANSparkMax[]{
			intakeMotor.getMotor()
		};
	}
}
