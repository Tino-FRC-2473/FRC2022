package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;
import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.drive.DriveModes;
import frc.robot.drive.DrivePower;
import frc.robot.drive.Functions;
import frc.robot.trajectory.AutoPaths;
import frc.robot.trajectory.Kinematics;
import frc.robot.trajectory.PurePursuit;
import frc.robot.HardwareMap;
import frc.robot.Constants;
import frc.robot.AutoSelector.DesiredMode;

// Java Imports
import java.util.ArrayList;

public class DriveFSMSystem {
	// FSM state definitions
	public enum FSMState {
		START_STATE,
		FORWARD_STATE_10_IN,
		BACK_TO_TARMAC,
		BACK_TO_HUB,
		TURN_STATE,
		TELEOP_STATE,
		PURE_PURSUIT,
		PURE_PURSUIT_TO_HUB,
		TURN_TO_HUB,
		DEPOSIT_FIRST_BALL_IDLE,
		DEPOSIT_SECOND_BALL_IDLE,
		DEPOSIT_PRELOAD_BALL_IDLE,
		WAIT_TO_RECEIVE_BALLS,
		TURN_TO_TERMINAL,
		HANGAR_TURN_STATE,
		TERMINAL_TURN_STATE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private boolean finishedMovingStraight;
	private boolean finishedTurning;
	private boolean finishedPurePursuitPath;
	private boolean isStateFinished = false;
	private double forwardStateInitialEncoderPos = -1;
	private double gyroAngle = 0;
	private Translation2d robotPosLine = Constants.PP_R3_START_POINT;
	// private Translation2d robotPosLine = new Translation2d(0, 0);
	private double prevEncoderPosLine = 0;
	private double prevEncoderPosArc = 0;
	private Translation2d robotPosArc = Constants.PP_R3_START_POINT;
	// private Translation2d robotPosArc = new Translation2d(0, 0);
	private double prevGyroAngle = 0;
	private double leftPower = 0;
	private double rightPower = 0;
	private double previousEncoderCount = 0;
	private Timer stateTimer;
	private boolean isDrivingForward = true;

	private DesiredMode autoPath;


	private PurePursuit ppController;
	private ArrayList<Translation2d> ballPoints = new ArrayList<>();
	private ArrayList<Translation2d> pointsToHub = new ArrayList<>();


	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private CANSparkMax frontRightMotor;
	private CANSparkMax frontLeftMotor;
	private CANSparkMax backRightMotor;
	private CANSparkMax backLeftMotor;

	private AHRS gyro;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init

		frontRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_RIGHT,
											CANSparkMax.MotorType.kBrushless);
		frontLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_FRONT_LEFT,
											CANSparkMax.MotorType.kBrushless);
		backLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT,
											CANSparkMax.MotorType.kBrushless);
		backRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT,
											CANSparkMax.MotorType.kBrushless);

		ballPoints = AutoPaths.r3BallPath();
		pointsToHub = AutoPaths.r3HubPath();

		ppController = new PurePursuit(ballPoints);

		gyro = new AHRS(SPI.Port.kMXP);

		stateTimer = new Timer();

		// Reset state machine
		reset(null);
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
	 * Reset this system to its intial state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void reset(TeleopInput input) {

		frontRightMotor.getEncoder().setPosition(0);
		frontLeftMotor.getEncoder().setPosition(0);
		backRightMotor.getEncoder().setPosition(0);
		backLeftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();

		finishedMovingStraight = false;
		finishedTurning = false;
		finishedPurePursuitPath = false;

		if (input == null) {
			currentState = FSMState.DEPOSIT_PRELOAD_BALL_IDLE;
		} else {
			currentState = FSMState.TELEOP_STATE;
		}

		stateTimer.reset();
		stateTimer.start();

		// Call one tick of update to ensure outputs reflect start state
		update(input);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		gyroAngle = getHeading();

		updateLineOdometry();
		updateArcOdometry();
		// System.out.println("odo: " + robotPosArc.getX() + " " + robotPosArc.getY());
		// System.out.println("left: " + leftMotor.getEncoder().getPosition());
		// System.out.println("right: " + rightMotor.getEncoder().getPosition());


		// System.out.println("Current State: " + currentState);

		switch (currentState) {
			case START_STATE:
				handleStartState();
				break;

			case TELEOP_STATE:
				handleTeleOpState(input);
				break;

			case FORWARD_STATE_10_IN:
				handleForwardOrBackwardState(input, Constants.RUN_2_LEAVE_TARMAC_DIST);
				break;

			case BACK_TO_TARMAC:
				handleForwardOrBackwardState(input, Constants.RUN_2_BACK_TO_TARMAC_DIST);
				break;

			case BACK_TO_HUB:
				handleForwardOrBackwardState(input, Constants.RUN_2_BACK_TO_HUB_DIST);
				break;

			case TURN_STATE:
				handleTurnState(input, Constants.RUN_1_TURN_TO_HUB_ANGLE,
						Constants.PP_TURN_RUN_TIME_SEC);
				break;

			case PURE_PURSUIT:
				handlePurePursuit(Constants.PP_BALL_MAX_RUN_TIME_SEC);
				break;

			case PURE_PURSUIT_TO_HUB:
				handlePurePursuitBackward(Constants.PP_TO_HUB_MAX_RUN_TIME_SEC);
				break;

			case TURN_TO_HUB:
				handleTurnState(input, Constants.PP_R3_HUB_ANGLE_DEG,
						Constants.PP_TURN_RUN_TIME_SEC);
				break;

			case DEPOSIT_FIRST_BALL_IDLE:
				handleBallDepositIdleState(input);
				break;

			case DEPOSIT_SECOND_BALL_IDLE:
				handleBallDepositIdleState(input);
				break;

			case TURN_TO_TERMINAL:
				handleTurnState(input, Constants.RED_TERMINAL_ANGLE_DEG,
					Constants.PP_TURN_RUN_TIME_SEC);
				break;

			case HANGAR_TURN_STATE:
				handleHangarTurnState(input);
				break;

			case TERMINAL_TURN_STATE:
				handleTerminalTurnState(input);
				break;

			case WAIT_TO_RECEIVE_BALLS:
				handleWaitToReceiveBallsState(input, Constants.PP_TERMINAL_BALL_WAIT_TIME_SEC);
				break;

			case DEPOSIT_PRELOAD_BALL_IDLE:
				handleBallDepositIdleState(input);
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		outputToShuffleboard();
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
			case START_STATE:
				if (input != null) {
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.DEPOSIT_PRELOAD_BALL_IDLE;
				}

			case TELEOP_STATE:
				if (input.getHangarButton()) {
					return FSMState.HANGAR_TURN_STATE;
				} else if (input.getTerminalButton()) {
					return FSMState.TERMINAL_TURN_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}

			case FORWARD_STATE_10_IN:
				if (finishedMovingStraight) {
					finishedMovingStraight = false;
					forwardStateInitialEncoderPos = -1;
					return FSMState.BACK_TO_TARMAC;
				} else {
					return FSMState.FORWARD_STATE_10_IN;
				}

			case BACK_TO_TARMAC:
				if (finishedMovingStraight) {
					finishedMovingStraight = false;
					forwardStateInitialEncoderPos = -1;
					return FSMState.TURN_STATE;
				} else {
					return FSMState.BACK_TO_TARMAC;
				}

			case TURN_STATE:
				if (finishedTurning) {
					finishedTurning = false;
					return FSMState.BACK_TO_HUB;
				} else {
					return FSMState.TURN_STATE;
				}

			case BACK_TO_HUB:
				if (finishedMovingStraight) {
					finishedMovingStraight = false;
					forwardStateInitialEncoderPos = -1;
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.BACK_TO_HUB;
				}

			case PURE_PURSUIT:
				if (finishedPurePursuitPath) {
					finishedPurePursuitPath = false;
					ppController = new PurePursuit(pointsToHub);
					stateTimer.reset();
					return FSMState.TURN_TO_TERMINAL;
				}
				return FSMState.PURE_PURSUIT;

			case PURE_PURSUIT_TO_HUB:
				if (finishedPurePursuitPath) {
					finishedPurePursuitPath = false;
					stateTimer.reset();
					return FSMState.TURN_TO_HUB;
				}
				return FSMState.PURE_PURSUIT_TO_HUB;

			case TURN_TO_HUB:
				if (finishedTurning) {
					finishedTurning = false;
				} else {
					return FSMState.TURN_TO_HUB;
				}
				return FSMState.DEPOSIT_FIRST_BALL_IDLE;

			case DEPOSIT_FIRST_BALL_IDLE:
				if (stateTimer.hasElapsed(Constants.TIME_FOR_AUTO_SHOOT)) {
					stateTimer.reset();
					return FSMState.DEPOSIT_SECOND_BALL_IDLE;
				} else {
					return FSMState.DEPOSIT_FIRST_BALL_IDLE;
				}

			case DEPOSIT_SECOND_BALL_IDLE:
				if (stateTimer.hasElapsed(Constants.TIME_FOR_AUTO_SHOOT)) {
					stateTimer.reset();
					return FSMState.TELEOP_STATE;
				} else {
					return FSMState.DEPOSIT_SECOND_BALL_IDLE;
				}

			case TURN_TO_TERMINAL:
				if (finishedTurning) {
					finishedTurning = false;
					stateTimer.reset();
				} else {
					return FSMState.TURN_TO_TERMINAL;
				}
				return FSMState.WAIT_TO_RECEIVE_BALLS;

			case HANGAR_TURN_STATE:
				if (input.getHangarButton() && Math.abs(input.getLeftJoystickY())
					< Constants.TELEOP_MIN_MOVE_POWER
					&& Math.abs(input.getDrivingJoystickY()) < Constants.TELEOP_MIN_MOVE_POWER) {
					return FSMState.HANGAR_TURN_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}

			case TERMINAL_TURN_STATE:
				if (input.getTerminalButton() && Math.abs(input.getLeftJoystickY())
					< Constants.TELEOP_MIN_MOVE_POWER
					&& Math.abs(input.getDrivingJoystickY()) < Constants.TELEOP_MIN_MOVE_POWER) {
					return FSMState.TERMINAL_TURN_STATE;
				} else {
					return FSMState.TELEOP_STATE;
				}

			case WAIT_TO_RECEIVE_BALLS:
				if (isStateFinished) {
					isStateFinished = false;
					stateTimer.reset();
					return FSMState.PURE_PURSUIT_TO_HUB;
				} else {
					return FSMState.WAIT_TO_RECEIVE_BALLS;
				}

			case DEPOSIT_PRELOAD_BALL_IDLE:
				if (stateTimer.hasElapsed(Constants.TIME_FOR_AUTO_SHOOT)) {
					stateTimer.reset();
					return FSMState.PURE_PURSUIT;
				} else {
					return FSMState.DEPOSIT_PRELOAD_BALL_IDLE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 */
	private void handleStartState() {
		setPowerForAllMotors(0); //start with all motors set to 0
	}
	/**
	* Handle behavior in FORWARD_STATE, or BACKWARD_STATE.
	* @param input Global TeleopInput if robot in teleop mode or null if
	*        the robot is in autonomous mode.
	* @param inches The number of inches to move forward or backward
	*/
	private void handleForwardOrBackwardState(TeleopInput input,
		double inches) {

		double currrentPosTicks = -frontLeftMotor.getEncoder().getPosition();

		if (forwardStateInitialEncoderPos == -1) {
			forwardStateInitialEncoderPos = currrentPosTicks;
		}
		double positionRev = currrentPosTicks - forwardStateInitialEncoderPos;
		double currentPosInches = (positionRev * Math.PI
			* Constants.WHEEL_DIAMETER_INCHES) / Constants.GEAR_RATIO;
		double error = inches - currentPosInches;

		// Checks if either the encoder value is equal to the inches required (reached destination)
		// or checks if the robot has hit a wall (encoder value is not changing and motor power is
		// not zero)
		if ((inches > 0 && error < Constants.ERR_THRESHOLD_STRAIGHT_IN)
			|| (inches < 0 && error > -Constants.ERR_THRESHOLD_STRAIGHT_IN)) {

			finishedMovingStraight = true;
			forwardStateInitialEncoderPos = -1;
			setPowerForAllMotors(0);
			return;
		}

		double speed = Constants.KP_MOVE_STRAIGHT * error;

		if (speed >= Constants.MOTOR_RUN_POWER) {
			setPowerForAllMotors(Constants.MOTOR_RUN_POWER);
			// setPowerForAllMotors(Constants.MOTOR_MAX_RUN_POWER_ACCELERATION
			// 	* (-Math.pow((Constants.MOTOR_MAX_POWER_RATIO_ACCELERATION
			// 	* Math.pow(error - inches / 2.0, 2)) / (inches * inches), 2)
			// 	+ Constants.MOTOR_INITAL_POWER_ACCELERATION));
		} else if (speed <= -Constants.MOTOR_RUN_POWER) {
			setPowerForAllMotors(-Constants.MOTOR_RUN_POWER);
		} else {
			setPowerForAllMotors(speed);
		}

		previousEncoderCount = currrentPosTicks;
		// System.out.println("Previous Encoder Count: " + previousEncoderCount);
	}

	/**
	* Sets power for all motors.
	* @param power The power to set all the motors to
	*/
	private void setPowerForAllMotors(double power) {
		frontLeftMotor.set(-power);
		frontRightMotor.set(power);
		backLeftMotor.set(-power);
		backRightMotor.set(power);

	}

	/**
	* Handle behavior in TURN_STATE.
	* @param input Global TeleopInput if robot in teleop mode or null if
	*        the robot is in autonomous mode.
	* @param degrees The final angle of the robot after the desired turn
	* @param maxRunTime the maximum time the state can run for (in seconds)
	*/
	private void handleTurnState(TeleopInput input, double degrees, double maxRunTime) {

		if (stateTimer.get() > maxRunTime) {
			finishedTurning = true;
			setPowerForAllMotors(0);
			return;
		}
		double error = degrees - getHeading();
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			finishedTurning = true;
			setPowerForAllMotors(0);
			return;
		}
		double power = error / Constants.TURN_ERROR_POWER_RATIO;
		if (Math.abs(power) < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER * (power < 0 ? -1 : 1);
		}

		setPowerForAllMotors(power * Constants.CONSTANT);

	}

	/**
	* Gets the heading from the gyro.
	* @return the gyro heading
	*/
	public double getHeading() {
		// double angle = 90 - gyro.getYaw();
		double angle = Constants.PP_R3_HUB_ANGLE_DEG - gyro.getYaw();
		if (angle < 0) {
			angle += 360;
		}
		if (angle > 360) {
			angle -= 360;
		}
		return angle;
	}

	/**
	* Handle behavior in TELEOP_STATE.
	* @param input Global TeleopInput if robot in teleop mode or null if
	*        the robot is in autonomous mode.
	*/
	private void handleTeleOpState(TeleopInput input) {
		if (input == null) {
			return;
		}

		double rightJoystickY = input.getDrivingJoystickY();
		double steerAngle = input.getSteerAngle();
		double currentLeftPower = frontLeftMotor.get();
		double currentRightPower = frontRightMotor.get();

		if (input.isForwardDrivingButtonPressed()) {
			isDrivingForward = true;
		} else if (input.isBackwardDrivingButtonPressed()) {
			isDrivingForward = false;
		}

		DrivePower targetPower = DriveModes.arcadeDrive(rightJoystickY,
			steerAngle, currentLeftPower,
			currentRightPower, isDrivingForward);

		// DrivePower targetPower = DriveModes.tankDrive(leftJoystickY, rightJoystickY);

		// multiple speed modes
		if (input.getTriggerPressed()) {
			targetPower.scale(Constants.MAX_POWER);
		} else {
			targetPower.scale(Constants.REDUCED_MAX_POWER);
		}

		DrivePower power;

		// acceleration
		power = Functions.accelerate(targetPower, new DrivePower(currentLeftPower,
			currentRightPower));

		// turning in place
		if (Math.abs(rightJoystickY) < Constants.TELEOP_MIN_MOVE_POWER) {
			power = Functions.turnInPlace(rightJoystickY, steerAngle);
		}

		leftPower = power.getLeftPower();
		rightPower = power.getRightPower();

		// frontRightMotor.set(rightPower * Constants.CONSTANT);
		// backRightMotor.set(rightPower * Constants.CONSTANT);
		// frontLeftMotor.set(leftPower * Constants.CONSTANT);
		// backLeftMotor.set(leftPower * Constants.CONSTANT);
		frontRightMotor.set(rightPower);
		backRightMotor.set(rightPower);
		frontLeftMotor.set(leftPower);
		backLeftMotor.set(leftPower);

		System.out.println("left power: " + frontLeftMotor.get());
		System.out.println("right power: " + frontRightMotor.get());
		System.out.println("joystick y-value: " + input.getDrivingJoystickY());
		System.out.println("wheel: " + input.getSteerAngle());

	}

	/**
	* Handle behavior in HANGAR_TURN_STATE.
	* @param input Global TeleopInput if robot in teleop mode or null if
	*        the robot is in autonomous mode.
	*/
	private void handleHangarTurnState(TeleopInput input) {

		double leftJoystickY = input.getLeftJoystickY();
		double rightJoystickY = input.getDrivingJoystickY();

		if (Math.abs(gyroAngle - Constants.HANGAR_TURN_TARGET_ANGLE)
				> Constants.AUTOALIGN_TURN_ERROR
				&& Math.abs(leftJoystickY) < Constants.TELEOP_MIN_MOVE_POWER
				&& Math.abs(rightJoystickY) < Constants.TELEOP_MIN_MOVE_POWER) {

			double error = Constants.HANGAR_TURN_TARGET_ANGLE - gyroAngle;
			double turnPower = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
			if (turnPower < Constants.TELEOP_MIN_TURN_POWER) {
				turnPower = Constants.TELEOP_MIN_TURN_POWER;
			}

			turnPower *= error < 0 && error > -180 ? -1 : 1;

			leftPower = turnPower;
			rightPower = turnPower;

			frontRightMotor.set(rightPower * Constants.CONSTANT);
			backRightMotor.set(rightPower * Constants.CONSTANT);
			frontLeftMotor.set(leftPower * Constants.CONSTANT);
			backLeftMotor.set(leftPower * Constants.CONSTANT);
		}
	}

	/**
	* Handle behavior in TERMINAL_TURN_STATE.
	* @param input Global TeleopInput if robot in teleop mode or null if
	*        the robot is in autonomous mode.
	*/
	private void handleTerminalTurnState(TeleopInput input) {

		double leftJoystickY = input.getLeftJoystickY();
		double rightJoystickY = input.getDrivingJoystickY();

		if (Math.abs(gyroAngle - Constants.RED_TERMINAL_ANGLE_DEG)
				> Constants.AUTOALIGN_TURN_ERROR
				&& Math.abs(leftJoystickY) < Constants.TELEOP_MIN_MOVE_POWER
				&& Math.abs(rightJoystickY) < Constants.TELEOP_MIN_MOVE_POWER) {

			double error = Constants.RED_TERMINAL_ANGLE_DEG - gyroAngle;
			double turnPower = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
			if (turnPower < Constants.TELEOP_MIN_TURN_POWER) {
				turnPower = Constants.TELEOP_MIN_TURN_POWER;
			}

			turnPower *= error < 0 && error > -180 ? -1 : 1;

			leftPower = turnPower;
			rightPower = turnPower;

			frontRightMotor.set(rightPower * Constants.CONSTANT);
			backRightMotor.set(rightPower * Constants.CONSTANT);
			frontLeftMotor.set(leftPower * Constants.CONSTANT);
			backLeftMotor.set(leftPower * Constants.CONSTANT);
		}
	}

	/**
	 * Updates the robot's position assuming the robot moves in an line.
	 * @return the robot's new position
	 */
	public Translation2d updateLineOdometry() {

		double newEncoderPos = ((-frontLeftMotor.getEncoder().getPosition()
			+ frontRightMotor.getEncoder().getPosition()) / 2.0);

		double adjustedAngle = gyroAngle;
		double currentEncoderPos = ((-frontLeftMotor.getEncoder().getPosition()
			+ frontRightMotor.getEncoder().getPosition()) / 2.0);
		double dEncoder = (currentEncoderPos - prevEncoderPosLine) / Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(adjustedAngle));
		double dY = dEncoder * Math.sin(Math.toRadians(adjustedAngle));
		robotPosLine.plus(new Translation2d(dX, dY));

		prevEncoderPosLine = currentEncoderPos;
		return robotPosLine;
	}

	/**
	 * Updates the robot's position assuming the robot moves in an arc.
	 * @return the robot's new position
	 */
	public Translation2d updateArcOdometry() {
		double newEncoderPos = ((-frontLeftMotor.getEncoder().getPosition()
			+ frontRightMotor.getEncoder().getPosition()) / 2.0);
		robotPosArc = Kinematics.updateArcOdometry(gyroAngle, prevGyroAngle,
			newEncoderPos, prevEncoderPosArc, robotPosArc);

		prevGyroAngle = gyroAngle;
		prevEncoderPosArc = newEncoderPos;

		return robotPosArc;
	}

	/**
	 * Gets the robot position from the arc-based odometry calculations.
	 * @return the robot's position
	 */
	public Translation2d getRobotPosArc() {
		return robotPosArc;
	}

	/**
	 * Gets the robot position from the line-based odometry calculations.
	 * @return the robot's position
	 */
	public Translation2d getRobotPosLine() {
		return robotPosLine;
	}

	private void handlePurePursuit(double maxRunTime) {
		if (stateTimer.get() > maxRunTime) {
			finishedPurePursuitPath = true;
			setPowerForAllMotors(0);
			return;
		}
		Translation2d target = ppController.findLookahead(getRobotPosArc(), getHeading());
		if (target == null) {
			finishedPurePursuitPath = true;
			setPowerForAllMotors(0);
			return;

		}

		Translation2d motorSpeeds = Kinematics.inversekinematics(gyroAngle,
			robotPosArc, target, true);
		frontLeftMotor.set(-motorSpeeds.getX() * Constants.PP_MAX_SPEED * Constants.CONSTANT);
		backLeftMotor.set(-motorSpeeds.getX() * Constants.PP_MAX_SPEED * Constants.CONSTANT);
		frontRightMotor.set(motorSpeeds.getY() * Constants.PP_MAX_SPEED * Constants.CONSTANT);
		backRightMotor.set(motorSpeeds.getY() * Constants.PP_MAX_SPEED * Constants.CONSTANT);

	}

	private void handlePurePursuitBackward(double maxRunTime) {
		if (stateTimer.get() > maxRunTime) {
			finishedPurePursuitPath = true;
			setPowerForAllMotors(0);
			return;
		}
		Translation2d target = ppController.findLookahead(getRobotPosArc(), getHeading());
		if (target == null) {
			finishedPurePursuitPath = true;
			setPowerForAllMotors(0);
			return;

		}

		Translation2d motorSpeeds = Kinematics.inversekinematics(gyroAngle,
			robotPosArc, target, false);
		frontLeftMotor.set(-motorSpeeds.getX() * Constants.PP_MAX_SPEED * Constants.CONSTANT);
		backLeftMotor.set(-motorSpeeds.getX() * Constants.PP_MAX_SPEED * Constants.CONSTANT);
		frontRightMotor.set(motorSpeeds.getY() * Constants.PP_MAX_SPEED * Constants.CONSTANT);
		backRightMotor.set(motorSpeeds.getY() * Constants.PP_MAX_SPEED * Constants.CONSTANT);

	}

	private void handleBallDepositIdleState(TeleopInput input) {
		setPowerForAllMotors(0);
	}

	private void handleWaitToReceiveBallsState(TeleopInput input, double waitTime) {
		if (stateTimer.get() > waitTime) {
			isStateFinished = true;
		}
		setPowerForAllMotors(0);
	}

	private void outputToShuffleboard() {
		SmartDashboard.putNumber("Gyro heading", getHeading());
	}

	/**
	 * Sets the autonomous path to run.
	 * @param path the path to run
	 */
	public void setAutoPath(DesiredMode path) {
		autoPath = path;
	}
}
