package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
		START_STATE(3),
		FORWARD_STATE_10_IN(3),
		BACK_TO_TARMAC(3),
		BACK_TO_HUB(3),
		TURN_STATE(3),
		TELEOP_STATE(3),
		PURE_PURSUIT(3),
		PURE_PURSUIT_TO_HUB(3),
		TURN_TO_HUB(3),
		DEPOSIT_FIRST_BALL_IDLE(1),
		DEPOSIT_SECOND_BALL_IDLE(2),
		DEPOSIT_PRELOAD_BALL_IDLE(0),
		WAIT_TO_RECEIVE_BALLS(3),
		TURN_TO_TERMINAL(3);

		private final int ballIndex;
		FSMState(int ballIndex) {
			this.ballIndex = ballIndex;
		}

		/**
		 * Returns the index of the ball being shot during Autonomous.
		 * @return the index of the ball being shot during Autonomous.
		 */
		public int getBallIndex() {
			return ballIndex;
		}
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private boolean finishedMovingStraight;
	private boolean finishedTurning;
	private boolean finishedPurePursuitPath;
	private boolean isStateFinished = false;
	private double forwardStateInitialEncoderPos = -1;
	private double gyroAngle = 0;
	private Translation2d robotPosLine = Constants.PP_B3_START_POINT;
	private double prevEncoderPosLine = 0;
	private double prevEncoderPosArc = 0;
	private Translation2d robotPosArc = Constants.PP_B3_START_POINT;
	private double prevGyroAngle = 0;
	private double leftPower = 0;
	private double rightPower = 0;
	private double previousEncoderCount = 0;
	private Timer stateTimer;
	private boolean isDrivingForward = true;

	private double[] cvBallPos = new double[] {0, 0};
	private double startAngle = Constants.PP_B3_HUB_ANGLE_DEG;
	private double hubAngle = Constants.PP_B3_HUB_ANGLE_DEG;
	private double terminalAngle = Constants.BLUE_TERMINAL_ANGLE_DEG;
	private boolean useInvisPoints = true;


	private PurePursuit ppController;
	private ArrayList<Translation2d> ballPoints = new ArrayList<>();
	private ArrayList<Translation2d> pointsToHub = new ArrayList<>();
	private DesiredMode defaultAutoPath = DesiredMode.BLUE_3_BALL;


	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.

	private CANSparkMax rightMotor;
	private CANSparkMax leftMotor;

	private AHRS gyro;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public DriveFSMSystem() {
		// Perform hardware init

		rightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_RIGHT,
											CANSparkMax.MotorType.kBrushless);
		leftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_LEFT,
											CANSparkMax.MotorType.kBrushless);

		ballPoints = AutoPaths.b3BallPath();
		pointsToHub = AutoPaths.b3HubPath();

		ppController = new PurePursuit(ballPoints);
		ppController.addInvisPoints(Constants.PP_R3_NUM_INVIS_POINTS,
			Constants.BLUE_TERMINAL_ANGLE_DEG);

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

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

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

		setAutoPath(defaultAutoPath);
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
				handleTurnState(input, hubAngle,
						Constants.PP_TURN_RUN_TIME_SEC);
				break;

			case DEPOSIT_FIRST_BALL_IDLE:
				handleBallDepositIdleState(input);
				break;

			case DEPOSIT_SECOND_BALL_IDLE:
				handleBallDepositIdleState(input);
				break;

			case TURN_TO_TERMINAL:
				handleTurnState(input, terminalAngle,
					Constants.PP_TURN_RUN_TIME_SEC);
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
				return FSMState.TELEOP_STATE;

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
					stateTimer.reset();
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

			case WAIT_TO_RECEIVE_BALLS:
				if (isStateFinished) {
					isStateFinished = false;
					stateTimer.reset();
					return FSMState.PURE_PURSUIT_TO_HUB;
				} else {
					return FSMState.WAIT_TO_RECEIVE_BALLS;
				}

			case DEPOSIT_PRELOAD_BALL_IDLE:
				if (stateTimer.hasElapsed(Constants.TIME_FOR_INITIAL_AUTO_SHOT)) {
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

		double currentPosTicks = -leftMotor.getEncoder().getPosition();

		if (forwardStateInitialEncoderPos == -1) {
			forwardStateInitialEncoderPos = currentPosTicks;
		}
		double positionRev = currentPosTicks - forwardStateInitialEncoderPos;
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
		} else if (speed <= -Constants.MOTOR_RUN_POWER) {
			setPowerForAllMotors(-Constants.MOTOR_RUN_POWER);
		} else {
			setPowerForAllMotors(speed);
		}

		previousEncoderCount = currentPosTicks;
	}

	/**
	* Sets power for all motors.
	* @param power The power to set all the motors to
	*/
	private void setPowerForAllMotors(double power) {
		leftMotor.set(-power);
		rightMotor.set(power);

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
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		double error = degrees - getHeading();
		if (error > 180) {
			error -= 360;
		}
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			finishedTurning = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		double power = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
		if (power < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER;
		}

		power *= (error < 0 && error > -180) ? -1 : 1;

		leftMotor.set(power);
		rightMotor.set(power);

	}

	/**
	* Gets the heading from the gyro.
	* @return the gyro heading
	*/
	public double getHeading() {
		double angle = startAngle - gyro.getYaw();
		if (angle < 0) {
			angle += 360;
		}
		if (angle > 360) {
			angle -= 360;
		}
		return angle;
	}

	private void handleTeleOpState(TeleopInput input) {
		if (input == null) {
			return;
		}
		double leftJoystickY = input.getLeftJoystickY();
		double rightJoystickY = input.getDrivingJoystickY();
		double steerAngle = input.getSteerAngle();
		double currentLeftPower = leftMotor.get();
		double currentRightPower = rightMotor.get();

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

		if (input.getHangarButton()) {
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
			}
		}

		if (input.getTerminalButton()) {
			if (Math.abs(gyroAngle - terminalAngle)
				> Constants.AUTOALIGN_TURN_ERROR
				&& Math.abs(leftJoystickY) < Constants.TELEOP_MIN_MOVE_POWER
				&& Math.abs(rightJoystickY) < Constants.TELEOP_MIN_MOVE_POWER) {

				double error = terminalAngle - gyroAngle;
				double turnPower = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
				if (turnPower < Constants.TELEOP_MIN_TURN_POWER) {
					turnPower = Constants.TELEOP_MIN_TURN_POWER;
				}

				turnPower *= error < 0 && error > -180 ? -1 : 1;

				leftPower = turnPower;
				rightPower = turnPower;
			}
		}

		if (input.getCVBallButton() && cvBallPos[0] != -1 && cvBallPos[1] != -1) {
			if (cvBallPos[1] < 0) {
				cvBallPos[1] += 360;
			}
			Translation2d cvTargetPos = new Translation2d(cvBallPos[0],
				Rotation2d.fromDegrees(cvBallPos[1]));
			cvTargetPos = cvTargetPos.plus(Constants.LIMELIGHT_POS);
			Translation2d desiredPowers = Kinematics.inversekinematics(0,
				new Translation2d(0, 0), cvTargetPos);
			leftPower = -desiredPowers.getX() * Constants.DETECTED_BALL_MAX_POWER;
			rightPower = desiredPowers.getY() * Constants.DETECTED_BALL_MAX_POWER;
		}

		rightMotor.set(rightPower);
		leftMotor.set(leftPower);

	}

	/**
	 * Updates the robot's position assuming the robot moves in an line.
	 * @return the robot's new position
	 */
	public Translation2d updateLineOdometry() {
		double currentEncoderPos = ((-leftMotor.getEncoder().getPosition()
			+ rightMotor.getEncoder().getPosition()) / 2.0);
		robotPosLine = Kinematics.updateLineOdometry(gyroAngle, currentEncoderPos,
			prevEncoderPosLine, robotPosLine);

		prevEncoderPosLine = currentEncoderPos;
		return robotPosLine;
	}

	/**
	 * Updates the robot's position assuming the robot moves in an arc.
	 * @return the robot's new position
	 */
	public Translation2d updateArcOdometry() {
		double newEncoderPos = ((-leftMotor.getEncoder().getPosition()
			+ rightMotor.getEncoder().getPosition()) / 2.0);
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
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		Translation2d target = ppController.findLookahead(getRobotPosArc(), getHeading());
		if (target == null) {
			finishedPurePursuitPath = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		} else if (useInvisPoints && ppController.isNearEnd(Constants.PP_R3_NUM_INVIS_POINTS)) {
			finishedPurePursuitPath = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}

		Translation2d motorSpeeds = Kinematics.inversekinematics(gyroAngle,
			robotPosArc, target, true);
		if (useInvisPoints && ppController.isNearEnd(Constants.PP_SLOW_DOWN_NUM_POINTS
			+ Constants.PP_R3_NUM_INVIS_POINTS)) {
			leftMotor.set(-motorSpeeds.getX() * Constants.PP_SLOW_DOWN_SPEED);
			rightMotor.set(motorSpeeds.getY() * Constants.PP_SLOW_DOWN_SPEED);
		} else if (!useInvisPoints && ppController.isNearEnd(Constants.PP_SLOW_DOWN_NUM_POINTS)) {
			leftMotor.set(-motorSpeeds.getX() * Constants.PP_SLOW_DOWN_SPEED);
			rightMotor.set(motorSpeeds.getY() * Constants.PP_SLOW_DOWN_SPEED);
		} else {
			leftMotor.set(-motorSpeeds.getX() * Constants.PP_MAX_SPEED);
			rightMotor.set(motorSpeeds.getY() * Constants.PP_MAX_SPEED);
		}

	}

	private void handlePurePursuitBackward(double maxRunTime) {
		if (stateTimer.get() > maxRunTime) {
			finishedPurePursuitPath = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		Translation2d target = ppController.findLookahead(getRobotPosArc(), getHeading());
		if (target == null) {
			finishedPurePursuitPath = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;

		}

		Translation2d motorSpeeds = Kinematics.inversekinematics(gyroAngle,
			robotPosArc, target, false);

		if (ppController.isNearEnd(Constants.PP_SLOW_DOWN_NUM_POINTS)) {
			leftMotor.set(-motorSpeeds.getX() * Constants.PP_SLOW_DOWN_SPEED);
			rightMotor.set(motorSpeeds.getY() * Constants.PP_SLOW_DOWN_SPEED);
		} else {
			leftMotor.set(-motorSpeeds.getX() * Constants.PP_MAX_SPEED);
			rightMotor.set(motorSpeeds.getY() * Constants.PP_MAX_SPEED);
		}
	}

	private void handleBallDepositIdleState(TeleopInput input) {
		leftMotor.set(0);
		rightMotor.set(0);
	}

	private void handleWaitToReceiveBallsState(TeleopInput input, double waitTime) {
		if (stateTimer.get() > waitTime) {
			isStateFinished = true;
		}
		leftMotor.set(0);
		rightMotor.set(0);
	}


	/**
	 * Sets the autonomous path to run.
	 * @param path the path to run
	 */
	public void setAutoPath(DesiredMode path) {
		switch (path) {
			case RED_3_BALL:
				startAngle = Constants.PP_R3_HUB_ANGLE_DEG;
				robotPosArc = Constants.PP_R3_START_POINT;
				ballPoints = AutoPaths.r3BallPath();
				pointsToHub = AutoPaths.r3HubPath();
				isStateFinished = false;
				forwardStateInitialEncoderPos = -1;
				gyroAngle = startAngle;
				prevEncoderPosArc = 0;
				prevGyroAngle = startAngle;
				leftPower = 0;
				rightPower = 0;
				previousEncoderCount = 0;
				isDrivingForward = true;
				useInvisPoints = true;
				ppController = new PurePursuit(ballPoints);
				ppController.addInvisPoints(Constants.PP_R3_NUM_INVIS_POINTS,
					Constants.RED_TERMINAL_ANGLE_DEG);
				hubAngle = Constants.PP_R3_HUB_ANGLE_DEG;
				terminalAngle = Constants.RED_TERMINAL_ANGLE_DEG;
				break;
			case RED_2_BALL:
				startAngle = Constants.PP_R2_HUB_ANGLE_DEG;
				robotPosArc = Constants.PP_R2_START_POINT;
				ballPoints = AutoPaths.r2BallPath();
				pointsToHub = AutoPaths.r2HubPath();
				isStateFinished = false;
				forwardStateInitialEncoderPos = -1;
				gyroAngle = startAngle;
				prevEncoderPosArc = 0;
				prevGyroAngle = startAngle;
				leftPower = 0;
				rightPower = 0;
				previousEncoderCount = 0;
				isDrivingForward = true;
				useInvisPoints = true;
				ppController = new PurePursuit(ballPoints);
				ppController.addInvisPoints(Constants.PP_R3_NUM_INVIS_POINTS,
					Constants.RED_TERMINAL_ANGLE_DEG);
				hubAngle = Constants.PP_R3_HUB_ANGLE_DEG;
				terminalAngle = Constants.RED_TERMINAL_ANGLE_DEG;
				currentState = FSMState.PURE_PURSUIT;
				break;
			case RED_1_BALL:
				startAngle = Constants.PP_R3_HUB_ANGLE_DEG;
				robotPosArc = Constants.PP_R1_START_POINT;
				ballPoints = AutoPaths.r1BallPath();
				pointsToHub = AutoPaths.r1HubPath();
				isStateFinished = false;
				forwardStateInitialEncoderPos = -1;
				gyroAngle = startAngle;
				prevEncoderPosArc = 0;
				prevGyroAngle = startAngle;
				leftPower = 0;
				rightPower = 0;
				previousEncoderCount = 0;
				isDrivingForward = true;
				useInvisPoints = false;
				ppController = new PurePursuit(ballPoints);
				hubAngle = Constants.PP_R3_HUB_ANGLE_DEG;
				terminalAngle = Constants.RED_TERMINAL_ANGLE_DEG;
				break;
			case BLUE_3_BALL:
				startAngle = Constants.PP_B3_HUB_ANGLE_DEG;
				robotPosArc = Constants.PP_B3_START_POINT;
				ballPoints = AutoPaths.b3BallPath();
				pointsToHub = AutoPaths.b3HubPath();
				isStateFinished = false;
				forwardStateInitialEncoderPos = -1;
				gyroAngle = startAngle;
				prevEncoderPosArc = 0;
				prevGyroAngle = startAngle;
				leftPower = 0;
				rightPower = 0;
				previousEncoderCount = 0;
				isDrivingForward = true;
				useInvisPoints = true;
				ppController = new PurePursuit(ballPoints);
				ppController.addInvisPoints(Constants.PP_R3_NUM_INVIS_POINTS,
					Constants.BLUE_TERMINAL_ANGLE_DEG);
				hubAngle = Constants.PP_B3_HUB_ANGLE_DEG;
				terminalAngle = Constants.BLUE_TERMINAL_ANGLE_DEG;
				break;
			case BLUE_2_BALL:
				startAngle = Constants.PP_B2_HUB_ANGLE_DEG;
				robotPosArc = Constants.PP_B2_START_POINT;
				ballPoints = AutoPaths.b2BallPath();
				pointsToHub = AutoPaths.b2HubPath();
				isStateFinished = false;
				forwardStateInitialEncoderPos = -1;
				gyroAngle = startAngle;
				prevEncoderPosArc = 0;
				prevGyroAngle = startAngle;
				leftPower = 0;
				rightPower = 0;
				previousEncoderCount = 0;
				isDrivingForward = true;
				useInvisPoints = true;
				ppController = new PurePursuit(ballPoints);
				ppController.addInvisPoints(Constants.PP_R3_NUM_INVIS_POINTS,
					Constants.BLUE_TERMINAL_ANGLE_DEG);
				hubAngle = Constants.PP_B3_HUB_ANGLE_DEG;
				terminalAngle = Constants.BLUE_TERMINAL_ANGLE_DEG;
				currentState = FSMState.PURE_PURSUIT;
				break;
			case BLUE_1_BALL:
				startAngle = Constants.PP_B3_HUB_ANGLE_DEG;
				robotPosArc = Constants.PP_B1_START_POINT;
				ballPoints = AutoPaths.b1BallPath();
				pointsToHub = AutoPaths.b1HubPath();
				isStateFinished = false;
				forwardStateInitialEncoderPos = -1;
				gyroAngle = startAngle;
				prevEncoderPosArc = 0;
				prevGyroAngle = startAngle;
				leftPower = 0;
				rightPower = 0;
				previousEncoderCount = 0;
				isDrivingForward = true;
				useInvisPoints = false;
				ppController = new PurePursuit(ballPoints);
				hubAngle = Constants.PP_B3_HUB_ANGLE_DEG;
				terminalAngle = Constants.BLUE_TERMINAL_ANGLE_DEG;
				break;
			case LEAVE_TARMAC:
				startAngle = 90.0;
				robotPosArc = new Translation2d(0, 0);
				ballPoints = AutoPaths.leaveTarmacBallPath();
				pointsToHub = AutoPaths.leaveTarmacHubPath();
				isStateFinished = false;
				forwardStateInitialEncoderPos = -1;
				gyroAngle = startAngle;
				prevEncoderPosArc = 0;
				prevGyroAngle = startAngle;
				leftPower = 0;
				rightPower = 0;
				previousEncoderCount = 0;
				isDrivingForward = true;
				useInvisPoints = false;
				ppController = new PurePursuit(ballPoints);
				hubAngle = Constants.PP_R3_HUB_ANGLE_DEG;
				terminalAngle = Constants.RED_TERMINAL_ANGLE_DEG;
				break;
			default:
				break;
		}
	}

	/**
	 * Sets the detected ball position obtained by the limelight camera.
	 * @param pos the detected ball's position
	 */
	public void setCVBallPos(double[] pos) {
		cvBallPos = pos;
	}
}
