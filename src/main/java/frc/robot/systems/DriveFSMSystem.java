package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Translation2d;

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
		DEPOSIT_BALL_IDLE
	}

	/* ======================== Private variables ======================== */
	private FSMState currentState;
	private boolean finishedMovingStraight;
	private boolean finishedTurning;
	private boolean finishedPurePursuitPath;
	private double forwardStateInitialEncoderPos = -1;
	private double gyroAngle = 0;
	private Translation2d robotPosLine = Constants.PP_R1_START_POINT;
	// private Translation2d robotPosLine = new Translation2d(0, 0);
	private double prevEncoderPosLine = 0;
	private double prevEncoderPosArc = 0;
	private Translation2d robotPosArc = Constants.PP_R1_START_POINT;
	// private Translation2d robotPosArc = new Translation2d(0, 0);
	private double prevGyroAngle = 0;
	private double leftPower = 0;
	private double rightPower = 0;
	private double previousEncoderCount = 0;
	private Timer timer;
	private double currentTime = 0;
	private boolean isDrivingForward = true;

	private PurePursuit ppController;
	private ArrayList<Translation2d> ballPoints = new ArrayList<>();
	private ArrayList<Translation2d> pointsToHub = new ArrayList<>();


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

		// ballPoints.add(new Translation2d(-20.5, -60));
		// ballPoints.add(new Translation2d(-26, -151));
		// ballPoints.add(new Translation2d(-80, -88));
		// ballPoints.add(new Translation2d(-40, -90));
		// pointsToHub.add(new Translation2d(-40, -90));
		// pointsToHub.add(new Translation2d(-30, -60));

		ballPoints = AutoPaths.r1BallPath();
		pointsToHub = AutoPaths.r1HubPath();

		ppController = new PurePursuit(ballPoints);

		gyro = new AHRS(SPI.Port.kMXP);

		timer = new Timer();

		// Reset state machine
		// reset();
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
	 * Reset this system to its autonomous state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void resetAutonomous() {

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();

		finishedMovingStraight = false;
		finishedTurning = false;
		finishedPurePursuitPath = false;

		currentState = FSMState.PURE_PURSUIT;

		timer.reset();
		timer.start();

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Reset this system to its teleop state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void resetTeleop() {

		rightMotor.getEncoder().setPosition(0);
		leftMotor.getEncoder().setPosition(0);

		gyro.reset();
		gyro.zeroYaw();

		finishedMovingStraight = false;
		finishedTurning = false;
		finishedPurePursuitPath = false;

		currentState = FSMState.TELEOP_STATE;

		timer.reset();
		timer.start();

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
		double updatedTime = timer.get();
		currentTime = updatedTime;
		gyroAngle = getHeading();
		if (currentTime > 1) {
			System.out.println("gyro: " + gyroAngle);
			timer.reset();
			currentTime = 0;
		}

		updateLineOdometry();
		updateArcOdometry();

		switch (currentState) {
			case START_STATE:
				handleStartState(input);
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
				handleTurnState(input, Constants.PP_R2_HUB_ANGLE_DEG,
						Constants.PP_TURN_RUN_TIME_SEC);
				break;

			case DEPOSIT_BALL_IDLE:
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
					return FSMState.START_STATE;
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
					timer.reset();
					return FSMState.PURE_PURSUIT_TO_HUB;
				}
				return FSMState.PURE_PURSUIT;

			case PURE_PURSUIT_TO_HUB:
				if (finishedPurePursuitPath) {
					finishedPurePursuitPath = false;
					timer.reset();
					return FSMState.TURN_TO_HUB;
				}
				return FSMState.PURE_PURSUIT_TO_HUB;

			case TURN_TO_HUB:
				if (finishedTurning) {
					finishedTurning = false;
				} else {
					return FSMState.TURN_TO_HUB;
				}
				return FSMState.TELEOP_STATE;

			case DEPOSIT_BALL_IDLE:
				if (true) {
					return FSMState.PURE_PURSUIT;
				} else {
					return FSMState.DEPOSIT_BALL_IDLE;
				}

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/* ------------------------ FSM state handlers ------------------------ */
	/**
	 * Handle behavior in START_STATE.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleStartState(TeleopInput input) {
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

		double currrentPosTicks = -leftMotor.getEncoder().getPosition();

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
		System.out.println("Previous Encoder Count: " + previousEncoderCount);
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

		if (timer.get() > maxRunTime) {
			finishedTurning = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		double error = degrees - getHeading();
		if (Math.abs(error) <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
			finishedTurning = true;
			leftMotor.set(0);
			rightMotor.set(0);
			return;
		}
		double power = error / Constants.TURN_ERROR_POWER_RATIO;
		if (Math.abs(power) < Constants.MIN_TURN_POWER) {
			power = Constants.MIN_TURN_POWER * (power < 0 ? -1 : 1);
		}

		leftMotor.set(power);
		rightMotor.set(power);

	}

	/**
	* Gets the heading from the gyro.
	* @return the gyro heading
	*/
	public double getHeading() {
		double angle = 90 - gyro.getYaw();
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

		DrivePower targetPower = DriveModes.arcadedrive(rightJoystickY,
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
			if (Math.abs(gyroAngle - Constants.TERMINAL_TURN_TARGET_ANGLE)
				> Constants.AUTOALIGN_TURN_ERROR
				&& Math.abs(leftJoystickY) < Constants.TELEOP_MIN_MOVE_POWER
				&& Math.abs(rightJoystickY) < Constants.TELEOP_MIN_MOVE_POWER) {

				double error = Constants.TERMINAL_TURN_TARGET_ANGLE - gyroAngle;
				double turnPower = Math.abs(error) / Constants.TURN_ERROR_POWER_RATIO;
				if (turnPower < Constants.TELEOP_MIN_TURN_POWER) {
					turnPower = Constants.TELEOP_MIN_TURN_POWER;
				}

				turnPower *= error < 0 && error > -180 ? -1 : 1;

				leftPower = turnPower;
				rightPower = turnPower;
			}
		}

		rightMotor.set(rightPower);
		leftMotor.set(leftPower);

	}

	/**
	 * Updates the robot's position assuming the robot moves in an line.
	 * @return the robot's new position
	 */
	public Translation2d updateLineOdometry() {

		double newEncoderPos = ((-leftMotor.getEncoder().getPosition()
			+ rightMotor.getEncoder().getPosition()) / 2.0);

		double adjustedAngle = gyroAngle;
		double currentEncoderPos = ((-leftMotor.getEncoder().getPosition()
			+ rightMotor.getEncoder().getPosition()) / 2.0);
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
		if (timer.get() > maxRunTime) {
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
			robotPosArc, target, true);
		leftMotor.set(-motorSpeeds.getX() * Constants.PP_MAX_SPEED);
		rightMotor.set(motorSpeeds.getY() * Constants.PP_MAX_SPEED);

	}

	private void handlePurePursuitBackward(double maxRunTime) {
		if (timer.get() > maxRunTime) {
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
		leftMotor.set(-motorSpeeds.getX() * Constants.PP_MAX_SPEED);
		rightMotor.set(motorSpeeds.getY() * Constants.PP_MAX_SPEED);

	}

	private void handleBallDepositIdleState(TeleopInput input) {

		leftMotor.set(0);
		rightMotor.set(0);
	}
}
