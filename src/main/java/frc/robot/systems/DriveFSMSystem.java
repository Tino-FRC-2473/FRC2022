package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.hal.SimEnum;

// Third party Hardware Imports
import com.revrobotics.CANSparkMax;

import org.ejml.dense.row.linsol.qr.AdjLinearSolverQr_DDRM;

import com.kauailabs.navx.frc.AHRS;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.drive.DriveModes;
import frc.robot.drive.DrivePower;
import frc.robot.drive.Functions;
import frc.robot.HardwareMap;
import frc.robot.Constants;

public class DriveFSMSystem {
    // FSM state definitions
    public enum FSMState {
        START_STATE,
        FORWARD_STATE_10_IN,
        TURN_STATE,
        TELEOP_STATE
    }

    /* ======================== Private variables ======================== */
    private FSMState currentState;
    private boolean finishedMovingStraight;
    private boolean finishedTurning;
    private double forwardStateInitialEncoderPos = -1;
    // private double rawGyroAngle = 0;
    private double robotXPosLine = 0;
    private double robotYPosLine = 0;
    private double prevEncoderPosLine = 0;
    private double prevEncoderPosArc = 0;
    private double robotXPosArc = 0;
    private double robotYPosArc = 0;
    private double prevGyroAngle = 0;
    private double leftPower = 0;
    private double rightPower = 0;
    private Timer timer;
    private double currentTime = 0;
    private boolean isDrivingForward = true;

    // Hardware devices should be owned by one and only one system. They must
    // be private to their owner system and may not be used elsewhere.

    private CANSparkMax frontRightMotor;
    private CANSparkMax backRightMotor;
    private CANSparkMax frontLeftMotor;
    private CANSparkMax backLeftMotor;

    // private AHRS gyro;

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
        backRightMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_RIGHT,
                                            CANSparkMax.MotorType.kBrushless);
        backLeftMotor = new CANSparkMax(HardwareMap.CAN_ID_SPARK_DRIVE_BACK_LEFT,
                                            CANSparkMax.MotorType.kBrushless);

        // gyro = new AHRS(SPI.Port.kMXP);

        timer = new Timer();

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

        frontRightMotor.getEncoder().setPosition(0);
        frontLeftMotor.getEncoder().setPosition(0);
        backRightMotor.getEncoder().setPosition(0);
        backLeftMotor.getEncoder().setPosition(0);

        // gyro.reset();
        // gyro.zeroYaw();

        finishedMovingStraight = false;
        finishedTurning = false;

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
        //System.out.println("DTime: " + (updatedTime - currentTime));
        currentTime = updatedTime;
        // rawGyroAngle = gyro.getAngle();
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
                handleForwardOrBackwardState(input, 30);
                break;

            case TURN_STATE:
                handleTurnState(input, 90); // test 90 degrees
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
                    return FSMState.TELEOP_STATE;
                } else {
                    return FSMState.FORWARD_STATE_10_IN;
                }

            case TURN_STATE:
                if (finishedTurning) {
                    finishedTurning = false;
                    return FSMState.TELEOP_STATE;
                } else {
                    return FSMState.TURN_STATE;
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
    * @param initialEncoderPos The encoder position of the front left motor
    * when the state/handler method was first initiated
    */
	private void handleForwardOrBackwardState(TeleopInput input,
	double inches) {
		double currrentPosTicks = -frontLeftMotor.getEncoder().getPosition();
		//System.out.println("currrentPosTicks: " + currrentPosTicks);
		// printing as 0
		if (forwardStateInitialEncoderPos == -1) {
			forwardStateInitialEncoderPos = currrentPosTicks;
		}
		// double positionRev = frontLeftMotor.getEncoder().getPosition() - forwardStateInitialEncoderPos;
		double positionRev = currrentPosTicks - forwardStateInitialEncoderPos;
		double currentPosInches = (positionRev * Math.PI * Constants.WHEEL_DIAMETER_INCHES) / Constants.GEAR_RATIO;
		double error = inches - currentPosInches;
		//System.out.println("Error: " + error);
		// Error is yeilding a negative number. About -16.8 almost every time. Sometimes
		// it's -14.2ish
		if (error < Constants.ERR_THRESHOLD_STRAIGHT_IN) {
			//System.out.println("im here");
			finishedMovingStraight = true;
            setPowerForAllMotors(0);
            return;
		}
		// another version of KP_MOVE_STRAIGHT which is dependent on the inches moved
		// double speedMultipler = 0.1; 
		// speed multipler if it is dependent on the inches 
		// double speedMultipler = inches / 100;
		
		double speed = Constants.KP_MOVE_STRAIGHT * error;
		//System.out.println("speed: " + speed);
		// double speed = speedMultipler * error;

		if (speed >= 0.1) {
			// make this 0.7ish if this is too fast
			setPowerForAllMotors(0.1);
		} else if (speed <= -0.1) {
			// goes in here everytime (wheels moving backwards)
			setPowerForAllMotors(-0.1);
		} else {
			setPowerForAllMotors(speed);
		}
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
    */
    private void handleTurnState(TeleopInput input, double degrees) {
        double error = degrees - getHeading();
        if (error <= Constants.TURN_ERROR_THRESHOLD_DEGREE) {
            finishedTurning = true;
            return;
        }
        double power = error / Constants.TURN_ERROR_POWER_RATIO;
        if (Math.abs(power) < Constants.MIN_TURN_POWER) {
            power = Constants.MIN_TURN_POWER * power < 0 ? -1 : 1;
        }

        // frontLeftMotor.set(power);
        // frontRightMotor.set(-power);
        // backLeftMotor.set(power);
        // backRightMotor.set(-power);
    }

    // /**
    // * Gets the heading from the gyro.
    // * @return the gyro heading
    // */
    // private double getHeading() {
    //     return -Math.IEEEremainder(gyro.getAngle(), 360);
    // }

    private void handleTeleOpState(TeleopInput input) {
        if (input == null) {
            return;
        }

        double joystickY = input.getDrivingJoystickY();
        double steerAngle = input.getSteerAngle();
        double currentLeftPower = frontLeftMotor.get();
        double currentRightPower = frontRightMotor.get();

        if(input.isForwardDrivingButtonPressed()) {
            isDrivingForward = true;
        }else if(input.isBackwardDrivingButtonPressed()) {
            isDrivingForward = false;
        }

        DrivePower targetPower = DriveModes.arcadedrive(joystickY, steerAngle, currentLeftPower, 
        currentRightPower, isDrivingForward);

        //multiple speed modes
        if (input.getTriggerPressed()) {
            targetPower.scale(Constants.MAX_POWER);
        } else {
            targetPower.scale(Constants.REDUCED_MAX_POWER);
        }

        DrivePower power;

        //acceleration
        power = Functions.accelerate(targetPower, new DrivePower(currentLeftPower, currentRightPower));

        //turning in place
        if (Math.abs(joystickY) < Constants.TELEOP_MIN_MOVE_POWER) {
            power = Functions.turnInPlace(joystickY, steerAngle);
        }

        leftPower = power.getLeftPower();
        rightPower = power.getRightPower();

        System.out.println("Ecoder left: " + frontLeftMotor.getEncoder().getPosition());
        System.out.println("Encoder right: " + frontRightMotor.getEncoder().getPosition());

        frontRightMotor.set(rightPower);
        frontLeftMotor.set(leftPower);
        backRightMotor.set(rightPower);
        backLeftMotor.set(leftPower);

        //check if voltage can be negative
        // frontRightMotor.setVoltage(rightPower * 12);
        // frontLeftMotor.setVoltage(leftPower * 12);
        // backRightMotor.setVoltage(rightPower * 12);
        // backLeftMotor.setVoltage(leftPower * 12);
        
    }

    private double limitPower(double number) {
        if (number > 1) {
            return 1;
        }
        if (number < -1) {
            return -1;
        }
        return number;
    }

    private void updateLineOdometry() {
        double adjustedAngle = gyroAngle;
        double currentEncoderPos = ((-frontLeftMotor.getEncoder().getPosition()
            + frontRightMotor.getEncoder().getPosition()) / 2.0);
        double dEncoder = (currentEncoderPos - prevEncoderPosLine) / Constants.REVOLUTIONS_PER_INCH;
        double dX = dEncoder * Math.cos(Math.toRadians(adjustedAngle));
        double dY = dEncoder * Math.sin(Math.toRadians(adjustedAngle));
        robotXPosLine += dX;
        robotYPosLine += dY;

        // prevEncoderPosLine = currentEncoderPos;
        // //System.out.println("Raw Encoder Value: " + currentEncoderPos);
        // //System.out.println("Line: (" + robotXPosLine + ", " + robotYPosLine + ")");
    }

    private void updateArcOdometry() {
        double adjustedAngle = gyroAngle;
        double theta = Math.abs(adjustedAngle - prevGyroAngle);
        double currentEncoderPos = ((-frontLeftMotor.getEncoder().getPosition()
            + frontRightMotor.getEncoder().getPosition()) / 2.0);
        double arcLength = (currentEncoderPos - prevEncoderPosArc) / Constants.REVOLUTIONS_PER_INCH;
        if (Math.abs(theta) < Constants.ODOMETRY_MIN_THETA) {
            theta = Constants.ODOMETRY_MIN_THETA;
        }
        double radius = 180 * arcLength / (Math.PI * theta);
        double alpha = prevGyroAngle - 90;
        double circleX = robotXPosArc + radius * Math.cos(Math.toRadians(alpha));
        double circleY = robotYPosArc + radius * Math.sin(Math.toRadians(alpha));
        double beta = alpha + 180 - theta;
        robotXPosArc = circleX + radius * Math.cos(Math.toRadians(beta));
        robotYPosArc = circleY + radius * Math.sin(Math.toRadians(beta));

        // prevGyroAngle = adjustedAngle;
        // prevEncoderPosArc = currentEncoderPos;
        // //System.out.println("Arc: (" + robotXPosArc + ", " + robotYPosArc + ")");

    }
}