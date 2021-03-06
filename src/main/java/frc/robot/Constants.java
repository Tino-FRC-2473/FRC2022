package frc.robot;

// WPILib Imports
import edu.wpi.first.math.geometry.Translation2d;

public class Constants {
	//Drive Constants
	public static final double KP_MOVE_STRAIGHT = 0.05;
	public static final double PROPORTION_MAX_POWER = 0.5;
	public static final double WHEEL_DIAMETER_INCHES = 4.0; //7.65
	public static final double ERR_THRESHOLD_STRAIGHT_IN = 0.1;
	public static final double TELEOP_ANGLE_POWER_RATIO = 90.0;
	public static final double MAX_POWER = 1;
	public static final double REDUCED_MAX_POWER = 0.75;
	public static final double TELEOP_MIN_TURN_POWER = 0.03;
	public static final double TELEOP_MIN_MOVE_POWER = 0.02;
	public static final double JOYSTICK_INPUT_ADJUSTMENT = 2.0;
	public static final double TURN_ERROR_POWER_RATIO = 360;
	public static final double MIN_TURN_POWER = 0.125;
	public static final double TURN_ERROR_THRESHOLD_DEGREE = 1.0;
	public static final double TELEOP_ACCELERATION_CONSTANT = 0.05;
	public static final double TELEOP_ACCELERATION_MIN = 0.1;
	public static final double TURNING_IN_PLACE_THRESHOLD = 0.05;
	public static final double ENCODER_CONSTANT = 1.0814;
	public static final double COUNTS_PER_MOTOR_REVOLUTION = 42;
	public static final double GEAR_RATIO = 8.0; //26.0 * 4.67 / 12.0;
	public static final double REVOLUTIONS_PER_INCH
		= GEAR_RATIO / (Math.PI * WHEEL_DIAMETER_INCHES * ENCODER_CONSTANT);
	public static final double ODOMETRY_MIN_THETA = 1.0;
	public static final double MOTOR_RUN_POWER = 0.1;
	public static final double MOTOR_MAX_RUN_POWER_ACCELERATION = 0.35;
	public static final double MOTOR_INITAL_POWER_ACCELERATION = 0.6;
	public static final double MOTOR_MAX_POWER_RATIO_ACCELERATION = 2.8;
	public static final double TANK_DRIVE_STRAIGHT_DRIVE_POWER_DIFF = 0.3;
	public static final double TRACKWIDTH_IN = 20.72;
	public static final double AUTOALIGN_TURN_ERROR = 2;
	public static final double AUTOALIGN_TURN_SPEED = 0.2;
	public static final double HANGAR_TURN_TARGET_ANGLE = 180;
	public static final double RED_TERMINAL_ANGLE_DEG = 43.752;
	public static final double BLUE_TERMINAL_ANGLE_DEG = 223.752;
	public static final double TIME_FOR_INITIAL_AUTO_SHOT = 0.3;


	//CV Ball Detection Constants
	public static final double DETECTED_BALL_MAX_POWER = 0.2;
	public static final Translation2d LIMELIGHT_POS = new Translation2d(12.0, 0.0);

	//Path constants
	// For Run 1 the start angle is 26.73
	public static final double RUN_1_LEAVE_TARMAC_DIST = 42.686;
	public static final double RUN_1_BACK_TO_TARMAC_DIST = -104.186;
	public static final double RUN_1_TURN_TO_HUB_ANGLE = 90;
	public static final double RUN_1_BACK_TO_HUB_DIST = -4.950;

	// For Run 2 the start angle is 90
	public static final double RUN_2_LEAVE_TARMAC_DIST = 55.699;
	public static final double RUN_2_BACK_TO_TARMAC_DIST = -76.050;
	public static final double RUN_2_TURN_TO_HUB_ANGLE = 240.319;
	public static final double RUN_2_BACK_TO_HUB_DIST = -9.737;

	// For Run 3 the start angle is 315
	public static final double RUN_3_LEAVE_TARMAC_DIST = 55.699;
	public static final double RUN_3_BACK_TO_TARMAC_DIST = -73.195;
	public static final double RUN_3_TURN_TO_HUB_ANGLE = 339.125;
	public static final double RUN_3_BACK_TO_HUB_DIST = -49.306;

	//Teleop Input Constants

	//Mech Joystick
	public static final int MECH_JOYSTICK_PORT = 0;
	public static final int SHOOTER_BUTTON = 1;
	public static final int INTAKE_BUTTON = 2;
	public static final int TERMINAL_RELEASE_BUTTON = 2;
	public static final int ASCEND_BUTTON = 4;
	public static final int DESCEND_BUTTON = 5;
	public static final int TOGGLE_INTAKE_BUTTON = 7;

	//Left Joystick
	public static final int LEFT_JOYSTICK_PORT = 1;

	//Steering Wheel
	public static final int STEERING_WHEEL_PORT = 2;

	//Driving Joystick
	public static final int DRIVING_JOYSTICK_PORT = 3;
	public static final int TURN_TO_TERMINAL_BUTTON = 2;
	public static final int TURN_TO_HANGAR_BUTTON = 3;
	public static final int DRIVING_FORWARD_BUTTON = 4;
	public static final int DRIVING_BACKWARDS_BUTTON = 5;
	public static final int DRIVING_TO_BALL_BUTTON = 6;

	//Pure Pursuit Constants
	public static final double MAX_IN_TO_POINT = 48;
	public static final double PP_MAX_SPEED = 0.8;
	public static final double PP_SLOW_DOWN_SPEED = 0.3;
	public static final double PP_BALL_MAX_RUN_TIME_SEC = 6.0;
	public static final double PP_TO_HUB_MAX_RUN_TIME_SEC = 6.0;
	public static final double PP_TURN_RUN_TIME_SEC = 1.0;
	public static final double PP_TERMINAL_BALL_WAIT_TIME_SEC = 3.5;
	public static final int PP_SLOW_DOWN_NUM_POINTS = 8;
	public static final int PP_R3_NUM_INVIS_POINTS = 4;

	//Pure Pursuit Start Points and Angles
	public static final Translation2d PP_R2_START_POINT = new Translation2d(89.687, 0);
	public static final double PP_R2_HUB_ANGLE_DEG = 1.500;

	public static final Translation2d PP_R1_START_POINT = new Translation2d(48.365, -18.775);

	public static final Translation2d PP_R3_START_POINT = new Translation2d(48.365, -18.775);
	public static final double PP_R3_HUB_ANGLE_DEG = 339.0;

	public static final Translation2d PP_B2_START_POINT = new Translation2d(-89.687, 0);
	public static final double PP_B2_HUB_ANGLE_DEG = 178.500;

	public static final Translation2d PP_B1_START_POINT = new Translation2d(-48.365, 18.775);

	public static final Translation2d PP_B3_START_POINT = new Translation2d(-48.365, 18.775);
	public static final double PP_B3_HUB_ANGLE_DEG = 159.0;

	//Inverse Kinematics Constants
	public static final double HORIZONTAL_HEADING_CORRECTION_DEG = 1;
	public static final double ZERO_THRESHOLD = 0.01;

	//Intake Shooter Constants
	public static final int MAX_NUMBER_OF_BALLS = 3;
	public static final double TIME_FOR_PISTON_EXTENSION = 0.5;
	public static final double TIME_FOR_PISTON_RETRACTION = 0.05;
	public static final double TIME_TO_DEPRESSURIZATION = 0.1;
	public static final double TIME_FOR_FULL_SHOT = TIME_FOR_PISTON_EXTENSION
													+ TIME_TO_DEPRESSURIZATION;
	public static final double TIME_FOR_FULL_RETRACTION = TIME_FOR_PISTON_RETRACTION
														+ TIME_TO_DEPRESSURIZATION;
	public static final double INTAKE_MOTOR_VOLTAGE = 5;
	public static final int BALL_PROXIMITY_THRESHOLD = 73;
	public static final double TIME_FOR_AUTO_SHOOT = TIME_FOR_FULL_SHOT
													+ TIME_FOR_FULL_RETRACTION
													+ 0.75;
	public static final int FORWARD_INTAKE_BUTTON = 3;
}
