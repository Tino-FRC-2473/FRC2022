package frc.robot;

public class Constants {
    //Drive Constants
    public static final double KP_MOVE_STRAIGHT = 0.05;
	public static final double PROPORTION_MAX_POWER = 0.5;
    public static final double WHEEL_DIAMETER_INCHES = 6.00;
    public static final double ERR_THRESHOLD_STRAIGHT_IN = 0.1;
    public static final double TELEOP_ANGLE_POWER_RATIO = 90.0;
    public static final double MAX_POWER = 1;
    public static final double REDUCED_MAX_POWER = 0.2;
    public static final double TELEOP_MIN_TURN_POWER = 0.05;
    public static final double TELEOP_MIN_MOVE_POWER = 0.05;
    public static final double JOYSTICK_INPUT_ADJUSTMENT = 2.0;
    public static final double TURN_ERROR_POWER_RATIO = 360;
    public static final double MIN_TURN_POWER = 0.1;
    public static final double TURN_ERROR_THRESHOLD_DEGREE = 1.0;
    public static final double TELEOP_ACCELERATION_CONSTANT = 0.05;
    public static final double TELEOP_ACCELERATION_MIN = 0.1;
    public static final double COUNTS_PER_MOTOR_REVOLUTION = 42;
    public static final double GEAR_RATIO = 26.0 * 4.67 / 12.0;
    public static final double REVOLUTIONS_PER_INCH
        = GEAR_RATIO / (Math.PI * WHEEL_DIAMETER_INCHES);
    public static final double ODOMETRY_MIN_THETA = 1.0;
    public static final float MOTOR_RUN_POWER = 0.1f;


}
