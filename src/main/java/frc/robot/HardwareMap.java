package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_GRABBER = 1;
	public static final int CAN_ID_SPARK_INTAKE = 2;
	public static final int CAN_ID_SPARK_DRIVE_LEFT = 3;
	public static final int CAN_ID_SPARK_DRIVE_RIGHT = 4;

	// Pneumatics channel numbers
	public static final int PCM_CHANNEL_SHOOTER_SOLENOID_EXTEND = 1;
	public static final int PCM_CHANNEL_SHOOTER_SOLENOID_EXTEND_RELEASE = 0;
	public static final int PCM_CHANNEL_SHOOTER_SOLENOID_RETRACT = 4;
	public static final int PCM_CHANNEL_SHOOTER_SOLENOID_RETRACT_RELEASE = 5;
	public static final int PCM_CHANNEL_INTAKE_RETRACT_SOLENOID = 2;
	public static final int PCM_CHANNEL_INTAKE_RELEASE_SOLENOID = 3;
}
