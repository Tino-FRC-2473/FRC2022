package frc.robot;

/**
 * HardwareMap provides a centralized spot for constants related to the hardware
 * configuration of the robot.
 */
public final class HardwareMap {
	// ID numbers for devices on the CAN bus
	public static final int CAN_ID_SPARK_DRIVE_FRONT_RIGHT = 1;
	public static final int CAN_ID_SPARK_DRIVE_BACK_RIGHT = 2;
	public static final int CAN_ID_SPARK_DRIVE_FRONT_LEFT = 3;
	public static final int CAN_ID_SPARK_DRIVE_BACK_LEFT = 4;
	public static final int CAN_ID_SPARK_SHOOTER = 10;
	public static final int CAN_ID_SPARK_GRABBER = 11;
	public static final int CAN_ID_SPARK_INTAKE = 9;
	public static final int CAN_ID_SPARK_DRIVE_LEFT = 6;
	public static final int CAN_ID_SPARK_DRIVE_RIGHT = 5;

	// Pneumatics channel numbers
	public static final int PCM_CHANNEL_PUSH_BOT_SOLENOID = 0;
	public static final int PCM_CHANNEL_PULL_BOT_SOLENOID = 1;
}
