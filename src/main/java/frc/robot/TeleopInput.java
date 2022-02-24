package frc.robot;


// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;
	private Joystick steeringWheel;
	private Joystick drivingJoystick;


	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
		rightJoystick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);

		steeringWheel = new Joystick(Constants.STEERING_WHEEL_PORT);
		drivingJoystick = new Joystick(Constants.DRIVING_JOYSTICK_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */

	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickY() {
		SmartDashboard.putNumber("Left Joystick", leftJoystick.getY());
		return leftJoystick.getY();
	}
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		SmartDashboard.putBoolean("Shooter Button",
			leftJoystick.getRawButton(Constants.SHOOTER_BUTTON));
		return leftJoystick.getRawButton(Constants.SHOOTER_BUTTON);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		SmartDashboard.putBoolean("Intake Button",
			leftJoystick.getRawButton(Constants.INTAKE_BUTTON));
		return leftJoystick.getRawButton(Constants.INTAKE_BUTTON);
	}
	/**
	 * Get the value of the terminal release button.
	 * @return True if button is pressed
	 */
	public boolean isTerminalReleaseButtonPressed() {
		SmartDashboard.putBoolean("Terminal Release Button",
			leftJoystick.getRawButton(Constants.TERMINAL_RELEASE_BUTTON));
		return leftJoystick.getRawButton(Constants.TERMINAL_RELEASE_BUTTON);
	}
	/**
	 * Get the value of the ascending button.
	 * @return True if button is pressed
	 */
	public boolean isAscendingButtonPressed() {
		SmartDashboard.putBoolean("Ascending Button",
			rightJoystick.getRawButton(1));
		return rightJoystick.getRawButton(1);
	}
	/**
	 * Get the value of the descending button.
	 * @return True if button is pressed
	 */
	public boolean isDescendingButtonPressed() {
		SmartDashboard.putBoolean("Descending Button",
			rightJoystick.getRawButton(2));
		return rightJoystick.getRawButton(2);
	}

	/* ------------------------ Right Joystick ------------------------ */

	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		SmartDashboard.putNumber("Right Joystick", rightJoystick.getY());
		return rightJoystick.getY();
	}

	/* ------------------------ Wheel ------------------------ */
	/**
	 * Get Angle of the steering Wheel from -1 to 1.
	 * @return Angle
	 */
	public double getSteerAngle() {
		SmartDashboard.putNumber("Wheel", steeringWheel.getX());
		return steeringWheel.getX();
	}

	/* ------------------------ Driving Joystick ------------------------ */
	/**
	 * Get Y value of Driving Joystick.
	 * @return Y-Axis value
	 */
	public double getDrivingJoystickY() {
		SmartDashboard.putNumber("Driving Joystick", drivingJoystick.getY());
		return drivingJoystick.getY();
	}

	/**
	 * Get if the Driving Joystick's trigger button is being pressed.
	 * @return Trigger button's state
	 */
	public boolean getTriggerPressed() {
		SmartDashboard.putBoolean("Driving Trigger", drivingJoystick.getTrigger());
		return drivingJoystick.getTrigger();
	}

	/**
	 * Checks if the forward driving button is pressed.
	 * @return true if the forward driving button is pressed
	 */
	public boolean isForwardDrivingButtonPressed() {
		SmartDashboard.putBoolean("Driving Forward",
			drivingJoystick.getRawButton(Constants.DRIVING_FORWARD_BUTTON));
		return drivingJoystick.getRawButton(Constants.DRIVING_FORWARD_BUTTON);
	}

	/**
	 * Checks if the backward driving button is pressed.
	 * @return true if the backward driving button is pressed
	 */
	public boolean isBackwardDrivingButtonPressed() {
		SmartDashboard.putBoolean("Driving Backward",
			drivingJoystick.getRawButton(Constants.DRIVING_BACKWARDS_BUTTON));
		return drivingJoystick.getRawButton(Constants.DRIVING_BACKWARDS_BUTTON);
	}

	/**
	 * Checks if the top button of the driving joystick is released.
	 * @return true if the top driving button is released
	 */
	public boolean getTopPressed() {
		SmartDashboard.putBoolean("Turn to Hangar",
			drivingJoystick.getRawButton(Constants.TURN_TO_HANGAR_BUTTON));
		return drivingJoystick.getRawButton(Constants.TURN_TO_HANGAR_BUTTON);
	}

	/* ======================== Private methods ======================== */

}
