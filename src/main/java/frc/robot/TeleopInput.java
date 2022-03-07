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
	private Joystick mechJoystick;
	private Joystick steeringWheel;
	private Joystick drivingJoystick;
	private Joystick leftJoystick;


	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		mechJoystick = new Joystick(Constants.MECH_JOYSTICK_PORT);

		steeringWheel = new Joystick(Constants.STEERING_WHEEL_PORT);
		drivingJoystick = new Joystick(Constants.DRIVING_JOYSTICK_PORT);
		leftJoystick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
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
		return leftJoystick.getY();
	}

	/* ------------------------ Mech Joystick ------------------------ */

	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		SmartDashboard.putBoolean("Shooter Button",
			mechJoystick.getRawButton(Constants.SHOOTER_BUTTON));
		return mechJoystick.getRawButton(Constants.SHOOTER_BUTTON);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		SmartDashboard.putBoolean("Intake Button",
			mechJoystick.getRawButton(Constants.INTAKE_BUTTON));
		return mechJoystick.getRawButton(Constants.INTAKE_BUTTON);
	}
	/**
	 * Get the value of the terminal release button.
	 * @return True if button is pressed
	 */
	public boolean isTerminalReleaseButtonPressed() {
		return mechJoystick.getRawButton(Constants.TERMINAL_RELEASE_BUTTON);
	}
	/**
	 * Get the value of the intake toggle button.
	 * @return True if button is pressed
	 */
	public boolean wasToggleIntakeButtonPressed() {
		return mechJoystick.getRawButtonPressed(Constants.TOGGLE_INTAKE_BUTTON);
	}
	/**
	 * Get the value of the ascending button.
	 * @return True if button is pressed
	 */
	public boolean isAscendingButtonPressed() {
		return mechJoystick.getRawButton(Constants.ASCEND_BUTTON);
	}
	/**
	 * Get the value of the descending button.
	 * @return True if button is pressed
	 */
	public boolean isDescendingButtonPressed() {
		return mechJoystick.getRawButton(Constants.DESCEND_BUTTON);
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
		return drivingJoystick.getRawButton(Constants.DRIVING_FORWARD_BUTTON);
	}

	/**
	 * Checks if the backward driving button is pressed.
	 * @return true if the backward driving button is pressed
	 */
	public boolean isBackwardDrivingButtonPressed() {
		return drivingJoystick.getRawButton(Constants.DRIVING_BACKWARDS_BUTTON);
	}

	/**
	 * Checks if the button to turn to hangar on the driving joystick is released.
	 * @return true if the button to turn to hangar is released
	 */
	public boolean getHangarButton() {
		SmartDashboard.putBoolean("Turn to Hangar",
			drivingJoystick.getRawButton(Constants.TURN_TO_HANGAR_BUTTON));
		return drivingJoystick.getRawButton(Constants.TURN_TO_HANGAR_BUTTON);
	}

	/**
	 * Checks if the button to turn to terminal on the driving joystick is pressed.
	 * @return true if the button to turn to terminal is pressed
	 */
	public boolean getTerminalButton() {
		return drivingJoystick.getRawButton(Constants.TURN_TO_TERMINAL_BUTTON);
	}

	/**
	 * Checks if the button to drive to the detected ball is pressed.
	 * @return true if the button is pressed
	 */
	public boolean getCVBallButton() {
		return drivingJoystick.getRawButton(Constants.DRIVING_TO_BALL_BUTTON);
	}

	/* ======================== Private methods ======================== */

}
