package frc.robot;

import com.ctre.phoenix.ButtonMonitor;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;



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
		return leftJoystick.getY();
	}
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(1);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(2);
	}

	/* ------------------------ Right Joystick ------------------------ */

	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/* ------------------------ Wheel ------------------------ */
	/**
	 * Get Angle of the steering Wheel from -1 to 1.
	 * @return Angle
	 */
	public double getSteerAngle() {
		return steeringWheel.getX();
	}

	/* ------------------------ Driving Joystick ------------------------ */
	/**
	 * Get Y value of Driving Joystick.
	 * @return Y-Axis value
	 */
	public double getDrivingJoystickY() {
		return drivingJoystick.getY();
	}

	/**
	 * Get if the Driving Joystick's trigger button is being pressed.
	 * @return Trigger button's state
	 */
	public boolean getTriggerPressed() {
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

	/* ======================== Private methods ======================== */

}

