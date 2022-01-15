package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
    /* ======================== Constants ======================== */
    private static final int LEFT_JOYSTICK_PORT = 0;
    private static final int RIGHT_JOYSTICK_PORT = 1;
    private static final int WHEEL_PORT = 2;
    private static final int DRIVING_JOYSTICK_PORT = 3;

    /* ======================== Private variables ======================== */
    // Input objects
    private Joystick leftJoystick;
    private Joystick rightJoystick;
    private Joystick wheel;
    private Joystick drivingJoystick;

    /* ======================== Constructor ======================== */
    /**
     * Create a TeleopInput and register input devices. Note that while inputs
     * are registered at robot initialization, valid values will not be provided
     * by WPILib until teleop mode.
     */
    public TeleopInput() {
        leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);
        rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);

        wheel = new Joystick(WHEEL_PORT);
        drivingJoystick = new Joystick(DRIVING_JOYSTICK_PORT);
    }

    /* ======================== Public methods ======================== */
    // Getter methods for fetch input values should be defined here.
    // Method names should be descriptive of the behavior, so the
    // control mapping is hidden from other classes.

    /* ------------------------ Left Joystick ------------------------ */
    /**
     * Get X axis of Left Joystick.
     * @return Axis value
     */
    public double getLeftJoystickX() {
        return leftJoystick.getX();
    }
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
     * Get X axis of Right Joystick.
     * @return Axis value
     */
    public double getRightJoystickX() {
        return rightJoystick.getX();
    }
    /**
     * Get Y axis of Right Joystick.
     * @return Axis value
     */
    public double getRightJoystickY() {
        return rightJoystick.getY();
    }

	/**
	 * Get if the Right Joystick's trigger is pressed
	 * @return Trigger Pressed value
	 */
	public boolean getRightTriggerPressed(){
		return rightJoystick.getTriggerPressed();
	}

	/**
	 * Get if the Right Joystick's trigger is released
	 * @return Trigger Released value
	 */
	public boolean getRightTriggerReleased(){
		return rightJoystick.getTriggerReleased();
	}

	/* ------------------------ Wheel ------------------------ */
	/**
	 * Get Angle of the steering Wheel.
	 * @return Angle
	 */
	public double getSteerAngle() {
		return wheel.getX();
	}

    /**
     * Get if the Right Joystick's trigger is released
     * @return Trigger Released value
     */
    public boolean getRightTriggerReleased(){
        return rightJoystick.getTriggerReleased();
    }

    /* ------------------------ Wheel ------------------------ */
    /**
     * Get Angle of the steering Wheel.
     * @return Angle
     */
    public double getSteerAngle() {
        return wheel.getX();
    }

    /* ------------------------ Driving Joystick ------------------------ */
    /**
     * Get Y value of Driving Joystick.
     * @return Y-Axis value
     */
    public double getDrivingJoystickY() {
        return drivingJoystick.getY();
    }

    /* ======================== Private methods ======================== */

}

