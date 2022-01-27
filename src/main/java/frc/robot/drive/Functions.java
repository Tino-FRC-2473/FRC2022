package frc.robot.drive;

import frc.robot.Constants;

public class Functions {

	/**
	 * Calculated the new motor powers by accelerating the current
	 * powers to the desire ones.
	 * @param targetPower the target powers for the motors to be at
	 * @param currentPower the current powers the motors are at
	 * @return the new motor powers
	 */
	public static DrivePower accelerate(DrivePower targetPower, DrivePower currentPower) {
		double dLeftPower = targetPower.getLeftPower() - currentPower.getLeftPower();
		double dRightPower = targetPower.getRightPower() - currentPower.getRightPower();
		double newLeftPower;
		double newRightPower;
		if (Math.abs(dLeftPower) > Constants.TELEOP_ACCELERATION_MIN) {
			newLeftPower = currentPower.getLeftPower()
				+ dLeftPower * Constants.TELEOP_ACCELERATION_CONSTANT;
		} else {
			newLeftPower = targetPower.getLeftPower();
		}
		if (Math.abs(dRightPower) > Constants.TELEOP_ACCELERATION_MIN) {
			newRightPower = currentPower.getRightPower()
				+ dRightPower * Constants.TELEOP_ACCELERATION_CONSTANT;
		} else {
			newRightPower = targetPower.getRightPower();
		}
		return new DrivePower(newLeftPower, newRightPower);
	}

    public static double calcSteeringPower(double steeringInput) {
        //return (-Math.abs(2 * Math.cos(Math.PI * steeringInput / 2.0 + Math.PI / 2.0)) + 1);
        return 1 - Math.abs(2 * steeringInput);
    }

	/**
	 * Calculates the adjusted steering power to set the motor
	 * given the steering wheel input.
	 * @param steeringInput the input from the steering wheel
	 * @return the adjusted steering power
	 */
	public static double calcSteeringPower(double steeringInput) {
		return 1 - Math.abs(2 * steeringInput);
	}

	/**
	 * Determines if the robot should turn in place and calculates the
	 * resulting motor powers.
	 * @param joystickInput the input from the joystick
	 * @param steeringInput the input from the steering wheel
	 * @return the new motor powers
	 */
	public static DrivePower turnInPlace(double joystickInput, double steeringInput) {
		if (Math.abs(steeringInput) > Constants.TELEOP_MIN_TURN_POWER) {
			return new DrivePower(-steeringInput, -steeringInput);
		} else {
			return new DrivePower(0, 0);
		}
	}
}
