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

	/**
	 * Calculates the adjusted power to set the motor given the joystick input.
	 * @param joystickInput the joystick input
	 * @return the adjusted motor power
	 */
	public static double calcForwardPower(double joystickInput) {
		return (1 - Math.cos(Math.PI * joystickInput / 2.0));
	}

		/**
	 * Calculates the adjusted steering power to set the motor
	 * given the steering wheel input.
	 * @param steeringInput the input from the steering wheel
	 * @return the adjusted steering power
	 */
	public static DrivePower calcSteeringPower(double steeringInput) {
		if (steeringInput > 0) {
			return new DrivePower(1 + 1 * Math.abs(steeringInput), 1 - 1 * Math.abs(steeringInput));
		} else {
			return new DrivePower(1 - 1 * Math.abs(steeringInput), 1 + 1 * Math.abs(steeringInput));
		}
		// return new DrivePower(
		// 	1 + Math.signum(steeringInput) * (1 - Math.cos(Math.PI * steeringInput)),
		// 	1 - Math.signum(steeringInput) * (1 - Math.cos(Math.PI * steeringInput))
		// );
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
			return new DrivePower(-Math.signum(steeringInput) * Math.pow(steeringInput, 2),
				-Math.signum(steeringInput) * Math.pow(steeringInput, 2));
		} else {
			return new DrivePower(0, 0);
		}
	}
}
