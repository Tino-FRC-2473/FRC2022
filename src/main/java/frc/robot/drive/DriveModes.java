package frc.robot.drive;

import frc.robot.Constants;

public class DriveModes {

	/**
	* Calculate the left and right motor powers in an arcade drive setup.
	* @param joystickY joystick input
	* @param steerAngle steering wheel input from -1 to 1
	* @param currentLeftPower current power of the left motor
	* @param currentRightPower current power of the right motor
	* @param isDrivingForward true if the robot is currently in a
	* 							forward driving setup
	* @return return the powers to set the left and right motors
	*/
	public static DrivePower arcadeDrive(double joystickY, double steerAngle,
		double currentLeftPower, double currentRightPower, boolean isDrivingForward) {

		double adjustedInput = Functions.calcForwardPower(joystickY);
		DrivePower adjustedSteering = Functions.calcSteeringPower(steerAngle);

		if (joystickY < 0 && adjustedInput > 0) {
			adjustedInput *= -1;
		}

		double targetLeftPower = 0;
		double targetRightPower = 0;

		// if (steerAngle > 0) {
		// 	targetRightPower = -adjustedInput * adjustedSteering;
		// 	targetLeftPower = adjustedInput * (1 + 2 * Math.abs(steerAngle));
		// } else {
		// 	targetLeftPower = adjustedInput * adjustedSteering;
		// 	targetRightPower = -adjustedInput * (1 + 2 * Math.abs(steerAngle));
		// }

		targetRightPower = -adjustedInput * adjustedSteering.getRightPower();
		targetLeftPower = adjustedInput * adjustedSteering.getLeftPower();

		//checks if the magnitude of the target powers is greater than 1
		if (Math.abs(targetLeftPower) > 1.0) {
			targetLeftPower /= Math.abs(targetLeftPower);
		}
		if (Math.abs(targetRightPower) > 1.0) {
			targetRightPower /= Math.abs(targetRightPower);
		}

		//reversible driving (currently set on buttons 5 and 6)
		if (!isDrivingForward) {
			// if (steerAngle > 0) {
			// 	targetLeftPower = -adjustedInput * adjustedSteering;
			// 	targetRightPower = adjustedInput;
			// } else {
			// 	targetRightPower = adjustedInput * adjustedSteering;
			// 	targetLeftPower = -adjustedInput;
			// }
			targetLeftPower = -adjustedInput * adjustedSteering.getLeftPower();
			targetRightPower = adjustedInput * adjustedSteering.getRightPower();

			//checks if the magnitude of the target powers is greater than 1
			if (Math.abs(targetLeftPower) > 1.0) {
				targetLeftPower /= Math.abs(targetLeftPower);
			}
			if (Math.abs(targetRightPower) > 1.0) {
				targetRightPower /= Math.abs(targetRightPower);
			}
		}

		return new DrivePower(targetLeftPower, targetRightPower);
	}

	/**
	* Calculate the left and right motor powers in a tank drive setup.
	* @param leftJoystickY left joystick input
	* @param rightJoystickY right joystick input
	* @return return the powers to set the left and right motors
	*/
	public static DrivePower tankDrive(double leftJoystickY, double rightJoystickY) {

		double leftAdjustedInput = Functions.calcForwardPower(leftJoystickY);
		double rightAdjustedInput = Functions.calcForwardPower(rightJoystickY);

		//if the left and right joysticks are close to the same value
		//make the power equal (makes driving straight easier)
		if (Math.abs(leftAdjustedInput - rightAdjustedInput)
			< Constants.TANK_DRIVE_STRAIGHT_DRIVE_POWER_DIFF) {
			rightAdjustedInput = leftAdjustedInput;
		}

		if (leftJoystickY < 0 && leftAdjustedInput > 0) {
			leftAdjustedInput *= -1;
		}

		if (rightJoystickY < 0 && rightAdjustedInput > 0) {
			rightAdjustedInput *= -1;
		}

		double targetLeftPower = leftAdjustedInput;
		double targetRightPower = -rightAdjustedInput;

		//check if the resulting power will be too low
		if (Math.abs(targetLeftPower) < Constants.TELEOP_MIN_MOVE_POWER) {
			targetLeftPower = 0;
		}
		if (Math.abs(targetRightPower) < Constants.TELEOP_MIN_MOVE_POWER) {
			targetRightPower = 0;
		}

		return new DrivePower(targetLeftPower, targetRightPower);
	}
}
