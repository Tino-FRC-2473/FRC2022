package frc.robot.drive;

import frc.robot.Constants;

public class Functions {
    public static DrivePower accelerate(DrivePower targetPower, DrivePower currentPower) {
        double dLeftPower = targetPower.getLeftPower() - currentPower.getLeftPower();
        double dRightPower = targetPower.getRightPower() - currentPower.getRightPower();
        double newLeftPower;
        double newRightPower;
        if (Math.abs(dLeftPower) > Constants.TELEOP_ACCELERATION_MIN) {
            newLeftPower = currentPower.getLeftPower() + dLeftPower * Constants.TELEOP_ACCELERATION_CONSTANT;
        }else {
            newLeftPower = targetPower.getLeftPower();
        }
        if (Math.abs(dRightPower) > Constants.TELEOP_ACCELERATION_MIN) {
            newRightPower = currentPower.getRightPower() + dRightPower * Constants.TELEOP_ACCELERATION_CONSTANT;
        }else {
            newRightPower = targetPower.getRightPower();
        }
        return new DrivePower(newLeftPower, newRightPower);
    }

    public static double calcForwardPower(double joystickInput) {
        return (1 - Math.cos(Math.PI * (Math.abs(joystickInput) / 2.0)));
    }

    public static double calcSteeringPower(double steeringInput) {
        return (-Math.abs(2 * Math.cos(Math.PI * steeringInput / 2.0 + Math.PI / 2.0)) + 1);
    }

    public static DrivePower turnInPlace(double joystickInput, double steeringInput) {
        if (Math.abs(steeringInput) > Constants.TELEOP_MIN_TURN_POWER) {
            return new DrivePower(-steeringInput, -steeringInput);
        } else {
            return new DrivePower(0, 0);
        }
    }
}
