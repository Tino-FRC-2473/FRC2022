package frc.robot.drivemodes;

import frc.robot.Constants;

public class Functions {
    public static double accelerate(double targetPower, double currentPower) {
        double dPower = targetPower - currentPower;
        if (Math.abs(dPower) > Constants.TELEOP_ACCELERATION_MIN) {
            return currentPower + dPower * Constants.TELEOP_ACCELERATION_CONSTANT;
        }else {
            return targetPower;
        }
    }

    public static double calcForwardPower(double joystickInput) {
        return (1 - Math.cos(Math.PI * (Math.abs(joystickInput) / 2.0)));
    }

    public static double calcSteeringPower(double steeringInput) {
        return (-Math.abs(2 * Math.cos(Math.PI * steeringInput / 2.0 + Math.PI / 2.0)) + 1);
    }

    public static double turnInPlace(double joystickInput, double steeringInput) {
        if (Math.abs(steeringInput) > Constants.TELEOP_MIN_TURN_POWER) {
            return -steeringInput;
        } else {
            return 0;
        }
    }
}
