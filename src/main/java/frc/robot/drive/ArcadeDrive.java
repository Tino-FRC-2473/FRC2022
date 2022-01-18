package frc.robot.drive;

public class ArcadeDrive {
    public static DrivePower drive(double joystickY, double steerAngle, 
    double currentLeftPower, double currentRightPower, boolean isDrivingForward) {

        double adjustedInput = Functions.calcForwardPower(joystickY);
        double adjustedSteering = Functions.calcSteeringPower(steerAngle);

        if (joystickY < 0 && adjustedInput > 0) {
            adjustedInput *= -1;
        }

        double targetLeftPower = 0;
        double targetRightPower = 0;

        if (steerAngle > 0) {
            targetRightPower = -adjustedInput * adjustedSteering;
            targetLeftPower = adjustedInput;
        } else {
            targetLeftPower = adjustedInput * adjustedSteering;
            targetRightPower = -adjustedInput;
        }

        //reversible driving (currently set on buttons 5 and 6)
        if(!isDrivingForward) {
            if (steerAngle > 0) {
                targetLeftPower = -adjustedInput * adjustedSteering;
                targetRightPower = adjustedInput;
            } else {
                targetRightPower = adjustedInput * adjustedSteering;
                targetLeftPower = -adjustedInput;
            }
        }

        return new DrivePower(targetLeftPower, targetRightPower);
    }
}
