package frc.robot.drive;

public class DriveModes {
    public static DrivePower arcadedrive(double joystickY, double steerAngle, 
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

    public static DrivePower tankDrive(double leftJoystickY, double rightJoystickY){
        
        double leftAdjustedInput = Functions.calcForwardPower(leftJoystickY);
        double rightAdjustedInput = Functions.calcForwardPower(rightJoystickY);

        if (leftJoystickY < 0 && leftAdjustedInput > 0) {
            leftAdjustedInput *= -1;
        }

        if (rightJoystickY < 0 && rightAdjustedInput > 0) {
            rightAdjustedInput *= -1;
        }

        double targetLeftPower = leftAdjustedInput;
        double targetRightPower = -rightAdjustedInput;
        
        if (targetLeftPower + targetRightPower < 0.05) {
            targetRightPower = -leftAdjustedInput;
        }

        return new DrivePower(targetLeftPower, targetRightPower);
    }
}
