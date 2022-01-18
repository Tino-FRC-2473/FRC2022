package frc.robot.drive;

public class DrivePower {

    private double leftPower;
    private double rightPower;

    public DrivePower(double leftPower, double rightPower) {
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }

    public DrivePower scale(double scalar) {
        leftPower *= scalar;
        rightPower *= scalar;
        return this;
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public void setLeftPower(double leftPower) {
        this.leftPower = leftPower;
    }

    public void setRightPower(double rightPower) {
        this.rightPower = rightPower;
    }
}
