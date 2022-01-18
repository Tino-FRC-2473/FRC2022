package frc.robot.trajectory;

import frc.robot.Constants;

public class Kinematics {

    private Point updateLineOdometry(double gyroAngle, double leftEncoderPos, 
    double rightEncoderPos, double prevEncoderPos, Point robotPos) {
        double currentEncoderPos = ((-leftEncoderPos
            + rightEncoderPos) / 2.0);
        double dEncoder = (currentEncoderPos - prevEncoderPos) / Constants.REVOLUTIONS_PER_INCH;
        double dX = dEncoder * Math.cos(Math.toRadians(gyroAngle));
        double dY = dEncoder * Math.sin(Math.toRadians(gyroAngle));
        return new Point(robotPos.x + dX, robotPos.y + dY);

        //System.out.println("Raw Encoder Value: " + currentEncoderPos);
        //System.out.println("Line: (" + robotXPosLine + ", " + robotYPosLine + ")");
    }

    private Point updateArcOdometry(double gyroAngle, double prevGyroAngle, double leftEncoderPos, 
    double rightEncoderPos, double prevEncoderPos, Point robotPos) {
        double theta = Math.abs(gyroAngle - prevGyroAngle);
        double currentEncoderPos = ((-leftEncoderPos + rightEncoderPos) / 2.0);
        double arcLength = (currentEncoderPos - prevEncoderPos) / Constants.REVOLUTIONS_PER_INCH;
        if (Math.abs(theta) < Constants.ODOMETRY_MIN_THETA) {
            theta = Constants.ODOMETRY_MIN_THETA;
        }
        double radius = 180 * arcLength / (Math.PI * theta);
        double alpha = prevGyroAngle - 90;
        double circleX = robotPos.x + radius * Math.cos(Math.toRadians(alpha));
        double circleY = robotPos.y + radius * Math.sin(Math.toRadians(alpha));
        double beta = alpha + 180 - theta;
        double robotNewXPos = circleX + radius * Math.cos(Math.toRadians(beta));
        double robotNewYPos = circleY + radius * Math.sin(Math.toRadians(beta));

        return new Point(robotNewXPos, robotNewYPos);
        //System.out.println("Arc: (" + robotXPosArc + ", " + robotYPosArc + ")");

    }
}