package frc.robot.trajectory;

import frc.robot.Constants;

public class Kinematics {

    public Point updateLineOdometry(double gyroAngle, double leftEncoderPos, 
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

    public Point updateArcOdometry(double gyroAngle, double prevGyroAngle, double leftEncoderPos, 
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

    public void inversekinematics(double gyroHeading, Point robotPos, Point targetPos) {
        Point newtargetPos = new Point(targetPos.x - robotPos.x, targetPos.y - robotPos.y);
        double m_RC = Math.tan(gyroHeading);
        Point midpoint = new Point(newtargetPos.x / 2.0, newtargetPos.y / 2.0);
        //need to make sure newTargetPos.x does not equal 0 (or isn't close to 0)
        double m_RT = newtargetPos.y / newtargetPos.x;
        //need to make sure m_RT isn't close to m_RC
        double x_c = (newtargetPos.x * m_RC + newtargetPos.y * m_RC * m_RT) / (2.0 * (m_RC - m_RT));
        //m_RC cannot be 0
        double y_c = -x_c / m_RC;
        double radius = getMagnitude(x_c, y_c);
        double RA = getMagnitude(midpoint.x, midpoint.y);
        double AC = getMagnitude(midpoint.x - x_c, midpoint.y - y_c);
        double alpha = Math.atan2(RA, AC);
        double radius_i = radius - Constants.TRACKWIDTH_IN / 2.0;
        double radius_o = radius + Constants.TRACKWIDTH_IN / 2.0;
        double s_i = 2 * radius_i * alpha;
        double s_o = 2 * radius_o * alpha;
        //powers to set the inner and outer set of wheels
        double p_i = s_i / s_o;
        double p_o = 1;

    }

    private double getMagnitude(double a, double b) {
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }
}