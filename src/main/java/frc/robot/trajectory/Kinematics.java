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

    public static Point inversekinematics(double gyroHeading, Point robotPos, Point targetPos) {
        Point newtargetPos = new Point(targetPos.x - robotPos.x, targetPos.y - robotPos.y);

        double m_RC = Math.tan(Math.toRadians(gyroHeading));
        if (gyroHeading == 0) m_RC = Math.tan(Math.toRadians(1));
        else if (gyroHeading == 180) m_RC = Math.tan(Math.toRadians(179));

        Point midpoint = new Point(newtargetPos.x / 2.0, newtargetPos.y / 2.0);
        //need to make sure newTargetPos.x does not equal 0 (or isn't close to 0)
        double m_RT;
        if (Math.abs(newtargetPos.x) < 0.01) m_RT = newtargetPos.y / 0.01;
        else m_RT = newtargetPos.y / newtargetPos.x;
        
        if (m_RT != m_RT) System.out.println(m_RT);
        if (Math.abs(newtargetPos.x) < 0.01) m_RT = newtargetPos.y / 0.01;
        double x_c;
        double y_c;
        //if m_RC - m_RT is close to 0, x_c will have a divide by 0 error
        if (Math.abs(m_RC - m_RT) < 0.01) {
            x_c = (newtargetPos.x * m_RC + newtargetPos.y * m_RC * m_RT) / (2.0 * (m_RC - m_RT < 0 ? -1 : 1) * 0.01);
            //if the heading is one way and the target point is 180 deg
            //in the other direction, set the center of the circle at ~(0, 0)
            if (gyroHeading > 270 && gyroHeading < 90 & newtargetPos.x < 0) {
                y_c = 0;
            } else if (gyroHeading < 270 && gyroHeading > 90 & newtargetPos.x > 0) {
                y_c = 0;
            } else {
                y_c = -x_c / m_RC;
            }

        } else {
            x_c = (newtargetPos.x * m_RC + newtargetPos.y * m_RC * m_RT) / (2.0 * (m_RC - m_RT));
            y_c = -x_c / m_RC;
        }

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
        
        //find out whether the left or right side is the inner/outer set of wheels
        double targetAngle = Math.atan2(newtargetPos.y, newtargetPos.x);
        if(targetAngle - gyroHeading > 0 || targetAngle - gyroHeading < -180) {
            //left side is the inner side
            return new Point(p_i, p_o);
        }else {
            //right side is the inner side
            return new Point(p_o, p_i);
        }

    }
    private static double getMagnitude(double a, double b) {
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }
}