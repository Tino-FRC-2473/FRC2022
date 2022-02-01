package frc.robot.trajectory;

import frc.robot.Constants;

public class Kinematics {

	public static Point updateLineOdometry(double gyroAngle, double currentEncoderPos, double prevEncoderPos, Point robotPos) {
		// double currentEncoderPos = ((-leftEncoderPos + rightEncoderPos) / 2.0);
		double dEncoder = (currentEncoderPos - prevEncoderPos) / Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngle));
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngle));

		return new Point(robotPos.getX() + dX, robotPos.getY() + dY);
	}

	public static Point updateArcOdometry(double gyroAngle, double prevGyroAngle, double currentEncoderPos, double prevEncoderPos, Point robotPos) {
		double theta = Math.abs(gyroAngle - prevGyroAngle);
		double arcLength = (currentEncoderPos - prevEncoderPos) / Constants.REVOLUTIONS_PER_INCH;
		if (Math.abs(theta) < Constants.ODOMETRY_MIN_THETA) {
			theta = Constants.ODOMETRY_MIN_THETA;
		}
		double radius = 180 * arcLength / (Math.PI * theta);
		double alpha = prevGyroAngle - 90;
		double circleX = robotPos.getX() + radius * Math.cos(Math.toRadians(alpha));
		double circleY = robotPos.getY() + radius * Math.sin(Math.toRadians(alpha));
		double beta = alpha + 180 - theta;
		double robotNewXPos = circleX + radius * Math.cos(Math.toRadians(beta));
		double robotNewYPos = circleY + radius * Math.sin(Math.toRadians(beta));
		
		return new Point(robotNewXPos, robotNewYPos);
	}

	public static Point inversekinematics(double gyroHeading, Point robotPos, Point targetPos) {
		Point newtargetPos = new Point(targetPos.getX() - robotPos.getX(), targetPos.getY() - robotPos.getY());

		double mRC = Math.tan(Math.toRadians(gyroHeading));
		if (gyroHeading == 0) {
			mRC = Math.tan(Math.toRadians(1));
		} else if (gyroHeading == 180) {
			mRC = Math.tan(Math.toRadians(179));
		}

		Point midpoint = new Point(newtargetPos.getX() / 2.0, newtargetPos.getY() / 2.0);
		//need to make sure newtargetPos.getX() does not equal 0 (or isn't close to 0)
		double mRT;
		if (Math.abs(newtargetPos.getX()) < 0.01) {
			mRT = newtargetPos.getY() / 0.01;
		} else {
			mRT = newtargetPos.getY() / newtargetPos.getX();
		}

		if (mRT != mRT) System.out.println(mRT);
		if (Math.abs(newtargetPos.getX()) < 0.01) mRT = newtargetPos.getY() / 0.01;
		Point center = new Point(0, 0);

		//if mRC - mRT is close to 0, center.getX() will have a divide by 0 error
		if (Math.abs(mRC - mRT) < 0.01) {
			center.setX((newtargetPos.getX() * mRC + newtargetPos.getY() * mRC * mRT)
				/ (2.0 * (mRC - mRT < 0 ? -1 : 1) * 0.01));
			//if the heading is one way and the target point is 180 deg
			//in the other direction, set the center of the circle at ~(0, 0)
			if (gyroHeading > 270 && gyroHeading < 90 & newtargetPos.getX() < 0) {
				center.setY(0);
			} else if (gyroHeading < 270 && gyroHeading > 90 & newtargetPos.getX() > 0) {
				center.setY(0);
			} else {
				center.setY(-center.getX() / mRC);
			}

		} else {
			center.setX((newtargetPos.getX() * mRC + newtargetPos.getY() * mRC * mRT)
				/ (2.0 * (mRC - mRT)));
			center.setY(-center.getX() / mRC);
		}

		double radius = getMagnitude(center.getX(), center.getY());

		double RA = getMagnitude(midpoint.getX(), midpoint.getY());
		double AC = getMagnitude(midpoint.getX() - center.getX(),
			midpoint.getY() - center.getY());

		double alpha = Math.atan2(RA, AC);

		double radius_i = radius - Constants.TRACKWIDTH_IN / 2.0;
		double radius_o = radius + Constants.TRACKWIDTH_IN / 2.0;

		double s_i = 2 * radius_i * alpha;
		double s_o = 2 * radius_o * alpha;

		//powers to set the inner and outer set of wheels
		double p_i = s_i / s_o;
		double p_o = 1;

		//find out whether the left or right side is the inner/outer set of wheels
		double targetAngle = Math.atan2(newtargetPos.getY(), newtargetPos.getX());
		if (targetAngle - gyroHeading > 0 || targetAngle - gyroHeading < -180) {
			//left side is the inner side
			return new Point(p_i, p_o);
		} else {
			//right side is the inner side
			return new Point(p_o, p_i);
		}

	}
	private static double getMagnitude(double a, double b) {
		return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
	}
}
