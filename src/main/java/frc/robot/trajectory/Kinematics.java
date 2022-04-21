package frc.robot.trajectory;

// WPILib Imports
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants;

public class Kinematics {

	/**
	 * Calculates the new robot position as if the robot
	 * traveled in a line.
	 * @param gyroAngle current heading of the robot (90 is straight forward)
	 * @param currentEncoderPos current encoder position
	 * @param prevEncoderPos previous encoder position
	 * @param robotPos current robot position
	 * @return new robot position
	 */
	public static Translation2d updateLineOdometry(double gyroAngle,
		double currentEncoderPos, double prevEncoderPos, Translation2d robotPos) {
		// double currentEncoderPos = ((-leftEncoderPos + rightEncoderPos) / 2.0);
		double dEncoder = (currentEncoderPos - prevEncoderPos) / Constants.REVOLUTIONS_PER_INCH;
		double dX = dEncoder * Math.cos(Math.toRadians(gyroAngle));
		double dY = dEncoder * Math.sin(Math.toRadians(gyroAngle));

		return new Translation2d(robotPos.getX() + dX, robotPos.getY() + dY);
	}

	/**
	 * Calculated the new robot position as if the robot
	 * traveled in an arc.
	 * @param gyroAngle current heading of the robot (90 is straight forward)
	 * @param prevGyroAngle previous heading of the robot
	 * @param currentEncoderPos current encoder position
	 * @param prevEncoderPos previous encoder position
	 * @param robotPos current robot position
	 * @return new robot position
	 */
	public static Translation2d updateArcOdometry(double gyroAngle,
		double prevGyroAngle, double currentEncoderPos, double prevEncoderPos,
			Translation2d robotPos) {
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

		return new Translation2d(robotNewXPos, robotNewYPos);
	}

	/**
	 * Calculates the motor powers needed to move to a specific point.
	 * @param gyroHeading heading of the robot (90 is straight forward)
	 * @param robotPos robot position
	 * @param targetPos target position
	 * @param isRobotGoingForward checks whether the robot wants to go forward
	 * or backwards
	 * @return the motor powers to needed to move to the target position
	 */
	public static Translation2d inversekinematics(double gyroHeading, Translation2d robotPos,
		Translation2d targetPos, boolean isRobotGoingForward) {

		//new target position relative to the robot
		Translation2d newtargetPos = new Translation2d(targetPos.getX() - robotPos.getX(),
			targetPos.getY() - robotPos.getY());

		//slope of the line form the robot to the center of the circle that
		//defines the arc the robot will travel
		double mRC = Math.tan(Math.toRadians(gyroHeading));
		if (gyroHeading == 0) {
			mRC = Math.tan(Math.toRadians(Constants.HORIZONTAL_HEADING_CORRECTION_DEG));
		} else if (gyroHeading == 180) {
			mRC = Math.tan(Math.toRadians(180 - Constants.HORIZONTAL_HEADING_CORRECTION_DEG));
		}

		//midpoint between the robot and the target point
		//this is point A
		Translation2d midpoint = new Translation2d(newtargetPos.getX() / 2.0,
			newtargetPos.getY() / 2.0);

		//slope of the line from the robot to the target point
		double mRT;
		if (Math.abs(newtargetPos.getX()) < Constants.ZERO_THRESHOLD) {
			mRT = newtargetPos.getY() / Constants.ZERO_THRESHOLD;
		} else {
			mRT = newtargetPos.getY() / newtargetPos.getX();
		}

		if (mRT != mRT) {
			System.out.println(mRT);
		}
		if (Math.abs(newtargetPos.getX()) < Constants.ZERO_THRESHOLD) {
			mRT = newtargetPos.getY() / Constants.ZERO_THRESHOLD;
		}

		//center of the circle that defines the arc
		Translation2d center = new Translation2d(0, 0);

		//if mRC - mRT is close to 0, center.getX() will have a divide by 0 error
		if (Math.abs(mRC - mRT) < Constants.ZERO_THRESHOLD) {
			center = new Translation2d((newtargetPos.getX() * mRC + newtargetPos.getY() * mRC * mRT)
			/ (2.0 * (mRC - mRT < 0 ? -1 : 1) * Constants.ZERO_THRESHOLD), center.getY());
			//if the heading is one way and the target point is 180 deg
			//in the other direction, set the center of the circle at ~(0, 0)
			if (gyroHeading > 270 && gyroHeading < 90 & newtargetPos.getX() < 0) {
				center = new Translation2d(center.getX(), 0);
			} else if (gyroHeading < 270 && gyroHeading > 90 & newtargetPos.getX() > 0) {
				center = new Translation2d(center.getX(), 0);
			} else {
				center = new Translation2d(center.getX(), -center.getX() / mRC);
			}

		} else {
			center = new Translation2d((newtargetPos.getX() * mRC + newtargetPos.getY() * mRC * mRT)
			/ (2.0 * (mRC - mRT)), center.getY());
			center = new Translation2d(center.getX(), -center.getX() / mRC);
		}

		//radius of the circle
		double radius = getMagnitude(center.getX(), center.getY());

		//distance from the robot to the midpoint
		double distRA = getMagnitude(midpoint.getX(), midpoint.getY());

		//distance from the midpoint to the center of the circle
		double distAC = getMagnitude(midpoint.getX() - center.getX(),
			midpoint.getY() - center.getY());

		double alpha = Math.atan2(distRA, distAC);

		//radius for the inner set of wheels
		double radInner = radius - Constants.TRACKWIDTH_IN / 2.0;
		//radius for the outer set of wheels
		double radOuter = radius + Constants.TRACKWIDTH_IN / 2.0;

		//length of the arc for the inner wheels
		double arcInner = 2 * radInner * alpha;
		//length of the arc for the outer wheels
		double arcOuter = 2 * radOuter * alpha;

		//powers to set the inner and outer set of wheels
		double powerInner = arcInner / arcOuter;
		double powerOuter = 1;

		double targetAngle = Math.toDegrees(Math.atan2(newtargetPos.getY(), newtargetPos.getX()));
		if (targetAngle < 0) {
			targetAngle += 360;
		}

		//check if the point is behind the robot
		//or not in front of it (requires the robot to make too large of an arc)
		// System.out.println("target angle: " + targetAngle);

		//find out whether the left or right side is the inner/outer set of wheels
		if ((targetAngle - gyroHeading > 0 && targetAngle - gyroHeading < 180)
			|| targetAngle - gyroHeading < -180) {
			//left side is the inner side
			if (isRobotGoingForward) {
				return new Translation2d(powerInner, powerOuter);
			} else {
				return new Translation2d(-powerInner, -powerOuter);
			}
		} else {
			//right side is the inner side
			if (isRobotGoingForward) {
				return new Translation2d(powerOuter, powerInner);
			} else {
				return new Translation2d(-powerOuter, -powerInner);
			}
		}
	}

	/**
	 * Calculates the motor powers needed to move to a specific point.
	 * @param gyroHeading heading of the robot (90 is straight forward)
	 * @param robotPos robot position
	 * @param targetPos target position
	 * @return the motor powers to needed to move to the target position
	 */
	public static Translation2d inversekinematics(double gyroHeading, Translation2d robotPos,
		Translation2d targetPos) {

		return inversekinematics(gyroHeading, robotPos, targetPos, true);
	}
	private static double getMagnitude(double a, double b) {
		return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
	}
}
