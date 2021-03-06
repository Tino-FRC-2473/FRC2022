package frc.robot.trajectory;

import edu.wpi.first.math.geometry.Rotation2d;
// WPILib Imports
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;

import frc.robot.Constants;

public class PurePursuit {

	private ArrayList<Translation2d> keyPoints = new ArrayList<>();
	private ArrayList<Translation2d> pathPoints = new ArrayList<>();
	private int lastClosestPointIndex = 0;
	// lookahead point is this many points ahead
	private static final int LOOKAHEAD_DISTACE = 8;

	// in inches
	private static final double SPACING = 6.0;

	/**
	 * Create PurePursuit controller and path of points to follow.
	 * @param initialPoints ArrayList containing key points that form outline of path.
	 */
	public PurePursuit(ArrayList<Translation2d> initialPoints) {
		keyPoints = initialPoints;
		pointInjection();
	}

	private void pointInjection() {
		for (int i = 1; i < keyPoints.size(); i++) {
			Translation2d startPoint = keyPoints.get(i - 1);
			Translation2d endPoint = keyPoints.get(i);

			Translation2d startToEnd = new Translation2d(endPoint.getX() - startPoint.getX(),
				endPoint.getY() - startPoint.getY());

			double numPoints = Math.ceil(Math.abs(startToEnd.getNorm()) / SPACING);

			startToEnd = startToEnd.div(startToEnd.getNorm()).times(SPACING);

			for (int j = 0; j <= (int) numPoints; j++) {
				Translation2d segmentToNextPt = startToEnd.times(j);

				Translation2d toInject = startPoint.plus(segmentToNextPt);
				pathPoints.add(toInject);
			}
		}
	}

	private Translation2d findClosestPoint(Translation2d currentPos) {
		// temporary list of distances from robot to each path point
		ArrayList<Double> tempDistances = new ArrayList<>();

		// store distances from robot to each path point in temp list
		for (int i = lastClosestPointIndex; i < lastClosestPointIndex + LOOKAHEAD_DISTACE; i++) {
			if (i >= pathPoints.size()) {
				break;
			}
			double distance = currentPos.getDistance(pathPoints.get(i));
			tempDistances.add(distance);
		}

		/* find the point closest to the robot (and if 2+ points are the same distance away,
		choose the point that is further along the path) */
		double closestDistance = Double.MAX_VALUE;
		int indexOfMin = 0;
		for (int i = 0; i < tempDistances.size(); i++) {
			if (tempDistances.get(i) <= closestDistance && i >= indexOfMin) {
				closestDistance = tempDistances.get(i);
				indexOfMin = i;
			}
		}

		/* since we're only looking at points ahead of the robot, we have to add index of
		closest point to index of previous closest point (to get the index of the closest
		point in the overall list) */
		lastClosestPointIndex += indexOfMin;

		/* make sure the index we chose is within bounds of the list of path points
		(and not the last point) */
		if (lastClosestPointIndex < pathPoints.size() - 1) {
			Translation2d closestPoint = pathPoints.get(lastClosestPointIndex);
			System.out.println("closest point: " + closestPoint);
			return closestPoint;
		}

		// if the point is the last point (or OOB) return null
		return null;
	}

	/**
	 * Find lookahead point for robot to drive to.
	 * @param robotPos Current robot position
	 * @param robotHeading Current robot angle (heading)
	 * @return the new lookahead point for robot to target.
	 */
	public Translation2d findLookahead(Translation2d robotPos, double robotHeading) {
		// update index of closest point
		findClosestPoint(robotPos);

		// if the closest point is the last point (or OOB) return null
		if (findClosestPoint(robotPos) == null) {
			return null;
		}

		// prevent OOB errors
		int lookaheadPointIndex = lastClosestPointIndex;
		for (int i = lastClosestPointIndex + 1;
			i <= lastClosestPointIndex + LOOKAHEAD_DISTACE; i++) {

			if (i >= pathPoints.size()) {
				break;
			}
			double dist = pathPoints.get(i).getDistance(robotPos);
			if (dist < Constants.MAX_IN_TO_POINT) {
				lookaheadPointIndex = i;
			}
		}
		Translation2d lookaheadPoint = lookaheadPointIndex < pathPoints.size()
			? pathPoints.get(lookaheadPointIndex) : pathPoints.get(pathPoints.size() - 1);
		return lookaheadPoint;
	}

	/**
	 * Checks if the robot is near the end of the path.
	 * @param numPoints
	 * @return Whether the robot is within a certain
	 * amount of points to the end points
	 */
	public boolean isNearEnd(double numPoints) {
		System.out.println("cutoff: " + (pathPoints.size() - 1 - numPoints));
		System.out.println("closest index: " + lastClosestPointIndex);
		return (pathPoints.size() - 1 - numPoints) <= lastClosestPointIndex;
	}

	/**
	 * Adds extra points at the end of the path to help with heading control.
	 * @param numPoints number of points to add at the end
	 * @param angle angle at which to add the points
	 */
	public void addInvisPoints(double numPoints, double angle) {
		ArrayList<Translation2d> tempInvisPoints = pathPoints;

		Translation2d lastPoint = pathPoints.get(pathPoints.size() - 1);
		for (int i = 1; i <= numPoints; i++) {
			tempInvisPoints.add(lastPoint.plus(new Translation2d(SPACING * i,
				Rotation2d.fromDegrees(angle))));
		}

		pathPoints = tempInvisPoints;
		System.out.println("------------------------");
	}

}
