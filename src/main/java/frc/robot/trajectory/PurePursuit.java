package frc.robot.trajectory;

import java.util.ArrayList;

import frc.robot.Constants;

public class PurePursuit {

	private ArrayList<Point> keyPoints = new ArrayList<>();
	private ArrayList<Point> pathPoints = new ArrayList<>();
	private int lastClosestPointIndex = 0;
	// lookahead point is this many inches ahead
	private final int lookaheadDistance = 9;

	// in inches
	private static final double SPACING = 6.0;

	public PurePursuit(ArrayList<Point> initialPoints) {
		keyPoints = initialPoints;
		pointInjection();
		for (Point p : pathPoints) {
			System.out.println("(" + p.getX() + ", " + p.getY() + ")");
		}
	}

	private void pointInjection() {
		for (int i = 1; i < keyPoints.size(); i++) {
			Point startPoint = keyPoints.get(i - 1);
			Point endPoint = keyPoints.get(i);

			Vector v = new Vector(startPoint, endPoint);

			double numPoints = Math.ceil(Math.abs(v.getMagnitude()) / SPACING);

			v = v.normalize().multiplyByScalar(SPACING);

			for (int j = 0; j <= (int) numPoints; j++) {
				Vector tempV = v.multiplyByScalar(j);

				Point toInject = startPoint.addVector(tempV);
				pathPoints.add(toInject);
			}
		}
	}

	private Point findClosestPoint(Point currentPos) {
		// temporary list of distances from robot to each path point
		ArrayList<Double> tempDistances = new ArrayList<>();

		// store distances from robot to each path point in temp list
		for (int i = lastClosestPointIndex; i < lastClosestPointIndex + lookaheadDistance; i++) {
			if (i >= pathPoints.size()) {
				break;
			}
			double distance = Point.findDistance(currentPos, pathPoints.get(i));
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
			Point closestPoint = pathPoints.get(lastClosestPointIndex);
			System.out.println("closest point: (" + closestPoint.getX() + ", "
				+ closestPoint.getY() + ")");
			return closestPoint;
		}

		// if the point is the last point (or OOB) return null
		return null;
	}

	public Point findLookahead(Point robotPos, double robotHeading) {
		// update index of closest point
		findClosestPoint(robotPos);

		// if the closest point is the last point (or OOB) return null
		if (findClosestPoint(robotPos) == null) {
			return null;
		}

		// prevent OOB errors
		int lookaheadPointIndex = lastClosestPointIndex;
		for (int i = lastClosestPointIndex + 1;
			i <= lastClosestPointIndex + lookaheadDistance; i++) {

			if (i >= pathPoints.size()) {
				break;
			}
			double dist = Point.findDistance(pathPoints.get(i), robotPos);
			if (dist < Constants.MAX_IN_TO_POINT) {
				lookaheadPointIndex = i;
			}
		}
		Point lookaheadPoint = lookaheadPointIndex < pathPoints.size()
			? pathPoints.get(lookaheadPointIndex) : pathPoints.get(pathPoints.size() - 1);
		return lookaheadPoint;
	}
}
