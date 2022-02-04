package frc.robot.trajectory;

import java.util.ArrayList;
import java.util.Collections;

import org.ejml.dense.row.linsol.qr.SolvePseudoInverseQrp_DDRM;

import frc.robot.systems.DriveFSMSystem;

public class PurePursuit {

	private DriveFSMSystem fsmSystem;

	private ArrayList<Point> keyPoints = new ArrayList<>();
	private ArrayList<Point> pathPoints = new ArrayList<>();
	private int lastClosestPointIndex = 0;
	// lookahead point is this many points ahead of the closest point
	private final int lookaheadDistance = 5;

	// in inches
	private static final double SPACING = 6.0;


	public PurePursuit(ArrayList<Point> initialPoints, DriveFSMSystem fsmSystem) {
		keyPoints = initialPoints;
		this.fsmSystem = fsmSystem;
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
			// need to consider point order when choosing closest point
			// (in case of looped path)
		}
	}

	private Point findClosestPoint() {
		
		Point currentPos = fsmSystem.getRobotPosLine();

		ArrayList<Double> tempDistances = new ArrayList<>();

		for (int i = lastClosestPointIndex; i < lastClosestPointIndex + lookaheadDistance; i++) {
			double distance = Point.findDistance(currentPos, pathPoints.get(i));
			tempDistances.add(distance);
		}

		// check what happens if two or more distances are equal; prevent overlap
		int indexOfMin = tempDistances.indexOf(Collections.min(tempDistances));
		lastClosestPointIndex += indexOfMin;
		if (lastClosestPointIndex < pathPoints.size() - 1) {
			Point closestPoint = pathPoints.get(lastClosestPointIndex);
			System.out.println("closest point: (" + closestPoint.getX() + ", " + closestPoint.getY() + ")");
        	return closestPoint;
        }

        return null;
	}

	public Point findLookahead() {
		findClosestPoint();
		Point robotPos = fsmSystem.getRobotPosArc();
		double robotHeading = fsmSystem.getHeading();

		if (findClosestPoint() == null) {

			return null;
		}
		// prevent OOB errors
		int lookaheadPointIndex = lastClosestPointIndex + lookaheadDistance;
		Point lookaheadPoint = lookaheadPointIndex < pathPoints.size()
			? pathPoints.get(lookaheadPointIndex) : pathPoints.get(pathPoints.size() - 1);
		return lookaheadPoint;
	}

	// private double calculateCurvature() {
	//     Point robotPos = fsmSystem.getRobotPosArc();
	//     double robotHeading = fsmSystem.getHeading();

	//     // prevent OOB errors
	//     int lookaheadPointIndex = lastClosestPointIndex + lookaheadDistance;
	//     Point lookaheadPoint = lookaheadPointIndex < pathPoints.size()
	//         ? pathPoints.get(lookaheadPointIndex) : pathPoints.get(pathPoints.size() - 1);
	//     double distanceToLookahead = Point.findDistance(fsmSystem.updateArcOdometry(),
	//         lookaheadPoint);

	//     // x is the horizontal distance to the lookahead point,
	//     // a is -tan(robot angle), b = 1, c = tan(robot angle) * robot.getX() - robot.getY()
	//     double a = -Math.tan(robotHeading);
	//     double b = 1.0;
	//     double c = (Math.tan(robotHeading) * robotPos.getX()) - robotPos.getY();
	//     double x = Math.abs(a * lookaheadPoint.getX() + b * lookaheadPoint.getY() + c)
	//         / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));


	//     double curvature = (2 * x) / Math.pow(distanceToLookahead, 2);
	//     double side = Math.signum(Math.sin(robotHeading * (lookaheadPoint.getX() - robotPos.getX())
	//         - Math.cos(robotHeading * (lookaheadPoint.getY() - robotPos.getY()))));

	//     return curvature * side;
	// }
}