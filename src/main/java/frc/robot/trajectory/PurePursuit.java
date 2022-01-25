package frc.robot.trajectory;

import java.util.ArrayList;
import java.util.Collections;
import frc.robot.systems.DriveFSMSystem;

public class PurePursuit {

    DriveFSMSystem fsmSystem = new DriveFSMSystem();
    
    ArrayList<Point> keyPoints = new ArrayList<>();
    ArrayList<Point> pathPoints = new ArrayList<>();
    int lastClosestPointIndex = 0;
    int lookaheadDistance = 3; // lookahead point is 3 points ahead of the closest point

    // in inches
    private static final double SPACING = 6.0;


    public PurePursuit(ArrayList<Point> initialPoints, DriveFSMSystem fsmSystem) {
        keyPoints = initialPoints;
        this.fsmSystem = fsmSystem;
        pointInjection();
    }

    public void pathGen() {

    }

    private void pointInjection() {
        for (int i = 1; i < keyPoints.size(); i++){
            Point startPoint = keyPoints.get(i - 1);
            Point endPoint = keyPoints.get(i);

            Vector v = new Vector (startPoint, endPoint);

            double numPoints = Math.ceil(v.getMagnitude() / SPACING);
            v = v.normalize().multiplyByScalar(SPACING);

            for (int j = 0; j < (int) numPoints; j++) {
                v = v.multiplyByScalar(j);
                Point toInject = startPoint.addVector(v);
                pathPoints.add(toInject);
            }
            // need to consider point order when choosing closest point (in case of looped path) 
        }
    }

    private Point findClosestPoint() {
        // should i limit closest point search to 3 points ahead (so that in the case of a looped path, we don't end up choosing one of the final points?)
        Point currentPos = fsmSystem.updateArcOdometry();
        ArrayList<Double> tempDistances = new ArrayList<>();
        for (int i = lastClosestPointIndex; i < pathPoints.size(); i++) {
            double distance = Point.findDistance(currentPos, pathPoints.get(i));
            tempDistances.add(distance);
        }
        int indexOfMin = tempDistances.indexOf(Collections.min(tempDistances));
        lastClosestPointIndex += indexOfMin;
        return pathPoints.get(lastClosestPointIndex < pathPoints.size() ? lastClosestPointIndex : pathPoints.size() - 1);
    }

    private double calculateCurvature() {
        Point robotPos = fsmSystem.getRobotPosArc();
        double robotHeading = fsmSystem.getHeading();

        // prevent OOB errors
        int lookaheadPointIndex = lastClosestPointIndex + lookaheadDistance;
        Point lookaheadPoint = lookaheadPointIndex < pathPoints.size() ? pathPoints.get(lookaheadPointIndex) : pathPoints.get(pathPoints.size() - 1);
        double distanceToLookahead = Point.findDistance(fsmSystem.updateArcOdometry(), lookaheadPoint);
        
        // x is the horizontal distance to the lookahead point, a is -tan(robot angle), b = 1, c = tan(robot angle) * robot x - robot y
        double a = -Math.tan(robotHeading);
        double b = 1.0;
        double c = (Math.tan(robotHeading) * robotPos.x) - robotPos.y;
        double x = Math.abs(a * lookaheadPoint.x + b * lookaheadPoint.y + c) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));


        double curvature = (2 * x) / Math.pow(distanceToLookahead, 2);
        double side = Math.signum(Math.sin(robotHeading * (lookaheadPoint.x - robotPos.x) - Math.cos(robotHeading * (lookaheadPoint.y - robotPos.y))));

        return curvature * side;
    }
}
