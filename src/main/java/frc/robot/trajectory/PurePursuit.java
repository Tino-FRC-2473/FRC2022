package frc.robot.trajectory;

import java.util.ArrayList;

public class PurePursuit {
    
    ArrayList<Point> keyPoints = new ArrayList<>();
    ArrayList<Point> pathPoints = new ArrayList<>();

    // in inches
    private static final double SPACING = 6.0;


    public PurePursuit(ArrayList<Point> initialPoints) {
        keyPoints = initialPoints;
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
        
    }
}
