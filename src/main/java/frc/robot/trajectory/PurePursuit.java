package frc.robot.trajectory;

import java.util.ArrayList;

public class PurePursuit {
    
    ArrayList<Point> pathPoints = new ArrayList<>();

    // in inches
    private static final double SPACING = 6.0;


    public PurePursuit(ArrayList<Point> initialPoints) {
        pathPoints = initialPoints;
    }

    public void pathGen() {

    }

    private void pointInjection() {
        ArrayList<Point> newPoints = new ArrayList<>();
        for (int i = 1; i < pathPoints.size(); i++){
            Point startPoint = pathPoints.get(i - 1);
            Point endPoint = pathPoints.get(i);

            Vector v = new Vector (startPoint, endPoint);

            double numPoints = Math.ceil(v.getMagnitude() / SPACING);
            v = v.normalize().multiplyByScalar(SPACING);

            for (int j = 0; j < (int) numPoints; j++) {
                v = v.multiplyByScalar(j);
                Point toInject = startPoint.addVector(v);
                newPoints.add(toInject);
            }
            // need to consider point order when choosing closest point (in case of looped path) 
        }
    }
}
