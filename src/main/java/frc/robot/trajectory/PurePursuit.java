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
        for (int i = 1; i < pathPoints.size(); i++){
            Point startPoint = pathPoints.get(i - 1);
            Point endPoint = pathPoints.get(i);

            Vector v = new Vector (startPoint, endPoint);

            double numPoints = Math.ceil(v.getMagnitude() / SPACING);
            v = v.normalize().multiplyByScalar(SPACING);

            for (int j = 0; j < (int) numPoints; j++) {
                // add (vector * i + startPoint) to pathPoints arraylist
            }
            // need to consider point order when choosing closest point (in case of looped path) 
        }
    }
}
