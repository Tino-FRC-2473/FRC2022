package frc.robot.trajectory;

public class Point {
    double x, y;

    public Point (double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return x;
    }
     
    public double getY() {
        return y;
    }

    public Point addVector (Vector v) {
        double newX = x + v.getXMag();
        double newY = y + v.getYMag();
        return new Point(newX, newY);
    }

    public static double findDistance(Point a, Point b){
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }
}
