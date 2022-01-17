package frc.robot.trajectory;

public class Vector {
    Point a, b;
    private double xMag, yMag, magnitude;

    public Vector (Point a, Point b) {
        this.a = a;
        this.b = b;
        calcMags();
    }

    private void calcMags() {
        xMag = Math.max(a.getX(), b.getX()) - Math.min(a.getX(), b.getX());
        yMag = Math.max(a.getY(), b.getY()) - Math.min(a.getY(), b.getY());
        magnitude = Math.sqrt(Math.pow(xMag, 2) + Math.pow(yMag, 2));
    }

    public Vector normalize() {
        xMag = xMag / magnitude;
        yMag = yMag / magnitude;
        magnitude = 1;
        return this;
    }

    public double getMagnitude() {
        return magnitude;
    }

    public Vector multiplyByScalar (double scalar) {
        xMag = xMag * scalar;
        yMag = yMag * scalar;
        magnitude = Math.sqrt(Math.pow(xMag, 2) + Math.pow(yMag, 2));
        return this;
    }
}
