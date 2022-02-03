package frc.robot.trajectory;

public class Vector {

	private double xMag;
	private double yMag;
	private double magnitude;

	public Vector(Point a, Point b) {
		xMag = Math.max(a.getX(), b.getX()) - Math.min(a.getX(), b.getX());
		yMag = Math.max(a.getY(), b.getY()) - Math.min(a.getY(), b.getY());
		calcMags();
	}

	public Vector(double xMag, double yMag) {
		this.xMag = xMag;
		this.yMag = yMag;
		calcMags();
	}

	private void calcMags() {
		magnitude = Math.sqrt(Math.pow(xMag, 2) + Math.pow(yMag, 2));
	}

	public Vector normalize() {
		double newXMag = xMag / magnitude;
		double newYMag = yMag / magnitude;
		return new Vector(newXMag, newYMag);
	}

	public double getMagnitude() {
		return magnitude;
	}

	public double getXMag() {
		return xMag;
	}

	public double getYMag() {
		return yMag;
	}

	public Vector multiplyByScalar (double scalar) {
		double newXMag = xMag * scalar;
		double newYMag = yMag * scalar;
		return new Vector(newXMag, newYMag);
	}
}
