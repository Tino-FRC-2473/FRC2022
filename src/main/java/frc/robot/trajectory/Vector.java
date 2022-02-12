package frc.robot.trajectory;

public class Vector {

	private double xMag;
	private double yMag;
	private double magnitude;

	/**
	 * Create vector and calculate its magnitude.
	 * @param a initial point of vector
	 * @param b point that the vector ends at
	 */
	public Vector(Point a, Point b) {
		xMag = b.getX() - a.getX();
		yMag = b.getY() - a.getY();
		calcMags();
	}

	/**
	 * Create vector and calculate its magnitude.
	 * @param xMag magnitude of vector in X direction.
	 * @param yMag magnitude of vector in Y direction.
	 */
	public Vector(double xMag, double yMag) {
		this.xMag = xMag;
		this.yMag = yMag;
		calcMags();
	}

	private void calcMags() {
		magnitude = Math.sqrt(Math.pow(xMag, 2) + Math.pow(yMag, 2));
		if ((xMag < 0 && yMag >= 0) || (yMag < 0 && xMag >= 0)) {
			magnitude = -magnitude;
		}
	}

	public Vector normalize() {
		double newXMag = xMag / Math.abs(magnitude);
		double newYMag = yMag / Math.abs(magnitude);
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

	public Vector multiplyByScalar(double scalar) {
		double newXMag = xMag * scalar;
		double newYMag = yMag * scalar;
		return new Vector(newXMag, newYMag);
	}
}
