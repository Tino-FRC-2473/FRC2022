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

	/**
	 * Scale vector's length to 1 (and change x/y magnitudes accordingly).
	 * @return normalized vector with length 1
	 */
	public Vector normalize() {
		double newXMag = xMag / Math.abs(magnitude);
		double newYMag = yMag / Math.abs(magnitude);
		return new Vector(newXMag, newYMag);
	}

	/**
	 *
	 * @return magnitude of vector
	 */
	public double getMagnitude() {
		return magnitude;
	}

	/**
	 * Get vector's magnitude in the x-direction.
	 * @return vector's magnitude in x-direction
	 */
	public double getXMag() {
		return xMag;
	}

	/**
	 * Get vector's magnitude in the y-direction.
	 * @return vector's magnitude in y-direction
	 */
	public double getYMag() {
		return yMag;
	}

	/**
	 * Scale up/down vector by a double value.
	 * @param scalar the scalar by which to multiply this vector
	 * @return scaled vector
	 */
	public Vector multiplyByScalar(double scalar) {
		double newXMag = xMag * scalar;
		double newYMag = yMag * scalar;
		return new Vector(newXMag, newYMag);
	}
}
