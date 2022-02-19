package frc.robot.trajectory;

// WPILib Imports
import edu.wpi.first.math.geometry.Translation2d;

public class Vector extends Translation2d {

	/**
	 * Create vector and calculate its magnitude.
	 * @param xMag magnitude of vector in X direction.
	 * @param yMag magnitude of vector in Y direction.
	 */
	public Vector(double xMag, double yMag) {
		super(xMag, yMag);
	}

	/**
	 * Scale vector's length to 1 (and change x/y magnitudes accordingly).
	 * @return normalized vector with length 1
	 */
	public Vector normalize() {
		double originalMag = getNorm();
		double newXMag = getX() / Math.abs(originalMag);
		double newYMag = getY() / Math.abs(originalMag);
		return new Vector(newXMag, newYMag);
	}

	@Override
	public Vector times(double scalar) {
		return new Vector(getX() * scalar, getY() * scalar);
	}
}
