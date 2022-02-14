package frc.robot.trajectory;

public class Point {
	private double x;
	private double y;

	/**
	 * Create point.
	 * @param x x coordinate of point
	 * @param y y coordinate of point
	 */
	public Point(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/**
	 * Get x coordinate of point.
	 * @return x coordinate of point
	 */
	public double getX() {
		return x;
	}

	/**
	 * Get y coordinate of point.
	 * @return y coordinate of point
	 */
	public double getY() {
		return y;
	}

	/**
	 * Set x coordinate of point.
	 * @param x new x coordinate for point
	 */
	public void setX(double x) {
		this.x = x;
	}

	/**
	 * Set y coordinate of point.
	 * @param y new y coordinate for point
	 */
	public void setY(double y) {
		this.y = y;
	}

	/**
	 * Change x coordinate of point.
	 * @param x x-value to add to current x-coordinate.
	 */
	public void addX(double x) {
		this.x += x;
	}

	/**
	 * Change y coordinate of point.
	 * @param y y-value to add to current y-coordinate.
	 */
	public void addY(double y) {
		this.y += y;
	}

	/**
	 * Translate point by adding a vector.
	 * @param v Vector to add to this point
	 * @return translated point
	 */
	public Point addVector(Vector v) {
		double newX = x + v.getXMag();
		double newY = y + v.getYMag();
		return new Point(newX, newY);
	}

	/**
	 * Find distance between two points.
	 * @param a first point
	 * @param b second point
	 * @return distance between the two points
	 */
	public static double findDistance(Point a, Point b) {
		return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
	}
}
