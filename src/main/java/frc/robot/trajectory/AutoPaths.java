package frc.robot.trajectory;

// WPILib Imports
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;

import frc.robot.Constants;

public class AutoPaths {
	//Auto paths for Pure Pursuit

	/**
	 * The red 1 ball path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> r1BallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(Constants.PP_R1_START_POINT);
		path.add(new Translation2d(62, -22));
		path.add(new Translation2d(150, 3));
		path.add(new Translation2d(225, 3));
		return path;
	}

	/**
	 * The red 1 "hub" path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> r1HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(225, 3));
		path.add(new Translation2d(225, 3));
		return path;
	}

	/**
	 * The red 2 ball path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Translation2d> r2BallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(Constants.PP_R2_START_POINT);
		// path.add(new Translation2d(191.230, 0));
		path.add(new Translation2d(149.215, 8.764));
		path.add(new Translation2d(227.904, 65.860));
		// path.add(new Translation2d(278.800, 105.490));
		path.add(new Translation2d(275.188, 111.126));
		return path;
	}

	/**
	 * The red 2 hub path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Translation2d> r2HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		// path.add(new Translation2d(278.800, 105.490));
		path.add(new Translation2d(275.188, 111.126));
		path.add(new Translation2d(227.904, 65.860));
		// path.add(new Translation2d(191.230, 0));
		path.add(new Translation2d(149.215, 8.764));
		path.add(new Translation2d(55, -19));
		return path;
	}

	/**
	 * The red 3 ball path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> r3BallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(Constants.PP_R3_START_POINT);
		path.add(new Translation2d(62, -22));
		path.add(new Translation2d(149.215, 8.764));
		path.add(new Translation2d(227.904, 65.860));
		path.add(new Translation2d(275.188, 111.126));
		return path;
	}

	/**
	 * The red 3 hub path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> r3HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(275.188, 111.126));
		path.add(new Translation2d(227.904, 65.860));
		path.add(new Translation2d(149.215, 8.764));
		path.add(new Translation2d(55, -19));
		return path;
	}

	/**
	 * The blue 1 ball path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> b1BallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(Constants.PP_B1_START_POINT);
		path.add(new Translation2d(-62, 22));
		path.add(new Translation2d(-150, -3));
		path.add(new Translation2d(-225, -3));
		return path;
	}

	/**
	 * The blue 1 "hub" path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> b1HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(-225, -3));
		path.add(new Translation2d(-225, -3));
		return path;
	}

	/**
	 * The blue 2 ball path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Translation2d> b2BallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(Constants.PP_B2_START_POINT);
		// path.add(new Translation2d(-191.230, 0));
		path.add(new Translation2d(-149.215, -8.764));
		path.add(new Translation2d(-227.904, -65.860));
		// path.add(new Translation2d(-278.800, -105.490));
		path.add(new Translation2d(-275.188, -111.126));
		return path;
	}

	/**
	 * The blue 2 hub path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Translation2d> b2HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		// path.add(new Translation2d(-278.800, -105.490));
		path.add(new Translation2d(-275.188, -111.126));
		path.add(new Translation2d(-227.904, -65.860));
		// path.add(new Translation2d(-191.230, 0));
		path.add(new Translation2d(-149.215, -8.764));
		path.add(new Translation2d(-55, 19));
		return path;
	}

	/**
	 * The blue 3 ball path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> b3BallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(Constants.PP_B3_START_POINT);
		path.add(new Translation2d(-62, 22));
		path.add(new Translation2d(-149.215, -8.764));
		path.add(new Translation2d(-227.904, -65.860));
		path.add(new Translation2d(-275.188, -111.126));
		return path;
	}

	/**
	 * The blue 3 hub path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> b3HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(-275.188, -111.126));
		path.add(new Translation2d(-227.904, -65.860));
		path.add(new Translation2d(-149.215, -8.764));
		path.add(new Translation2d(-55, 19));
		return path;
	}

	/**
	 * The path to leave the tarmac.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> leaveTarmacBallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(0, 0));
		path.add(new Translation2d(0, 90));
		return path;
	}

	/**
	 * The leave tarmac "hub" path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> leaveTarmacHubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(0, 90));
		path.add(new Translation2d(0, 90));
		return path;
	}
}
