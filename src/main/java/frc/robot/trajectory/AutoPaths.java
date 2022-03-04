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
		return path;
	}

	/**
	 * The red 1 "hub" path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> r1HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(150, 3));
		path.add(new Translation2d(150, 3));
		return path;
	}

	/**
	 * The red 2 ball path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Translation2d> r2BallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(Constants.PP_R2_START_POINT);
		path.add(new Translation2d(191.230, 0));
		path.add(new Translation2d(227.904, 65.860));
		path.add(new Translation2d(269.300, 105.490));
		return path;
	}

	/**
	 * The red 2 hub path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Translation2d> r2HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(269.300, 105.490));
		path.add(new Translation2d(227.904, 65.860));
		path.add(new Translation2d(191.230, 0));
		path.add(new Translation2d(62, -22));
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
		path.add(new Translation2d(191.230, 0));
		path.add(new Translation2d(227.904, 65.860));
		// path.add(new Translation2d(249.300, 85.490));
		path.add(new Translation2d(276.800, 112.990));
		return path;
	}

	/**
	 * The red 3 hub path.
	 * @return an arraylist containing the key points
	 */
	public static ArrayList<Translation2d> r3HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		// path.add(new Translation2d(249.300, 85.490));
		path.add(new Translation2d(276.800, 112.990));
		path.add(new Translation2d(227.904, 65.860));
		path.add(new Translation2d(191.230, 0));
		path.add(new Translation2d(62, -22));
		return path;
	}
}
