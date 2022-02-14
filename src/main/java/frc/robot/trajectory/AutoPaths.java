package frc.robot.trajectory;

import java.util.ArrayList;

import frc.robot.Constants;

public class AutoPaths {
	//Auto paths for Pure Pursuit

	/**
	 * The red 2 ball path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Point> r2BallPath() {
		ArrayList<Point> path = new ArrayList<>();
		path.add(Constants.PP_R2_START_POINT);
		path.add(new Point(129, -82));
		return path;
	}

	/**
	 * The red 2 hub path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Point> r2HubPath() {
		ArrayList<Point> path = new ArrayList<>();
		path.add(new Point(129, -82));
		path.add(new Point(33, -24));
		return path;
	}
}
