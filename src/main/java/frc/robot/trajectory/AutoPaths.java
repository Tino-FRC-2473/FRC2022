package frc.robot.trajectory;

// WPILib Imports
import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;

import frc.robot.Constants;

public class AutoPaths {
	//Auto paths for Pure Pursuit

	/**
	 * The red 2 ball path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Translation2d> r2BallPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(Constants.PP_R2_START_POINT);
		path.add(new Translation2d(129, -82));
		return path;
	}

	/**
	 * The red 2 hub path.
	 * @return an arraylist containing the key points
	 */

	public static ArrayList<Translation2d> r2HubPath() {
		ArrayList<Translation2d> path = new ArrayList<>();
		path.add(new Translation2d(129, -82));
		path.add(new Translation2d(33, -24));
		return path;
	}
}
