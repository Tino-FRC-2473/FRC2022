package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {
	enum DesiredMode {
		RED_3_BALL,
		BLUE_3_BALL,
		RED_2_BALL,
		BLUE_2_BALL,
		RED_1_BALL,
		BLUE_1_BALL
	}

	private SendableChooser<DesiredMode> modeChooser;

	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public AutoSelector() {
		modeChooser = new SendableChooser<>();
		modeChooser.setDefaultOption("Red 3 Ball", DesiredMode.RED_3_BALL);
		modeChooser.addOption("Blue 3 Ball A", DesiredMode.BLUE_3_BALL);
		modeChooser.addOption("Red 2 Ball", DesiredMode.RED_2_BALL);
		modeChooser.addOption("Blue 2 Ball", DesiredMode.BLUE_2_BALL);
		modeChooser.addOption("Red 1 Ball", DesiredMode.RED_1_BALL);
		modeChooser.addOption("Blue 1 Ball", DesiredMode.BLUE_1_BALL);
		SmartDashboard.putData("Auto mode", modeChooser);
	}

	/**
	 * Gets the current auto from Shuffleboard.
	 * @return name of current auto
	 */
	public String getSelectedAuto() {
		return modeChooser.getSelected().name();
	}

	/**
	* Outputs the current auto to Shuffleboard.
	*/
	public void outputToShuffleboard() {
		SmartDashboard.putString("Auto Mode selected", modeChooser.getSelected().name());
	}
}
