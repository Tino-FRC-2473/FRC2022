package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {
	public enum DesiredMode {
		RED_3_BALL_A,
		BLUE_3_BALL_A,
		RED_3_BALL_B,
		BLUE_3_BALL_B,
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
		modeChooser.setDefaultOption("3 Ball Red A", DesiredMode.RED_3_BALL_A);
		modeChooser.addOption("3 Ball Blue A", DesiredMode.BLUE_3_BALL_A);
		modeChooser.addOption("3 Ball Red B", DesiredMode.RED_3_BALL_B);
		modeChooser.addOption("3 Ball Blue B", DesiredMode.BLUE_3_BALL_B);
		modeChooser.addOption("1 Ball Red", DesiredMode.RED_1_BALL);
		modeChooser.addOption("1 Ball Blue", DesiredMode.BLUE_1_BALL);
		SmartDashboard.putData("Auto mode", modeChooser);
	}

	/**
	 * Gets the current auto from Shuffleboard.
	 * @return name of current auto
	 */
	public DesiredMode getSelectedAuto() {
		return modeChooser.getSelected();
	}

	/**
	* Outputs the current auto to Shuffleboard.
	*/
	public void outputToShuffleboard() {
		SmartDashboard.putString("Auto Mode selected", modeChooser.getSelected().name());
	}
}
