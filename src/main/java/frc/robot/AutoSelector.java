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
		modeChooser.setDefaultOption("Red 3 Ball A", DesiredMode.RED_3_BALL_A);
		modeChooser.addOption("Blue 3 Ball A", DesiredMode.BLUE_3_BALL_A);
		modeChooser.addOption("Red 3 Ball B", DesiredMode.RED_3_BALL_B);
		modeChooser.addOption("Blue 3 Ball B", DesiredMode.BLUE_3_BALL_B);
		modeChooser.addOption("Red 1 Ball", DesiredMode.RED_1_BALL);
		modeChooser.addOption("Blue 1 Ball", DesiredMode.BLUE_1_BALL);
		SmartDashboard.putData("Auto mode", modeChooser);
	}

	/**
	 * Returns current auto selector.
	 * @return auto selector
	 */
	public SendableChooser<DesiredMode> getAutoSelector() {
		return modeChooser;
	}

	/**
	 * Gets the current auto from Shuffleboard.
	 * @return name of current auto
	 */
	public DesiredMode getSelectedAuto() {
		return modeChooser.getSelected();
	}

	/**
	 * Returns if a red auto path is selected.
	 * @return true if a red auto path is selected
	 */
	public boolean isRedAutoSelected() {
		return (getSelectedAuto() == DesiredMode.RED_1_BALL)
			|| (getSelectedAuto() == DesiredMode.RED_3_BALL_A)
			|| (getSelectedAuto() == DesiredMode.RED_3_BALL_B);
	}

	/**
	* Outputs the current auto to Shuffleboard.
	*/
	public void outputToShuffleboard() {
		SmartDashboard.putString("Auto Mode selected", modeChooser.getSelected().name());
	}
}
