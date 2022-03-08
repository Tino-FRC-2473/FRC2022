package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoSelector {
	public enum DesiredMode {
		RED_3_BALL(3),
		BLUE_3_BALL(3),
		RED_2_BALL(3),
		BLUE_2_BALL(3),
		RED_1_BALL(1),
		BLUE_1_BALL(1),
		LEAVE_TARMAC(0);

		private final int numBalls;
		DesiredMode(int numBalls) {
			this.numBalls = numBalls;
		}

		/**
		 * Returns the number of balls being fired in the relevant mode.
		 * @return The number of balls being fired in the relevant mode.
		 */
		public int getNumBalls() {
			return numBalls;
		}
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
		modeChooser.addOption("Blue 3 Ball", DesiredMode.BLUE_3_BALL);
		modeChooser.addOption("Red 2 Ball", DesiredMode.RED_2_BALL);
		modeChooser.addOption("Blue 2 Ball", DesiredMode.BLUE_2_BALL);
		modeChooser.addOption("Red 1 Ball", DesiredMode.RED_1_BALL);
		modeChooser.addOption("Blue 1 Ball", DesiredMode.BLUE_1_BALL);
		modeChooser.addOption("Leave Tarmac", DesiredMode.LEAVE_TARMAC);
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
			|| (getSelectedAuto() == DesiredMode.RED_3_BALL)
			|| (getSelectedAuto() == DesiredMode.RED_2_BALL);
	}

	/**
	* Outputs the current auto to Shuffleboard.
	*/
	public void outputToShuffleboard() {
		SmartDashboard.putString("Auto Mode selected", modeChooser.getSelected().name());
	}
}
