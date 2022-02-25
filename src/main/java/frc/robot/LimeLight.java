package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimeLight {
	private static LimeLight mLimeLight;

	private NetworkTable table;

	private NetworkTableEntry tx;
	private NetworkTableEntry ty;
	private NetworkTableEntry ta;

	/**
	 * LimeLight Constructor.
	 */
	public LimeLight() {
		table = NetworkTableInstance.getDefault().getTable("limelight");

		tx = table.getEntry("tx");
		ty = table.getEntry("ty");
		ta = table.getEntry("ta");
	}

	/**
	 * Checks if limelight is tracking any targets.
	 * @return If limelight has any valid targets (0 or 1)
	 */
	public boolean hasValidTargets() {
		return table.getEntry("tv").getDouble(0) == 1;
	}

	/**
	 * Returns horizontal offset from crosshair to target.
	 * @return horizontal offset from crosshair to target
	 */
	public double getXAngle() {
		return tx.getDouble(0);
	}

	/**
	 * Returns vertical offset from crosshair to target.
	 * @return vertical offset from crosshair to target
	 */
	public double getYAngle() {
		return ty.getDouble(0);
	}

	/**
	 * Returns area of vision tracking box.
	 * @return Target Area (0% to 100% of image)
	 */
	public double getArea() {
		return ta.getDouble(0);
	}

	/**
	 * Returns rotation of object.
	 * @return Skew or rotation of object (-90 to 0 degrees)
	 */
	public double getSkew() {
		return table.getEntry("ts").getDouble(0);
	}

	/**
	 * X-coordinates of the tracked box.
	 * @return Number array of corner x-coordinates
	 */
	public double[] getXCorners() {
		return table.getEntry("tcornx").getDoubleArray(new double[] {0, 0, 0, 0});
	}

	/**
	 * Y-coordinates of the tracked box.
	 * @return Number array of corner y-coordinates
	 */
	public double[] getYCorners() {
		return table.getEntry("tcorny").getDoubleArray(new double[] {0, 0, 0, 0});
	}

	/**
	 * Get the instance of LimeLight. Makes new instance if null.
	 * @return The LimeLight instance
	 */
	public static LimeLight getInstance() {
		if (mLimeLight == null) {
			mLimeLight = new LimeLight();
		}
		return mLimeLight;
	}

	/**
	 * Updates limelight data by calling outputToShuffleboard.
	 */
	public void update() {
		outputToShuffleboard();
	}

	/**
	 * Updates limelight data by calling outputToShuffleboard.
	 */
	public void update() {
		outputToShuffleboard();
	}

	/**
	 * Outputs data to Shuffleboard.
	 */
	private void outputToShuffleboard() {
		SmartDashboard.putBoolean("Has Target", hasValidTargets());
		SmartDashboard.putNumber("Horizontal Offset", getXAngle());
		SmartDashboard.putNumber("Vertical Offset", getYAngle());
		SmartDashboard.putNumber("Area", getArea());
	}
}
