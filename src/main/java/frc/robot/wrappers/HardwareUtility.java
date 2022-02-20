package frc.robot.wrappers;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class HardwareUtility {

	//HARDWARE Components
	private final PowerDistribution pDH;

	private Compressor pneumaticsCompressor;

	/**
	 * Constructor for general HardwareController.
	 * @param runCompressor Whether or not the compressor
	 * 		should be running (must be TRUE for games, or
	 *	  anytime pneumatics are being used)
	 */
	public HardwareUtility(boolean runCompressor) {
		pDH = new PowerDistribution(1, ModuleType.kRev);

		pneumaticsCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

		if (runCompressor) {
			pneumaticsCompressor.enableDigital();
		} else {
			pneumaticsCompressor.disable();
		}
	}

	/**
	 * Get the current pressure of the pressure tanks.
	 * @return current pressure of the pressure tanks
	 */
	public double getCompressorPressure() {
		return pneumaticsCompressor.getPressure();
	}

	/**
	 * Get the current consumption of the Compressor in Amps.
	 * @return current consumption of the Compressor in Amps
	 */
	public double getCompressorCurrent() {
		return pneumaticsCompressor.getCurrent();
	}

	/**
	 * Set the desired state of the Switchable Channel on the PDH.
	 * True if channel should be enabled, false if should be disabled.
	 * @param state desired state of the Switchable Channel on the PDH
	 */
	public void setPDPSwitchableChannel(boolean state) {
		pDH.setSwitchableChannel(state);
	}

	/**
	 * Get the current state of the Switchable Channel on the PDH.
	 * True if the channel is enabled, false if it is disabled.
	 * @return Current state of the Switchable Channel on the PDH
	 */
	public boolean getPDPSwitchableChannel() {
		return pDH.getSwitchableChannel();
	}

	/**
	 * Get the object for the REV Power Distribution Hub. This can
	 * be used to get information about temperature, power
	 * consumption, etc, of the PDH, and information about current
	 * consumption of the individual channels on the PDH.
	 * @return object representing the REV Power Distribution Hub
	 */
	public PowerDistribution getPDH() {
		return pDH;
	}
}
