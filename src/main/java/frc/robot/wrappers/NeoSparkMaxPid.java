package frc.robot.wrappers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;

public class NeoSparkMaxPid {

	private final CANSparkMax motor;

	private final RelativeEncoder encoder;

	/**
	 * Constructor for Neo motor controlled by Spark Max
	 * controller wrapping up PIDF algorithms. Defaults
	 * to using 1 tick per revolution (almost certainly
	 * not the gear ratio of the actual motor). Tune the
	 * constant parameters to achieve desired outputs in
	 * terms of speed and accuracy.
	 * @param canId HardwareMapped ID
	 * @param p	 PIDF Position constant
	 * @param i	 PIDF Integral constant
	 * @param d	 PIDF Derivative constant
	 * @param f	 PIDF Feed Forward constant
	 */
	public NeoSparkMaxPid(final int canId,  final double p,
											final double i,
											final double d,
											final double f) {
		motor = new CANSparkMax(canId,
						CANSparkMax.MotorType.kBrushless);

		motor.getPIDController().setP(p);
		motor.getPIDController().setI(i);
		motor.getPIDController().setD(d);
		motor.getPIDController().setFF(f);
		motor.getPIDController().setIZone(0);
		motor.getPIDController().setOutputRange(-1, 1);

		encoder = motor.getEncoder();
	}


	/**
	 * Set the internal ticks per revolution to base velocity
	 * and position calculations on. By default is set to 1.
	 * If ticksPerRevolution is left at the default of 1, or
	 * is set to 1, then all encoder position/velocity
	 * methods return values in units of encoder ticks rather
	 * than revolutions
	 * @param ticksPerRevolution number of ticks per full
	 *	  revolution of the motor shaft
	 */
	public void setTicksPerRevolution(int ticksPerRevolution) {
		if (ticksPerRevolution < 1) {
			throw new IllegalArgumentException(
				"Ticks Per Revolution must be >= 1");
		}
		encoder.setPositionConversionFactor(ticksPerRevolution);
		encoder.setVelocityConversionFactor(ticksPerRevolution);
	}


	/**
	 * Use PIDF to set the velocity of the motor in revolutions
	 * per second.
	 * @param rpm target revolutions per second
	 */
	public void setVelocity(double rpm) {
		motor.getPIDController().setReference(rpm, ControlType.kVelocity);
	}


	/**
	 * Get the velocity of the motor in revolutions per second.
	 * @return velocity of the motor as measured by the encoder
	 */
	public double getVelocity() {
		return motor.getEncoder().getVelocity();
	}


	/**
	 * Set the position of the motor in units of revolutions.
	 * motor.setPosition(1), will cause the motor to go to the
	 * position exactly 1 positive revolution away from the
	 * zero position (as calculated by setTicksPerRevolution()).
	 * @param position desired position of the motor in units of
	 *	  revolutions
	 */
	public void setPosition(double position) {
		motor.getPIDController().setReference(position, ControlType.kPosition);
	}


	/**
	 * Return the revolutions away from the zero position the
	 * encoder has traveled.
	 * @return position of the encoder in revolutions
	 */
	public double getPosition() {
		return encoder.getPosition();
	}


	/**
	 * Set the desired behavior for the motor when idling.
	 * @param mode desired IdleMode for the motor
	 */
	public void setIdleMode(CANSparkMax.IdleMode mode) {
		motor.setIdleMode(mode);
	}


	/**
	 * Get the current behavior for the motor when idling.
	 * @return current idle behavior
	 */
	public CANSparkMax.IdleMode getIdleMode() {
		return motor.getIdleMode();
	}


	/**
	 * Get the motor temperature in Celcius.
	 * @return motor temperature in Celcius
	 */
	public double getMotorTemperature() {
		return motor.getMotorTemperature();
	}


	/**
	 * Get the motor object to pass to robot class for simulation.
	 * @return motor object
	 */
	public CANSparkMax getMotor() {
		return motor;
	}
}
