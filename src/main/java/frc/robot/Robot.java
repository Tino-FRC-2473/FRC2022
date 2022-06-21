// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

// WPILib Imports
import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.wpilibj.Relay;
// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.GrabberFSM;
import frc.robot.systems.MidHangerFSMSystem;
import frc.robot.systems.BallHandlingFSM;
import frc.robot.systems.CompressorSystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	private AutoSelector autoSelector;
	private LimeLight limelight;
	// private Relay lightswitch;

	// Systems
	private DriveFSMSystem driveFsmSystem;
	private BallHandlingFSM ballSystem;
	private GrabberFSM grabberSystem;
	// private MidHangerFSMSystem hangerSystem;

	// Constants
	private static final boolean RUN_COMPRESSOR = true;
	// private Timer timerForOff;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");

		//Init Compressor
		new CompressorSystem(RUN_COMPRESSOR);

		//Init Driver Inputs
		input = new TeleopInput();

		// Instantiate all systems here
		driveFsmSystem = new DriveFSMSystem();
		// hangerSystem = new MidHangerFSMSystem();
		ballSystem = new BallHandlingFSM();
		grabberSystem = new GrabberFSM();
		limelight = new LimeLight();
		limelight.setOffLimelight();
		autoSelector = new AutoSelector();

		// lightswitch.set(Relay.Value.kOn);
		// timerForOff.reset();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
		driveFsmSystem.reset(null);
		// hangerSystem.reset();
		ballSystem.reset();
		grabberSystem.reset();
		autoSelector.outputToShuffleboard();
		limelight.setAllianceColor(autoSelector.isRedAutoSelected());
		driveFsmSystem.setAutoPath(autoSelector.getSelectedAuto());
	}

	@Override
	public void autonomousPeriodic() {
		driveFsmSystem.update(input);
		ballSystem.update(null, driveFsmSystem.getCurrentState());
		// grabberSystem.update(null);
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		SmartDashboard.putString("Match Cycle", "TELEOP");
		driveFsmSystem.reset(input);

		// hangerSystem.reset();
		ballSystem.reset();
		grabberSystem.reset();
		limelight.update();
		// timerForOff.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveFsmSystem.update(input);
		// hangerSystem.update(input);
		ballSystem.update(input, driveFsmSystem.getCurrentState());
		grabberSystem.update(input);
		limelight.update();
		driveFsmSystem.setCVBallPos(limelight.getBallPosition());

		// lightswitch.set(Relay.Value.kOn);
		// if (timerForOff.hasElapsed(1)) {
		// 	timerForOff.reset();
		// 	lightswitch.set(Relay.Value.kOff);
		// }
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
		SmartDashboard.putString("Match Cycle", "DISABLED");
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
		SmartDashboard.putString("Match Cycle", "TEST");
	}

	@Override
	public void testPeriodic() {
	}

	/* Simulation mode handlers, only used for simulation testing  */
	@Override
	public void simulationInit() {
		// System.out.println("-------- Simulation Init --------");

		// CANSparkMax[] sparkMaxs = ballSystem.getSparkMaxs();

		// for (int i = 0; i < sparkMaxs.length; i++) {
		// 	REVPhysicsSim.getInstance().addSparkMax(sparkMaxs[i], DCMotor.getNEO(1));
		// }
	}

	@Override
	public void simulationPeriodic() {
		REVPhysicsSim.getInstance().run();
	}

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}
