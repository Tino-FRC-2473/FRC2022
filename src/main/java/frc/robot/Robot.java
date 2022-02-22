// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

// WPILib Imports
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.GrabberFSM;
import frc.robot.systems.BallHandlingFSM;
import frc.robot.systems.CameraFSM;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private DriveFSMSystem driveFsmSystem;
	private BallHandlingFSM ballSystem;
	private GrabberFSM grabberSystem;
	private CameraFSM cameraSystem;

	private AutoSelector autoSelector = new AutoSelector();

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		//Enable Compressor feedback loop on the REV PH
		new Compressor(1, PneumaticsModuleType.REVPH).enableDigital();

		// Instantiate all systems here
		driveFsmSystem = new DriveFSMSystem();
		ballSystem = new BallHandlingFSM();
		grabberSystem = new GrabberFSM();
		cameraSystem = new CameraFSM();

		autoSelector.updateModeChooser();
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
		driveFsmSystem.reset();
		ballSystem.reset();
		grabberSystem.reset();
		cameraSystem.reset();
		autoSelector.updateModeChooser();
	}

	@Override
	public void autonomousPeriodic() {
		driveFsmSystem.update(input);
		ballSystem.update(null);
		grabberSystem.update(null);
		cameraSystem.update(null);
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		SmartDashboard.putString("Match Cycle", "TELEOP");
		driveFsmSystem.reset();
		ballSystem.reset();
		grabberSystem.reset();
		cameraSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveFsmSystem.update(input);
		ballSystem.update(input);
		grabberSystem.update(input);
		cameraSystem.update(input);
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
		SmartDashboard.putString("Match Cycle", "DISABLED");

		autoSelector.reset();
		autoSelector.updateModeChooser();
	}

	@Override
	public void disabledPeriodic() {
		autoSelector.updateModeChooser();
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
		System.out.println("-------- Simulation Init --------");

		CANSparkMax[] sparkMaxs = ballSystem.getSparkMaxs();

		for (int i = 0; i < sparkMaxs.length; i++) {
			REVPhysicsSim.getInstance().addSparkMax(sparkMaxs[i], DCMotor.getNEO(1));
		}

		System.out.println("-------- Simulation Init --------");
	}

	@Override
	public void simulationPeriodic() {
		REVPhysicsSim.getInstance().run();
	}

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }
}
