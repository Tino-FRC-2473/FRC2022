// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.math.system.plant.DCMotor;
// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoException;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.GrabberFSM;
import frc.robot.wrappers.HardwareUtility;
import frc.robot.systems.BallHandlingFSM;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	private HardwareUtility hwUtilityController;

	// Systems
	private DriveFSMSystem driveFsmSystem;
	private BallHandlingFSM ballSystem;
	private GrabberFSM grabberSystem;

	// Constants
	private final int fps = 30;
	private final int cameraBrightness = 25;
	private final int camWidth = 320;
	private final int camHeight = 240;
	public static final boolean RUN_COMPRESSOR = false;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");

		//Init hardware utility controller
		hwUtilityController = new HardwareUtility(RUN_COMPRESSOR);
		
		//Init Driver Inputs
		input = new TeleopInput();

		// Instantiate all systems here
		driveFsmSystem = new DriveFSMSystem();
		ballSystem = new BallHandlingFSM();
		grabberSystem = new GrabberFSM();

		//Initialize CV resources
		try {
			UsbCamera driverCamera = CameraServer.startAutomaticCapture("Driver Camera", 0);
			CvSink cvSinkRear = CameraServer.getVideo(driverCamera);
			CvSource outputStreamRear =
				new CvSource("Driver Camera", PixelFormat.kMJPEG, camWidth, camHeight, fps);
			cvSinkRear.setSource(outputStreamRear);
			driverCamera.setBrightness(cameraBrightness);
		} catch (VideoException e) {
			e.printStackTrace();
		}
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		driveFsmSystem.reset();
		ballSystem.reset();
		grabberSystem.reset();
	}

	@Override
	public void autonomousPeriodic() {
		driveFsmSystem.update(input);
		ballSystem.update(null);
		grabberSystem.update(null);
		updateShuffleboardVisualizations();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		driveFsmSystem.reset();
		ballSystem.reset();
		grabberSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		driveFsmSystem.update(input);
		ballSystem.update(input);
		grabberSystem.update(input);
		updateShuffleboardVisualizations();
	}

	@Override
	public void disabledInit() {
		System.out.println("-------- Disabled Init --------");
	}

	@Override
	public void disabledPeriodic() {
		updateShuffleboardVisualizations();
	}

	@Override
	public void testInit() {
		System.out.println("-------- Test Init --------");
	}

	@Override
	public void testPeriodic() {
		updateShuffleboardVisualizations();
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

	/**
	 * Updates shuffleboard values.
	 */
	public void updateShuffleboardVisualizations() {
		SmartDashboard.updateValues();
	}

	/**
	 * Get the hardware utility controller object to control
	 * and read information from the PDH and Compressor.
	 * @return HardwareUtility object
	 */
	public HardwareUtility getHardwareUtilityController() {
		return hwUtilityController;
	}
}
