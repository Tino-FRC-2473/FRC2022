// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;

// Systems
import frc.robot.systems.DriveFSMSystem;
import frc.robot.systems.BallHandlingFSM;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private DriveFSMSystem driveFsmSystem;
	private BallHandlingFSM ballSystem;

	// Constants
	private final int fps = 30;
	private final int cameraBrightness = 25;
	private final int camWidth = 320;
	private final int camHeight = 240;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		// Instantiate all systems here
		driveFsmSystem = new DriveFSMSystem();
		ballSystem = new BallHandlingFSM();

		UsbCamera rearCam = CameraServer.startAutomaticCapture("Rear Camera", 0);
		CvSink cvSinkRear = CameraServer.getVideo(rearCam);
		CvSource outputStreamRear =
			new CvSource("Rear Camera", PixelFormat.kMJPEG, camWidth, camHeight, fps);
		cvSinkRear.setSource(outputStreamRear);
		rearCam.setBrightness(cameraBrightness);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		driveFsmSystem.reset();
		ballSystem.reset();
	}

	@Override
	public void autonomousPeriodic() {
		driveFsmSystem.update(input);
		ballSystem.update(null);
		updateShuffleboardVisualizations();
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		ballSystem.reset();
		driveFsmSystem.reset();
		ballSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		ballSystem.update(input);
		driveFsmSystem.update(input);
		ballSystem.update(input);
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
	}

	@Override
	public void simulationPeriodic() { }

	// Do not use robotPeriodic. Use mode specific periodic methods instead.
	@Override
	public void robotPeriodic() { }

	/**
	 * Updates shuffleboard values.
	 */
	public void updateShuffleboardVisualizations() {
		SmartDashboard.updateValues();
	}
}
