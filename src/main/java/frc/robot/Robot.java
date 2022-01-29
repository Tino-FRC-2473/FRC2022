// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.CvSink;
// import edu.wpi.first.cscore.CvSource;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

// Systems
import frc.robot.systems.FSMSystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;

	// Systems
	private FSMSystem fsmSystem;
	
	// Constants
	private final int CAMERA_BRIGHTNESS = 25;
	private final int FPS = 15;

	NetworkTableEntry cameraSelection;

	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		System.out.println("robotInit");
		input = new TeleopInput();

		// Instantiate all systems here
		fsmSystem = new FSMSystem();

		UsbCamera frontCam = CameraServer.startAutomaticCapture("Front Camera", 0);
		frontCam.setBrightness(CAMERA_BRIGHTNESS);
		frontCam.setFPS(FPS);

		cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("Camera Selection");

		UsbCamera rearCam = CameraServer.startAutomaticCapture("Rear Camera", 1);
		rearCam.setBrightness(CAMERA_BRIGHTNESS);
		rearCam.setFPS(FPS);
	}

	@Override
	public void autonomousInit() {
		System.out.println("-------- Autonomous Init --------");
		fsmSystem.reset();
	}

	@Override
	public void autonomousPeriodic() {
		updateShuffleboardVisualizations();
		fsmSystem.update(null);
	}

	@Override
	public void teleopInit() {
		System.out.println("-------- Teleop Init --------");
		fsmSystem.reset();
	}

	@Override
	public void teleopPeriodic() {
		updateShuffleboardVisualizations();
		fsmSystem.update(input);
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

	/* Update Shuffleboard visualizations */
	public void updateShuffleboardVisualizations() {
		Shuffleboard.update();
	}
}
