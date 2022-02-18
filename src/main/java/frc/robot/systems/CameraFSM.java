package frc.robot.systems;

// WPILib Imports
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;

// Robot Imports
import frc.robot.TeleopInput;

public class CameraFSM {
	// FSM state definitions
	public enum FSMState {
		DRIVER_CAM
        //CV_CAM
	}

    private final int fps = 30;
	private final int cameraBrightness = 25;
	private final int camWidth = 320;
	private final int camHeight = 240;

    UsbCamera driverCam;
    UsbCamera cvCam;
    VideoSink server;

	/* ======================== Private variables ======================== */
	private FSMState currentState;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public CameraFSM() {
        driverCam = CameraServer.startAutomaticCapture("Driver Camera", 0);
		cvCam = CameraServer.startAutomaticCapture("CV Camera", 1);

		// Reset state machine
		reset();
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset() {
		currentState = FSMState.DRIVER_CAM;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}
	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {
		switch (currentState) {
			case DRIVER_CAM:
				handleDriverState(input);
				break;

            // case CV_CAM:
            //     handleCvState(input);
            //     break;
            
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

    private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case DRIVER_CAM:
                // if(input != null) {
                //     if(input.isCamSwitchPressed()) {
                //         return FSMState.CV_CAM;
                //     }
                // }
				return currentState;

            // case CV_CAM:
            //     if(input != null) {
            //         if(input.isCamSwitchPressed()) {
            //             return FSMState.DRIVER_CAM;
            //         }
            //     }
			// 	return currentState;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

    private void handleDriverState(TeleopInput input) {
        CvSink cvSinkFront = CameraServer.getVideo(driverCam);
		CvSource outputStreamDriver =
			new CvSource("Front Camera", PixelFormat.kMJPEG, camWidth, camHeight, fps);
		cvSinkFront.setSource(outputStreamDriver);
		driverCam.setBrightness(cameraBrightness);
    }

    // private void handleCvState(TeleopInput input) {
    //     CvSink cvSinkCV = CameraServer.getVideo(cvCam);
    //     CvSource outputStreamCV =
    //         new CvSource("CV Camera", PixelFormat.kMJPEG, camWidth, camHeight, fps);
    //     cvSinkCV.setSource(outputStreamCV);
    //     cvCam.setBrightness(cameraBrightness);
    // }
}
