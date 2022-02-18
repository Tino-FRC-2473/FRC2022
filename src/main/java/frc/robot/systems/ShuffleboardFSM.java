package frc.robot.systems;

import frc.robot.TeleopInput;

public class ShuffleboardFSM {
    public enum FSMState {
        SHUFFLEBOARD_STATE
    }

    private FSMState currentState;

    public ShuffleboardFSM() {
        reset();
    }

    public void reset() {
		currentState = FSMState.SHUFFLEBOARD_STATE;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

    public void update(TeleopInput input) {
		switch (currentState) {
			case SHUFFLEBOARD_STATE:
				updateShuffleboardData();
				break;
            
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		currentState = nextState(input);
	}

    private FSMState nextState(TeleopInput input) {
		switch (currentState) {
			case SHUFFLEBOARD_STATE:
				return currentState;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

    public void updateShuffleboardData() {
        
    }
}
