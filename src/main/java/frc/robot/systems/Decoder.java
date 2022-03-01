package frc.robot.systems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.SerialPort;

import frc.robot.TeleopInput;

public class Decoder {
	/* ================== Constants ================== */
	public enum FSMState {
		//IDLE,
		LISTENING
	}
	/* ================== Private Variables ================== */
	private SerialPort port;
	
	//most current distance value read
	private double distance;
	
	//most current angle value read
	private double angle;

	//arraylist of all distance values
	private ArrayList<Double> distArray;

	//arraylist of all angle values
	private ArrayList<Double> anglArray;

	//current FSM state
	private FSMState currentState;
	
	/* ================== Constructor ================== */
	/**
	 * Constructor of the Decoder class. Creates FSMSystem and SerialPort 
	 * to read the data sent by Jetson
	 * @param baudRate the baud rate to configure the serial port
	 * @param serial the serial port to use
	 */
	public Decoder(int baudRate, SerialPort.Port serial) {
		distArray = new ArrayList<Double>();
		anglArray = new ArrayList<Double>();

		port = new SerialPort(baudRate, serial);

		reset();
	}
	
	/* ================== FSMSystem Handlers ================== */
	//public void idleState(TeleopInput input){}

	/**
	 * Handles behavior while in LISTENING.
	 * This method will read data via the SerialPort and, once it detects
	 * a full data set, will send the full data to the decoder method to 
	 * interpret the data. "X" will mark the beginning of the data string
	 * and "E" will mark the end. There will be two numbers between the 
	 * two characters: distance and angle, respectively. All variables are 
	 * separated by a new line.
	 * Example data sent:
	 * "
	 * X
	 * 12.34
	 * 1234.56
	 * E
	 * X
	 * 123.45
	 * -67.89
	 * E
	 * X
	 * 12
	 * "
	 * 
	 * @param input Global TeleopInput if the robot is in teleop mode or 
	 * null if it's in autonomous mode.
	 */
	private void listeningState(TeleopInput input){
		String str = "";
		boolean startedReading = false;

		while(true){

			String tempStr = port.readString();
			
			for(int i = 0; i < tempStr.length(); i++){

				if(tempStr.charAt(i) == 'X'){
					startedReading = true;
				}

				if(startedReading){
					str += tempStr.charAt(i);
				}
				
				if(tempStr.charAt(i) == 'E'){
					str += tempStr.charAt(i);
					decode(str);
					str = "";
					startedReading = false;
				}
			}
		}
		
	}

	/* ================== Public Methods ================== */

	/**
	 * Decodes String str, containing data retreived by the SerialPort and
	 * read by listeningState. Will separate the distance and angel variables
	 * into separate strings and parse them into Doubles. The private variables
	 * "distance" and "angle" will be updated and the new values will be added
	 * to the distArray and anglArray.
	 * 
	 * @param str String containing data to be read, starts with X and ends with E
	 */
	public void decode(String str) {
		double first = 0;
		int mult1 = 1;
		int mult2 = 1;
		double second = 0;
		String str1 = "";
		String str2 = "";
		
		boolean firstIsDone = false;
		
		
		for(int i = 0; i < str.length(); i++) {
			char c = str.charAt(i);
			if(str2.length() != 0 || c == '\n') {
				firstIsDone = true;
			}
			if(firstIsDone == false) {
				if(Character.isDigit(c) || c == '.') {
					str1 += c;
				}
				if(c == '-') {
					mult1 = -1;
				}
			}else {
				if(Character.isDigit(c) || c == '.') {
					str2 += c;
				}
				if(c == '-') {
					mult2 = -1;
				}
			}
		}
		
		first = Double.parseDouble(str1);
		second = Double.parseDouble(str2);
		
		distance = first*mult1;
		angle = second*mult2;

		distArray.add(distance);
		anglArray.add(angle);
		
	}
	
	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
	public void reset(){
		currentState = FSMState.LISTENING;

		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * 
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input){
		switch(currentState){
			case LISTENING:
				listeningState(input);
				break;
			//case IDLE:
				//idleState(input);
				//break;
			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
		
		currentState = nextState(input);
	}

	
	/**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * 
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * 
	 * @return FSM state for the next iteration
	 */
	private FSMState nextState(TeleopInput input){
		switch (currentState) {
			case LISTENING:
				//if (input == null) {
					return FSMState.LISTENING;
				//} else {
					//return FSMState.IDLE;
				//}

			//case IDLE:
				//return FSMState.IDLE;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/**
	 * Returns most current distance value read by the SerialPort
	 * 
	 * @return most current distance
	 */
	public double getDistance() {
		return distance;
	}

	/**
	 * Returns most current angle value read by the SerialPort
	 * 
	 * @return most current angle
	 */
	public double getAngle() {
		return angle;
	}	

	/**
	 * Returns an ArrayList containing all the distance values read by the 
	 * SerialPort
	 * 
	 * @return ArrayList containing all distance values
	 */
	public ArrayList<Double> getDistArray(){
		return distArray;
	}

	/**
	 * Returns an ArrayList containing all the angle values read by the
	 * SerialPort
	 * 
	 * @return ArrayList containing all angle values
	 */
	public ArrayList<Double> getAnglArray(){
		return anglArray;
	}

	/**
	 * Returns current FSM state
	 * 
	 * @return current FSM state
	 */
	public FSMState getCurrentState(){
		return currentState;
	}
	
}

