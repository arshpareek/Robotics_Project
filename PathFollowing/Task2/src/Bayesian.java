
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;
import java.lang.Math;

/*
 * @author Jaeheon Lee.
 */

public class Bayesian {
	
	// Array of BayesCell objects to model the features of the line pattern.
	private BayesCell[] cells;
	
	private float cellLength = (float) 0.5;
	// Pilot class used for moving the robot forward.
	private MovePilot robotPilot;
	// Probabilities related to robot movement.
	private float P_moveWork = (float) 0.92;
	private float P_moveFail = (float) 0.08;
	// Probabilities related to sensor readings.
	private float P_sensorWork = (float) 0.9;
	private float P_sensorFail = (float) 0.1;
	// Color sensor object for retreiving data.
	private SensorMode sensorRed;
	// Objects for controlling each wheel.
	private RegulatedMotor motorLeft;
	private RegulatedMotor motorRight;
	
	
	public Bayesian(SensorMode sensorRed, RegulatedMotor left, RegulatedMotor right) {
		this.sensorRed = sensorRed;
		motorLeft = left;
		motorRight = right;
	}
	
	// Return the cell index in the cells array corresponding to the likely location of the robot.
	public float bayesianLocalize() {
		//Prepare pilot for movement.
		Wheel leftWheel = WheeledChassis.modelWheel(motorLeft, 56.0).offset(-60);
		Wheel rightWheel = WheeledChassis.modelWheel(motorRight, 56.0).offset(60);
		Chassis chassis = new WheeledChassis(new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);
		robotPilot = pilot;
		robotPilot.setLinearSpeed(160);
		robotPilot.setLinearAcceleration(80);
		
		// Creates a close representation of the actual line as an array.
		createMap(130);
		
		// Variables to store the highest and second highest peak probabilities.
		float highestProbability = 1;
		float secondHighestProbability = 1;

		// Loop until the robot has high likelihood of localisation.
		// High likelihood refers to the situation where the second highest peak in probability 
		// is less than 0.2 times the highest peak. This ratio has been tested to work with no failure.
		while(secondHighestProbability > 0.2 * highestProbability) {
		
		// Update localisation probabilities across the array depending on the sensor observation.
		measurementUpdate();
		
		// Move forward by one block distance (0.5cm) and perform control update.
		// Control update is forward shifting probabilities based on possibility of making a successful movement.
		moveForward();
		
		// Sum of all probabilities across the array is normalised to 1.
		normalizeProbabilities();
		
		// Update the two highest peak probabilities on the line.
		Pair<Float, Float>twoHighestProbabilities = findTwoHighestProbs();
		highestProbability = twoHighestProbabilities.getFirst().floatValue();
		secondHighestProbability = twoHighestProbabilities.getSecond().floatValue();
		
		} 
		
		// The final localisation refers to the location of the sensor. It needs to be offset by the distance to the wheel axle.
		int sensorToAxle = 11; // 1 cell equals 0.5cm
    	return (findHighestProbabilityIndex() - sensorToAxle) * cellLength;
	}
	
	// If the light sensor reading is less than 0.45, increase the probability of all blue cells.
	// Otherwise increase the probability of all white cells.
	public void measurementUpdate() {
		float []arr = new float [1]; 
		sensorRed.fetchSample(arr, 0); 
		float lightSample = arr[0];
		  
		// Value 0.45 is chosen as the midpoint between white and blue based on multiple readings.
		if(lightSample < 0.45) { 
			increaseBlueProbability(); 
		} 
		else {
		  increaseWhiteProbability(); 
		}
	}
	
	// Encodes the patterned line as an array.
	public void createMap(int resolution) {
		
		// Resolution is the number of cells in the array. The line is 65cm so 130 resolution means each cell is 0.5cm.
		cells = new BayesCell[resolution];
	
		// The light sensor cannot reach the first 15cm of the board (30 cells) because of its placement at the front.
		int inaccessibleCells = 30; 
		for(int i=0; i < inaccessibleCells; i++) {
			BayesCell newCell = new BayesCell(0, true); 
			cells[i] = newCell; 
		} 
		// The rest of the cells are given equal probability at the beginning.
		for(int i = inaccessibleCells; i < resolution; i++) { 
			BayesCell newCell = new BayesCell((float)1/(resolution-inaccessibleCells), true);
			cells[i] = newCell; 
		}
		
		// Certain cells are marked as white to accurately model the line. All are blue initially.
		createWhiteSpaces();
	}
	
	// Multiplies the probability of all cells marked as blue by P(sensor detecting correct color)
	// and remaining cells have their probability multiplied by P(sensor detecting incorrect color).
	public void increaseBlueProbability() {
		for(int i=0; i < cells.length; i++) {
			if(cells[i].isBlue) {
				cells[i].probability = (float) (cells[i].probability * P_sensorWork);
			}
			else {
				cells[i].probability = (float) (cells[i].probability * P_sensorFail);
			}
		}
	}
	
	// Multiplies the probability of all cells marked as blue by P(sensor detecting incorrect color)
	// and remaining cells have their probability multiplied by P(sensor detecting correct color).
	public void increaseWhiteProbability() {
		for(int i=0; i < cells.length; i++) {
			if(cells[i].isBlue) {
				cells[i].probability = (float) (cells[i].probability * P_sensorFail);
			}
			else {
				cells[i].probability = (float) (cells[i].probability * P_sensorWork);
			}
		}
	}
	
	// Multiplies the probability of each cell by the sum of probabilities.
	public void normalizeProbabilities() {
		float sumProbabilities = 0;
		for(int i=0; i < cells.length; i++) {
			sumProbabilities = (float) (sumProbabilities + cells[i].probability);
		}
		
		for(int i=0; i < cells.length; i++) {
			cells[i].probability = (float) (cells[i].probability / sumProbabilities);
		}
	}
	
	// Return the index of the cell with highest probability.
	public int findHighestProbabilityIndex() {
		float max = 0;
		int maxIndex = -1;
		for(int i=0; i < cells.length; i++) {
			float currProbability = cells[i].probability;
				if(currProbability >= max) {
					max = currProbability;
					maxIndex = i;
				}
		}
		return maxIndex;
	}
	
	// Returns a pair of probability values of the two highest peaks in the distribution. Written by Arshdeep.
	public Pair<Float, Float> findTwoHighestProbs() {
		float max = 0;
		float secondMax = 0;
		Pair<Float, Float> twoHighestProbs = new Pair<Float, Float>((float) -1, (float) -1);
		for(int i=1; i < cells.length - 1; i++) {
			float currProbability = cells[i].probability;
			if(currProbability >= cells[i-1].probability && currProbability > cells[i+1].probability) {
				if(currProbability >= max) {
					secondMax = max;
					max = currProbability;
				}
				if(currProbability >= secondMax && currProbability < max) {
					secondMax = currProbability;
				}
			
			}
		}
		twoHighestProbs = new Pair<Float, Float>(max, secondMax);
		return twoHighestProbs;
	}
	
	// Moves the robot forward by 0.5cm and performs control update.
	public void moveForward() {
		robotPilot.travel(5); // 5mm
		controlUpdate("forward");
	}
	
	// Depending on the direction of movement (always forward in this case), shift probabilities along the array
	// keeping movement probabilities in consideration.
	public void controlUpdate(String direction) {
		if(direction.contentEquals("forward")) {
			forwardShiftCells();
		}
		else if(direction.contentEquals("backward")) {
			backwardShiftCells();
		}
	}
	
	// After every forward movement, each cell probability changes to 
	// (P(movement fail) * current cell probability) + (P(movement successful) * previous cell probability).
	public void forwardShiftCells() {
		for(int i = cells.length - 1; i > 0; --i) {
			float currProbability = cells[i].probability;
			cells[i].probability = (float) ((P_moveWork * cells[i-1].probability) + (P_moveFail * currProbability));
		}
		cells[0].probability = 0;
	}
	
	
	// After every forward movement, each cell probability changes to 
	// (P(movement fail) * current cell probability) + (P(movement successful) * next cell probability).
	public void backwardShiftCells() {
		for(int i = 0; i < cells.length; --i) {
			float currProbability = cells[i].probability;
			cells[i].probability = (float) ((P_moveWork * cells[i+1].probability) + (P_moveFail * currProbability));
		}
		cells[cells.length - 1].probability = 0;
	}
	
	// Set certain cells to represent white spaces on the line.
	public void createWhiteSpaces() {
		/*
		 * setWhite(0, 13); setWhite(23, 27); setWhite(33, 40); setWhite(50, 54);
		 * setWhite(60, 67); setWhite(77, 84); setWhite(94, 98); setWhite(105, 111);
		 * setWhite(122, 125);
		 */
		
		//Input numbers of consecutive black and white blocks on the line into the array.
		//Each block represents 0.5cm on the actual line.
		int[] map = new int[] {10, 10, 3, 7, 7, 10, 3, 7, 7, 10, 7, 10, 3, 7, 7, 10, 3, 9};
		int offset = 0;
		for(int i=0; i < map.length; i += 2) {
			int startIndex = offset;
			int endIndex = startIndex + map[i];
			setWhite(startIndex, endIndex);
			offset = endIndex + map[i+1];
		}
	}
	
	// Marks a set of array cells starting from startIndex to endIndex as white.
	public void setWhite(int startIndex, int endIndex) {
		for(int i=startIndex; i < endIndex; ++i) {
			cells[i].setWhiteSpace();
		}
	}
	
	// Converts the robot's location from a distance on the line to cartesian coordinates on the board using trigonometry.
	public Position getCoordinates(float distOnLine, int angle) {
		float cornerOffset = 21;
		int xCoordinate = (int) ((distOnLine + cornerOffset) * Math.cos(Math.toRadians(angle)));
		int yCoordinate = (int) ((distOnLine + cornerOffset) * Math.sin(Math.toRadians(angle)));
		return new Position(xCoordinate, yCoordinate);
	}
	
}
