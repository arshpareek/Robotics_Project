

import java.util.ArrayList;
import java.util.Iterator;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;

/*
 * @author Jaeheon Lee
 */
public class PathFollowing {
	
	// Starting position of the robot.
	private Position startPos;
	
	// Motors for each wheel.
	private RegulatedMotor motorLeft;
	private RegulatedMotor motorRight;
	
	// Pilot class for navigation.
	private GyroPilot gyroPilot;
	
	// Object to fetch data from the gyro sensor.
	private SampleProvider gyroSample;
	EV3GyroSensor gyroSensor;
	
	// Object to fetch data from the color sensor.
	private SensorMode sensorColorID;
	
	// Creates a map with obstacles at required positions.
	private MapGenerator mapGenerator;
	
	// Stores the map for the board.
	Position[][] map;
	
	public PathFollowing(RegulatedMotor left, RegulatedMotor right, SensorMode sampleProvider, MapGenerator mapGenerator) {
		motorLeft = left;
		motorRight = right;
		sensorColorID = sampleProvider;
		gyroSensor = new EV3GyroSensor(SensorPort.S4);
		gyroSensor.reset();
		gyroSample = gyroSensor.getAngleMode();
		this.mapGenerator = mapGenerator;
	}
	
	// Combines the functionality of map generator, navigator and A* algorithm to move robot from one position to another.
	public void moveTo(Position start, Position targetPos, boolean goingToParking) {
		startPos = start;
		
		gyroPilot = new GyroPilot(gyroSample, motorLeft, motorRight, gyroSensor);
		
		// Create a map based on whether the robot is going to the parking area.
		mapGenerator.setGoingToTarget(goingToParking);
		Position[][] map = mapGenerator.getMap();
		
		AStarSearch pathFinder = new AStarSearch(map, (float) 7.5);
		ArrayList<Position> path = pathFinder.getPath(startPos, targetPos.xCoord, targetPos.yCoord);
		
		 for(int i = 0; i < path.size(); ++i) { 
			 System.out.println("Coordinates are: " + path.get(i).xCoord + ", " + path.get(i).yCoord);
			 System.out.println("____________");
		 }
		 
		
		// Provide the path to navigation class to navigate the robot.
		Navigation pathFollower = new Navigation(gyroPilot, path, gyroSample);
		pathFollower.navigate();
	}
	
	// Enters the parking area, reads the color inside, and exits the area.
	public void checkBox(int boxHeading) {
		
		// Rotate the robot to face the parking area entrance.
		gyroPilot.rotateTo(boxHeading);
		
		// Reset tachoCount to keep track of distnace inside the parking area.
		motorLeft.resetTachoCount();
		motorRight.resetTachoCount();
		
		//Initialise both touch sensors.
		EV3TouchSensor touchSensorOne = new EV3TouchSensor(SensorPort.S1);
		EV3TouchSensor touchSensorTwo = new EV3TouchSensor(SensorPort.S2);
		SensorMode touchSensorOneSample = touchSensorOne.getTouchMode();
		SensorMode touchSensorTwoSample = touchSensorTwo.getTouchMode();
		float []touchArray = new float [2]; 
		float touchValueOne = 0;
		float touchValueTwo = 0;
		
		// Move forward until both sensors are activated.
		while(touchValueOne == 0 && touchValueTwo == 0) {
			motorLeft.setSpeed(70);
			motorRight.setSpeed(70);
			motorLeft.forward();
			motorRight.forward();
			touchSensorOneSample.fetchSample(touchArray, 0); 
			touchSensorTwoSample.fetchSample(touchArray, 1); 
			touchValueOne = touchArray[0];
			touchValueTwo = touchArray[1];
		}
		
		// Stop to take the color reading.
		motorLeft.startSynchronization();
		motorLeft.stop();
		motorRight.stop();
		motorLeft.endSynchronization();
		
		// Take the color reading and change the map accordingly.
		float []colorArray = new float [1]; 
		sensorColorID.fetchSample(colorArray, 0); 
		float colorSample = colorArray[0];
		
		if(colorSample == Color.GREEN) {
			mapGenerator.setGreenObstacle();
		}
		else if(colorSample == Color.RED) {
			mapGenerator.setRedObstacle();
		}
		Sound.beepSequenceUp();
		mapGenerator.createReturnMap();
		
		// Rotate the motors backwards by the same amount to exit the area safely.
		motorLeft.startSynchronization();
		motorLeft.rotate( - motorLeft.getTachoCount());
		motorRight.rotate(- motorRight.getTachoCount());
		motorLeft.endSynchronization();
		motorLeft.waitComplete();
		
	}
	
	public GyroPilot getPilot() {
		return gyroPilot;
	}
}
