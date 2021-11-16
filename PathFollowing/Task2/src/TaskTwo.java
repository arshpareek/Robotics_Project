import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;

/*
 * This is the class with main method. This needs to be executed to run the program.
 * @author Arshdeep
 */

public class TaskTwo {
	
	// Initialise color sensor.
	public static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S3);
	public static SensorMode sensorRed = colorSensor.getRedMode();
	public static SensorMode sensorColorID = colorSensor.getColorIDMode();
	
	// Initialise motor objects.
	public static RegulatedMotor motorLeft = Motor.B;
	public static RegulatedMotor motorRight = Motor.C;

	public static void main(String[] args) {
		
		// Create a map.
		MapGenerator mapGenerator = new MapGenerator(123, 123, (float) 7.5);
		
		// Localise using Bayesian Localisation.
		Bayesian localiser = new Bayesian(sensorRed, motorLeft, motorRight);
		float location = localiser.bayesianLocalize();
		
		// Determine starting position.
		Position startPos = localiser.getCoordinates(location, 45);
        
		// Position object for the parking area.
        Position parkingPos = new Position((float) 56.25, (float) 108.75);
        // Position object for intermediate waypoint on return path.
        Position intermediateReturnPos = new Position((float) 93.75, (float) 56.25);
        // Position object for the end point of a lap.
        Position returnPos = new Position((float) 26.25, (float) 11.25);
        
        // Use path follower to move to the parking area.
        PathFollowing pathFollower = new PathFollowing(motorLeft, motorRight, sensorColorID, mapGenerator);
        pathFollower.moveTo(startPos, parkingPos, true);
        
		// Enter and exit the parking area.
        pathFollower.checkBox(0);
        
        // Move from the parking area to the intermediate waypoint.
        pathFollower.moveTo(parkingPos, intermediateReturnPos, false);
        
        // Move from the intermediate waypoint to the end point.
        pathFollower.moveTo(intermediateReturnPos, returnPos, false);
	}

	
}
