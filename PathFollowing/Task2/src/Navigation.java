import java.util.ArrayList;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.CompassPoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

/**
 * This class takes a path created by the A* algorithm and makes the 
 * robot traverse the path from its current position to its destination
 * 
 */

public class Navigation {

	// Initial angle of the robot.
	private GyroPilot gyroPilot;
	private ArrayList<Position> path;
	private SampleProvider gyroSample;
	
	public Navigation(GyroPilot gyroPilot, ArrayList<Position> path, SampleProvider gyro) {
		this.gyroPilot = gyroPilot;
		this.path = path;
		gyroSample = gyro;
	}
	
	/**
	 * This method moves the robot from its current position to the 
	 * final position of calculated by the A* search algorithm.
	 */
	public void navigate() {
		int i = 0;
		
		Position currPosition = path.get(0);
		Position nextPosition = path.get(1);
		
		float distance = currPosition.straightLineDistanceTo(nextPosition);
		float angle = currPosition.angleTo(nextPosition);
		
		// For the edge case when the robot starts too close to the centre block.
		if(angle == -135) {
			gyroPilot.travel(-distance);
			i = 1;
		}
		
		for(; i < path.size() - 1; ++i) {
			
			currPosition = path.get(i);
			nextPosition = path.get(i+1);
			
			angle = getNormalizedAngle(currPosition, nextPosition);
			
			distance = currPosition.straightLineDistanceTo(nextPosition);
			gyroPilot.rotateTo(angle);
			gyroPilot.travel(distance);
			
		}
	}
	
	/**
	 * @return angle between two positions normalized between -180 and 180 degrees
	 * @param currPosition, nextPosition
	 */
	private float getNormalizedAngle(Position currPosition, Position nextPosition) {
		float gyroValue = gyroPilot.getHeading();
		float angle = currPosition.angleTo(nextPosition);
		
		float angleDifference = angle - gyroValue;
		
		if(angleDifference < -180) {
			angle += 360;
		}
		if(angleDifference > 180) {
			angle -= 360;
		}
		if(angle < -180) {
			angle += 360;
		}
		if(angle > 180) {
			angleDifference -= 360;
		}
		
		return angle;
	}
	
	
}
