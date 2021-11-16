/*
 * @author Shahdad Razavi Algharavi
 */
public class Position {
	// Cartesian coordinates for cell location.
	public float xCoord;
	public float yCoord;
	
	// Determines whether an obstacle is located at this position.
	public boolean isObstacle;
	
	// Heuristic values for A* Search algorithm. Written by Arshdeep.
	public float hValue;
	public float gValue;
	
	// Pointer to the previous cell in the A* path. Written by Arshdeep.
	Position parent;
	
	public Position() {
		xCoord = 0;
		yCoord = 0;
		isObstacle = false;
		parent = null;
	}
	
	public Position(float x, float y) {
		xCoord = x;
		yCoord = y;
		isObstacle = false;
		parent = null;
	}
	
	// Method to check whether two Position objects have the same coordinates.
	public boolean equals(Position other) {
		return other.xCoord == xCoord && other.yCoord == yCoord;
	}
	
	/*
	 *  @return euclidean distance to another Position. Written by Talal.
	 */
	public float straightLineDistanceTo(Position other) {
	    float xDistance = other.xCoord - xCoord;
	    float yDistance = other.yCoord - yCoord;
		return (float) Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
	}
	
	/*
	 *  @return angle to another Position. Written by Talal.
	 */
	public float angleTo(Position other) {
		float xDistance = other.xCoord - xCoord;
	    float yDistance = other.yCoord - yCoord;
	    return (float) Math.toDegrees(Math.atan2(yDistance, xDistance));
	}
}
