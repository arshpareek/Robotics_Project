import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;

/*
 * @author Shahdad Razavi Algharavi
 */
public class MapGenerator {
	
	// Dimensions of the board.
	private int mapHeight;
	private int mapWidth;
	
	// Distance between the centre of adjeacent cells.
	private float gridSpace;
	
	// 2D array of Position objects.
	private Position[][] map;
	
	// Check if the robot is going to the parking area.
	private boolean isGoingToTarget;
	
	// True if the grren obstacle is present on the way back. Written by Jaeheon.
	private boolean greenObstacle;
	
	public MapGenerator(int height, int width, float gridSpace) {
		mapHeight = height;
		mapWidth = width;
		this.gridSpace = gridSpace;
		
		isGoingToTarget = true;
		greenObstacle = false;
		
		//Prepare the grid.
		createGrid();
		setBoundary();
		setObstacles();
	}
	
	// Return the 2D array of Position objects.
	public Position[][] getMap() {
		return map;
	}
	
	// Select which obstacle will be present on the left-hand side.
	private void chooseObstacle() {
		LCD.drawString("CHOOSE OBSTACLE:", 1, 1);
		LCD.drawString("LEFT", 1, 5);
		LCD.drawString("MID", 7, 5);
		LCD.drawString("RIGHT", 12, 5);
		LCD.drawString("<-", 2, 6);
		LCD.drawString("|O|", 7, 6);
		LCD.drawString("->", 13, 6);
		Sound.beepSequenceUp();
		while(true) {
			if (Button.LEFT.isDown()) {
				setLeftObstacle();
				break;
			}
			else if (Button.ENTER.isDown()) {
				setMiddleObstacle();
				break;
			}
			else if (Button.RIGHT.isDown()) {
				setRightObstacle();
				break;
			}
		}
		LCD.clear();
	}
	
	private void setLeftObstacle() {
		setSquare(1, 2, 14, 13);
	}
	
	private void setMiddleObstacle() {
		setSquare(2, 3, 13, 12);
	}

	private void setRightObstacle() {
		setSquare(3, 4, 12, 11);
		setObstacle(4, 10);
	}
	
	// Creates obstacle cells depending on the color sensor reading inside the parking area. Method written by Jaeheon.
	public void createReturnMap() {
		if(greenObstacle) {
			setSquare(11, 12, 4, 3);
		}
		else{
			setSquare(12, 13, 2, 1);
		}
		
	}

	// Creates a 2D array to model the board.
	private void createGrid() {
		// Determine the number of cells required in each dimension for the 2D array.
		int gridWidth = (int) (mapWidth/gridSpace);
		int gridHeight = (int) (mapHeight/gridSpace);
		
		// Create a 2D array to store map features.
		map = new Position[gridWidth][gridHeight];
		
		//Ensure the cells at the edge fall within the board boundaries.
		float clearance = gridSpace/2;
		
		//Assign Position objects with appropriate location coordinates to each cell.
		for(int y = 0; y < gridHeight; ++y) {
			for(int x = 0; x < gridWidth; ++x) {
				map[x][y] = new Position(x * gridSpace + clearance, y * gridSpace + clearance);
			}
		}
	}
	
	// Set obstacle cells on the 2D array depending on their actual placement on the board.
	private void setObstacles() {
		// Set obstacle cells at the two corner walls.
		setDiagonalLine(0, 3, 3, 0);
		setDiagonalLine(12, 15, 15, 12);
		
		//Set obstacle cells for the central block. The block thickness is over-estimated by one cell on each side to avoid collisions.
		setDiagonalLine(5, 9, 9, 5);
		setDiagonalLine(5, 10, 10, 5);
		setDiagonalLine(6, 10, 10, 6);

		// Choose the correct left-hand side obstacle at the start of a run.
		if(isGoingToTarget) {
		chooseObstacle();
		}
		
		// Set obstacle cells for the parking area box.
		setSquare(8, 11, 15, 12);
	}
	
	// Set cells at the edge of the board as obstacles to represent walls.
	private void setBoundary() {
		setStraightLine(0, map[0].length - 1, map.length - 1, map.length - 1);
		setStraightLine(map[0].length - 1, map[0].length - 1, 0, map.length - 1);
		setStraightLine(0, map[0].length - 1, 0, 0);
		setStraightLine(0, 0, 0, map.length - 1);
	}
	
	//Set obstacles in a diagonal line with coordinates of top left point(x1, y1) and bottom right(x2, y2).
	private void setDiagonalLine(int x1, int x2, int y1, int y2){
		int xIndex = x1;
		int yIndex = y1;
		while(xIndex <= x2) {
			setObstacle(xIndex, yIndex);
			++xIndex;
			--yIndex;
		}
	}
	
	//Set obstacles in a square with coordinates of top left corner (x1, y1) and bottom right(x2, y2).
	private void setSquare(int x1, int x2, int y1, int y2){
		
		
		for(int yIndex = y1; yIndex >= y2; --yIndex) {
			for(int xIndex = x1; xIndex <= x2; ++xIndex) {
				setObstacle(xIndex, yIndex);
			}
		}
	}
	
	// Set obstacles in a straight line. Either from left end (x1, y1) to the right end (x2, y2) or bottom (x1, y1) to top(x2, y2).
	private void setStraightLine(int x1, int x2, int y1, int y2){
		int xIndex = x1;
		int yIndex = y1;
		if(y1 != y2 && x1 == x2) {
			while(yIndex <= y2) {
				setObstacle(xIndex, yIndex);
				++yIndex;	
			}
		}
		
		else if(x1 != x2 && y1 == y2) {
			while(xIndex <= x2) {
				setObstacle(xIndex, yIndex);
				++xIndex;	
			}
		}
	}
	
	// Set boolean goinToTarget to true.
	public void setGoingToTarget(boolean goal) {
		isGoingToTarget = goal;
	}
	
	// Set the isObstacle field of a given cell to true.
	private void setObstacle(int x, int y) {
		map[x][y].isObstacle = true;
	} 
	
	// Set greenObstacle field to true and isGoingTarget to false. Method written by Jaeheon.
	public void setGreenObstacle() {
		greenObstacle = true;
		isGoingToTarget = false;
	}

	// Set greenObstacle field to false and isGoingTarget to false. Method written by Jaeheon
	public void setRedObstacle() {
		greenObstacle = false;
		isGoingToTarget = false;
	}
	
}
