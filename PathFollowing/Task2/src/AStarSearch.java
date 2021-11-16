import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.PriorityQueue;

import lejos.robotics.geometry.Line2D;

/*
 * @ author Arshdeep.
 */
public class AStarSearch {

	// Map of the board stored as a 2D array of Position objects.
	private Position[][] map;
	
	// Coordinates of the target location.
	private int goalX;
	private int goalY;
	
	// The target location as a Position object.
	private Position goal;
	
	// Distance between adjacent nodes in cm.
	private float gridSpace;
	
	// Open and closed lists for the use of A* Algorithm.
	private ArrayList<Position> open = new ArrayList<Position>();
	private ArrayList<Position> closed = new ArrayList<Position>();
	
	public AStarSearch(Position[][] map, float gridSpace) {
		this.map = map;
		this.gridSpace = gridSpace;
	}
	
	// Find a path from the starting position to the goal position using A* algorithm.
	public ArrayList<Position> getPath(Position startPos, float goalXCoordinate, float goalYCoordinate){
		
		// Find the closest cell to the goal position in case the goal does not lie on a node.
		goalX = coordinateToCell(goalXCoordinate);
		goalY = coordinateToCell(goalYCoordinate);
		goal = map[goalX][goalY];
		
		// Find closest cell to the starting position in case it does not lie on a node.
		Position closestCellOnGrid = getClosestCellToPosition(startPos);
		closestCellOnGrid.parent = startPos;
		
		// Add the closest cell on grid to the open list as the first waypoint.
		open.add(closestCellOnGrid);
		
		// Initial variable currPos as the closest cell on grid and set its g value to 0.
		Position currPos = closestCellOnGrid;
		currPos.gValue = 0;
		
		// Loop until either the open list is empty or the goal node is found.
		while(!open.isEmpty()) {
			
			// Choose the cheapest node from the open list and add it to the closed list.
			currPos = getSmallestHeuristic();
			closed.add(currPos);
			
			// If the chosen node is the goal, derive a path and break from the loop.
			if(currPos.equals(goal)) { 
				// If the goal node is outside the grid, add it to the path.
				Position goalPos = new Position(goalXCoordinate, goalYCoordinate);
				if(!(goalPos.equals(currPos))) {
				goalPos.parent = currPos;
				}
				else {
					goalPos = currPos;
				}
				
				// Backtrack from the goal node to find the complete shortest path. Path so obtained is reversed.
				ArrayList<Position> reversePath = backTrackPath(goalPos, closestCellOnGrid, startPos);
				
				// Reverse the backtrack path to obtain a path starting from startPos and ending at goalPos.
				Collections.reverse(reversePath);
				
				// Smoothen the path obtained by A* algorithm.
				ArrayList<Position> finalPath = getPrunedPath(reversePath);
				return finalPath;
			}
			
			// Avoid error condition.
			if(currPos.equals(map[0][0])) {
				break;
			}
			
			// Get neighbouring nodes in eight directions to the current position.
			ArrayList<Position> neighbours = getNeighbouringPositions(currPos);
			
			// Follow the standard A* procedure for each of the neighbouring node.
			for(int i = 0; i < neighbours.size(); ++i) {
				Position neighbour = neighbours.get(i);
				if(!closed.contains(neighbour) && !neighbour.isObstacle) {
					float tempG = currPos.gValue + neighbour.straightLineDistanceTo(currPos);
					
					if(open.contains(neighbour)) {
						if(tempG < neighbour.gValue) {
							neighbour.gValue = tempG;
						}
					}
					else {
						neighbour.gValue = tempG;
						neighbour.hValue = neighbour.straightLineDistanceTo(goal);
						neighbour.parent = currPos;
						open.add(neighbour);
					}
				}
			}
			
		}
		
		
		System.out.println("No Path Could Be Found");
		return new ArrayList<Position>();
	}
	
	// Once the goal node is found, follow the parent pointers until the start node is reached.
	private ArrayList<Position> backTrackPath(Position curr, Position first, Position start){
		ArrayList<Position> path = new ArrayList<Position>();
		while(curr != start) {
			path.add(curr);
			curr = curr.parent;
		}
		path.add(start);
		return path;
	}
	
	// Get neighbouring nodes in eight directions to the current position.
	private ArrayList<Position> getNeighbouringPositions(Position currPosition){
		ArrayList<Position> neighbouringSquares = new ArrayList<Position>();
		int currX = coordinateToCell(currPosition.xCoord);
		int currY = coordinateToCell(currPosition.yCoord);
		
		for(int rowOffset = -1; rowOffset <= 1; ++rowOffset ){
		     int nextRow = currY + rowOffset;
		     if(nextRow >= 0 && nextRow < map.length) {
		         for(int colOffset = -1; colOffset <= 1; ++colOffset) {
		             int nextCol = currX + colOffset;
		             if(nextCol >= 0 && nextCol < map[0].length && (rowOffset != 0 || colOffset != 0)){
		                Position neighbour = map[nextCol][nextRow];
		                neighbouringSquares.add(neighbour);
		             }
		         }
		     }
		 }
		
		return neighbouringSquares;
	}
	
	// In case a position does not lie directly at the centre of a cell, find the closest cell.
	private Position getClosestCellToPosition(Position pos) {
		return map[coordinateToCell(pos.xCoord)][coordinateToCell(pos.yCoord)];
	}
	
	// Search the open list to get the node with smallest f value and also add it to the closed list.
	private Position getSmallestHeuristic() {
		float min = 1000;
		Position minPosition = map[0][0];
		int minIndex = -1;
		for(int i=0; i < open.size(); ++i) {
			Position pos = open.get(i);
			if(pos.gValue + pos.hValue < min) {
			min = pos.gValue + pos.hValue;
			minIndex = i;
			minPosition = pos;
			}
		}
		if(minIndex != -1) {
		open.remove(minIndex);
		}
		return minPosition;
	}
	
	// Adapted from https://www.gamasutra.com/view/feature/131505/toward_more_realistic_pathfinding.php?print=1
	// Apply a smoothing algorithm to derive a shorter path from the one calulated by A* algorithm.
	private ArrayList<Position> getPrunedPath(ArrayList<Position> path) {
		ArrayList<Position> obstacles = getObstacles();
		int i = 0;
		
		for(int j = 2; j < path.size() - 1; ++j) {
			if(traversable(path.get(i), path.get(j), obstacles) && path.size() > 2) {
				path.remove(j - 1);
				--j;
			}
			else {
				++i;
			}
		}
		return path;
	}
	
	// Adapted from https://www.gamasutra.com/view/feature/131505/toward_more_realistic_pathfinding.php?print=1
	// Checks two nodes and determines whether it is safe for the robot to travel between them without collisions.
	private boolean traversable(Position start, Position end, ArrayList<Position> obstacles) {
		
		for(int i = 0; i < obstacles.size() - 1; ++i) {
			Position obstacle = obstacles.get(i);
			double distanceToObstacle = Line2D.ptSegDist(start.xCoord, start.yCoord, end.xCoord, end.yCoord, obstacle.xCoord, obstacle.yCoord);
			
			// Safe distance was chosen to be 5.3cm because it is the distance from the centre of a cell to its corner
			// and is deemed safe in an eight direction A* path finding.
			if(distanceToObstacle < 5.3) {
				return false;
			}
		}
		
		return true;
	}
	
	// Iterate through the entire 2D array of the map and return all the obstacle cells as an arraylist.
	private ArrayList<Position> getObstacles(){
		ArrayList<Position> obstacles = new ArrayList<Position>();
		for(int i = 0; i < map.length; ++i) {
			for(int j = 0; j < map[0].length; ++j) {
				Position currPosition = map[i][j];
				if(currPosition.isObstacle) {
					obstacles.add(currPosition);
				}
			}
		}
		return obstacles;
	}
	
	// Returns the closest cell index for a given coordinate number. 
	private int coordinateToCell(float coordinate) {
		return (int) ((coordinate - 3.75)/gridSpace);
	}
}
