/*
 * @author Jaeheon Lee.
 */
public class BayesCell {

	// Probability of the robot being located at the cell position.
	public float probability;
	// Determines the color of the line section represented by the cell.
	public boolean isBlue;
	
	BayesCell(float probability, boolean onLine){
		this.probability = probability;
		isBlue = onLine;
	}
	
	BayesCell(){
		this.probability = 0;
		isBlue = false;
	}
	
	// Set the cell to represent white space.
	public void setWhiteSpace() {
		isBlue = false;
	}
}
