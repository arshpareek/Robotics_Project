/**
 * @author Arshdeep
 */
public class Pair<S, T> {
	// Used to store a pair of probabilities.
	private S first;
	private T second;
	
	public Pair(S first, T second) {
		this.first = first;
		this.second = second;
	}
	
	public S getFirst() {
		return first;
	}
	
	public T getSecond() {
		return second;
	}
}
