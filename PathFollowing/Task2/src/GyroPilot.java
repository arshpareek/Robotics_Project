import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.utility.Delay;

/**
 * This class contains the methods which rotate and move the robot 
 * The gyroscope sensor and a PID controller are used to give accurate turns.
 * @author Mohammad Talal Hassan.
 */
public class GyroPilot {
	
	// Gyro sensor objects to collect data.
	private EV3GyroSensor gyroSensor;
	private SampleProvider gyroSampleProvider;
	
	// Motor objects for each wheel.
	private RegulatedMotor motorLeft;
	private RegulatedMotor motorRight;

	// The robot heading at the start of a run.
	private float initialAngle;
	
	// Scale-up ratio to reduce forward travel error.
	private float errorRatio;
	
	public GyroPilot(SampleProvider gyroSample, RegulatedMotor motorLeft, RegulatedMotor motorRight, EV3GyroSensor gyro) {
	gyroSampleProvider = gyroSample;
	gyroSensor = gyro;
	initialAngle = 45;
	errorRatio = (float) 1.04;
	this.motorLeft = motorLeft;
	this.motorRight = motorRight;
	motorLeft.synchronizeWith(new RegulatedMotor[] {motorRight});
	}
	
	/**
	 * Moves the robot in a straight line ahead.
	 * @param distance
	 */
	public void travel(float distance) {
		
		// Scale up distance input to mitigate travel undershoot.
		distance = distance * errorRatio;
		
		// Set desired speed for movement.
		motorLeft.setSpeed(200);
		motorRight.setSpeed(200);
		
		// Wheel diameter is used to translate travel distance to wheel rotation angle.
		float wheelDiameter = (float) 5.60;
		
		// Wheel circumference.
		float distancePerRotation = (float) (Math.PI * wheelDiameter);
		
		//Number of rotations required to move the given distance.
		float distanceRatio = (float) distance/distancePerRotation;
		float rotationAngle = (distanceRatio*360);
		
		motorLeft.startSynchronization();
		motorLeft.rotate( (int) Math.rint(rotationAngle));
		motorRight.rotate((int) Math.rint(rotationAngle));
		motorLeft.endSynchronization();
		
		motorLeft.waitComplete();
		
		motorLeft.resetTachoCount();
		motorRight.resetTachoCount();
		Delay.msDelay(400);
	}
	
	/**
	 *  Uses a PID controller to rotate the robot.
	 *  @param angle
	 */
	public void rotateTo(float angle) {
		
		// Ensure the speed of movement is capped.
		int maxSpeed = 90;
		
		// Initialise the gains for the PID controller.
		float Kp = (float) 3;
		float Ki = (float) 0.1;
		float Kd = (float) 5;
		
		// Set target angle.
		int target = (int) angle;
		
		// Measure difference between measurement and target.
		float []gyroArray = new float[2];
		gyroSampleProvider.fetchSample(gyroArray,  0);
		float sample = gyroArray[0] + 45;
		float error = target - sample;
		
		// Initialise speed variables.
		float speedL = 1;
    	float speedR = 1;
    	
    	// Initialise variables for integral and derivative terms.
    	float integral = 0;
    	float derivative = 0;
    	float lastError = 0;
    	
    	// To decide when rotation is complete.
    	boolean actionNotComplete = true;
    	int counter = 0;
    	while(actionNotComplete) {
    		
    		// Measure difference in measurement and target values.
    		error = target - getHeading();
    		
    		// Calculate proportional term.
    		float pTerm = Kp * error;

    		//  Calculate integral.
    		integral += error;
    		
    		// Anti-windup code.
        	if(integral > 20) {integral =(float) 20;}
        	if(integral < -20) {integral =(float) -20;}
        	
        	
        	//Reduce integral term whenever robot reaches target to 
        	//avoid overshoot.
        	if(error > 0 && lastError < 0) {integral = (float) (integral*0.1);}
        	if(error < 0 && lastError > 0) {integral = (float) (integral*0.1);}
    		
        	// Calculate derivative.
    		derivative = error - lastError;
    		lastError = error;
    		
    		// Calculate integral and derivative terms.
    		float dTerm = Kd * derivative;
    		float iTerm = Ki * integral;
    		
    		// Combine all terms for PID controller.
    		float turn = pTerm + iTerm + dTerm;
    		
    		// Cap the maximum output.
    		if(turn > maxSpeed) {turn = maxSpeed;}
    		if(turn < -maxSpeed) {turn = -maxSpeed;}
    		
    		speedL =(float) -turn;
        	speedR =(float) turn;
        	
        	// Since the motors do not take negative input directly, this makes 
        	// one motor go backwards if negative input is given.
        	if(speedL < -1) {
        		motorLeft.startSynchronization();
        		motorLeft.setSpeed((int) -speedL);
        		motorRight.setSpeed((int) speedR);
        		motorLeft.endSynchronization();
        		
        		motorLeft.startSynchronization();
        		motorLeft.backward();
        		motorRight.forward();
        		motorLeft.endSynchronization();
        	}
        	else if(speedR < -1) {
        		motorLeft.startSynchronization();
        		motorLeft.setSpeed((int) speedL);
        		motorRight.setSpeed((int) -speedR);
        		motorLeft.endSynchronization();
        		
        		motorLeft.startSynchronization();
        		motorLeft.forward();
        		motorRight.backward();
        		motorLeft.endSynchronization();
        		
        	}
        	else {
        		motorLeft.startSynchronization();
        		motorLeft.setSpeed(1);
        		motorRight.setSpeed(1);
        		motorLeft.endSynchronization();
        		
        		motorLeft.startSynchronization();
        		motorLeft.forward();
        		motorRight.forward();
        		motorLeft.endSynchronization();
        	}
        	
        	// If the robot has reached the target then increment counter.
        	if(error == 0) {
        		++counter;
        	}
        	
        	// If the robot has stayed at the target for 15 iterations then exit loop.
        	if(counter > 15) {
        		actionNotComplete = false;
        	}
        	Delay.msDelay(1);
    	}
    	
    	// Stop the motors simultaneously.
    	motorLeft.startSynchronization();
    	motorLeft.setSpeed(1);
		motorRight.setSpeed(1);
		motorLeft.endSynchronization();
		
		motorLeft.startSynchronization();
    	motorLeft.forward();
		motorRight.forward();
		motorLeft.endSynchronization();
		
		motorLeft.startSynchronization();
		motorLeft.stop();
		motorRight.stop();
		motorLeft.endSynchronization();
		
		motorLeft.resetTachoCount();
		motorRight.resetTachoCount();
		// Delay to safely exit rotate action without causing irregular movement.
		Delay.msDelay(400);
	}
	public void resetGyro() {
		gyroSensor.reset();
	}
	
	/**
	 * @param initialAngle
	 * @return current heading
	 */
	public float getHeading() {
		float []gyroArray = new float[2];
		gyroSampleProvider.fetchSample(gyroArray,  0);
		float gyroValue = gyroArray[0] + initialAngle;
		return gyroValue;
	}
}
	
	

