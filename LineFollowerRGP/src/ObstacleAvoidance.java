
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.robotics.subsumption.Behavior;
import lejos.utility.Delay;

public class ObstacleAvoidance implements Behavior{
	
	//Determines when obstacle avoidance is not needed.
	private boolean suppressed = false;
	
	//Collect samples from distance and color sensors.
	private SampleProvider sensorSample;
    private SensorMode sensor;
    
    //Motors attached to the robot.
    public static EV3LargeRegulatedMotor motorA;
    public static EV3LargeRegulatedMotor motorB;
    public static EV3MediumRegulatedMotor sensorMotor;
    
    //Variables to control obstacle avoidance.
    private float target = (float)0.12;
    private float speed = 160;
	
	public ObstacleAvoidance(SampleProvider distanceSample, SensorMode colorSample, EV3LargeRegulatedMotor motorA, EV3LargeRegulatedMotor motorB, EV3MediumRegulatedMotor motorS){
		sensorSample = distanceSample;
		sensor = colorSample;
		this.motorA = motorA;
		this.motorB = motorB;
		sensorMotor = motorS;
	}
	
	//Return true if obstacle avoidance is required.
	//Subsumption framework not used because of slow sample rates
    //and redundancy for two behaviours only.
	public boolean takeControl() {
		float []arr = new float [1];
    	sensorSample.fetchSample(arr, 0);
    	float distanceSample = arr[0];	
		  return distanceSample < 0.06;
		}
	
	//Suppress the obstacle avoidance algorithm when needed.
	public void suppress() {
	       suppressed = true;
	    }

	//Initiate obstacle avoidance.
	public void action() {
		Button.LEDPattern(4);     
        Sound.twoBeeps();   

        suppressed = false;
        
        motorA.synchronizeWith(new RegulatedMotor[] {motorB});
        
        initialiseObstacleAvoidance();
    	
    	avoidObstacle();
        
        findLine();

}

	private void rotateRobot(int degrees) {
		//All values are in centimetres.
		float trackLength = 11;
		float wheelDiameter = (float) 5.5;
		float rotateRatio = trackLength/wheelDiameter;
		int wheelDegrees = (int) (degrees*rotateRatio); 
		
		motorA.setSpeed(100);
        motorB.setSpeed(100);
        motorA.startSynchronization();
        
        motorA.rotate(-wheelDegrees);
        motorB.rotate(wheelDegrees);
        
        motorA.endSynchronization();
        motorA.waitComplete();
	}
	
	//Aligns robot with the line after obstacle avoidance
	//to ensure reliable transition to line following.
	private void alignRobot() {
		
		//Measure initial light intensity.
		float []colorArr = new float [1];
    	sensor.fetchSample(colorArr, 0);
    	float sample = colorArr[0];	
    	
    	//Rotate the robot ccw until it reaches close to the target light intensity.
        while(sample >= 0.6) {
        colorArr = new float [1];
    	sensor.fetchSample(colorArr, 0);
    	sample = colorArr[0];
    	motorA.setSpeed(65);
    	motorB.setSpeed(65);
    	motorA.forward();
    	motorB.backward();
		}
	}
	
	//Make the robot stationary. Stop method often takes longer to 
	//execute and is not as responsive as this alternative.
	private void stopRobot() {
		motorA.setSpeed(2);
    	motorB.setSpeed(2);
        motorA.forward();
    	motorB.forward();
	}
	
	//Prepare robot for obstacle avoidance.
	private void initialiseObstacleAvoidance() {
		//Rotate the robot by about 90 degrees before moving along the obstacle
        //and make distance sensor perpendicular to the obstacle.
        rotateRobot(83);
        sensorMotor.rotate(-90);
        
        stopRobot();
    	Delay.msDelay(400);
	}
	
	private boolean robotIsOnLine() {
		float []colorArr = new float [1];
    	sensor.fetchSample(colorArr, 0);
    	return colorArr[0] < 0.15;
    	
	}
	
	private float calculateError() {
		float []arr = new float [1];
    	sensorSample.fetchSample(arr, 0);
    	float distanceSample = arr[0];
    	
    	//Avoid large values (even infinty) when sensor goes past the obstacle.
    	if(distanceSample >= 0.21) {distanceSample = (float)0.21;}
    	
    	return  distanceSample - target;
	}
	
	//Find the line after obstacle avoidance to resume line following.
	private void findLine() {
        rotateRobot(190);
        
        //The robot would randomly dash forward after rotating. It was because the motors
        //could not switch from rotation to forward fast enough, hence the delay.
        Delay.msDelay(400);

        alignRobot();
    	
        stopRobot();
        Delay.msDelay(200);
        
        //Point the ultrasonic sensor forwards.
        sensorMotor.rotate(90);
	}
	
	//PID algorithm for obstacle avoidance. Maintains a constant distance from
	//the obstacle to move around it reliably regardless of its shape.
	private void avoidObstacle(){
		
		//Initialising variables for PID controller.
		float integral = 0;
    	float derivative = 0;
    	float lastError = 0;
    	
    	//Setting gains for the PID controller.
    	float Kp = (float) 900;
    	float Ki = (float) 12;
    	float Kd = (float) 75;
    	
    	//Repeatedly measure the error between demanded and measured distance values
    	//and provide output to the motors accordingly using a PID controller.
        while(!suppressed) {
        	
        	//Determine the difference between target distance value and the measured value.
        	float error = calculateError();
        	
        	//Add all past error values for the integral term in PID.
        	integral += error;
        	
        	//Cap integral value to minimise wind-up.
        	if(integral >= 0.7) {integral =(float) 0.7;}
        	if(integral <= -0.7) {integral =(float) -0.7;}
        	
        	//Reduce integral term whenever robot reaches target to 
        	//avoid overshoot.
        	if(error > 0 && lastError < 0) {integral = (float) (integral*0.55);}
        	if(error < 0 && lastError > 0) {integral = (float) (integral*0.45);}
        	
        	//Calculate the instantaneous change in error for d term.
        	derivative = error - lastError;
        	lastError = error;
        	
        	//Calculate the output values for the PID controller.
        	float pTerm = error*Kp;
        	float iTerm = integral*Ki;
        	float dTerm = derivative*Kd;
        	
        	//Add P, I, and D terms to form the output.
        	float turn = pTerm + iTerm + dTerm;
        	
        	//Prevent the motors from reaching negative speeds as the motor
        	//classes do not support them.
        	if(turn >= speed){
        		turn = speed - 1;
        	}
        	
        	if(turn <= speed * -1){
        		turn = -(speed - 1);
        	}
        	
        	
        	//Set motor speeds independently depending on proportional output.
        	float speedL =(float) speed + turn;
        	float speedR =(float) speed - turn;
        	
        	motorA.setSpeed(speedL);
        	motorB.setSpeed(speedR);
        	
        	
        	
        	//Check if the robot has reached the black line at each loop.
        	//If it has, suspend the obstacle avoidance and start re-aligning
        	//the robot with the line to continue line follow.
    		if(robotIsOnLine()) {
    			  Delay.msDelay(270);
    			  suppress();
    		  }
        
        }
	}

}
