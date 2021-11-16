



import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.*;
import lejos.hardware.port.*;
import lejos.utility.Delay;
import lejos.robotics.navigation.*;
import java.lang.Object.*;

import lejos.hardware.port.SensorPort;
import lejos.robotics.Color;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;

import lejos.robotics.subsumption.*;

public class LineFollow implements Behavior {
	
	public static void main(String[] args) {
		
	}
	
	//SensorMode objects for distance sensor and color sensor in RGB and Red modes.
	//private SensorMode sensorRGB;
	//private SensorMode sensorRed;
	
	//public static EV3ColorSensor color;
	private SampleProvider sensorSample;
	
	//Determines when the line following algorithm is needed.
	private boolean suppressed = false;
	
	//Objects to control the motors of the robot.
	public static EV3LargeRegulatedMotor motorA;
    public static EV3LargeRegulatedMotor motorB;
    
    //Obstacle avoidance behaviour.
    //public static Behavior oAvoid;
    
    //Variables to control line following.
    private float target = (float)0;
    private float defaultSpeed = 1;
    private float maxSpeed = 250;
    private float angle = 0;
    
	public LineFollow(SampleProvider distanceSensor, EV3LargeRegulatedMotor motorA, EV3LargeRegulatedMotor motorB) {
		
		//color = colorSensor;
		sensorSample = distanceSensor;
		this.motorA = motorA;
		this.motorB = motorB;
		//sensorRed = color.getRedMode();
		//sensorRGB = color.getRGBMode();
	}
	
	//Return true when line following algorithm is needed.
	public boolean takeControl() {
	      return true;
	   }

	//Suppress the line following algorithm when needed.
	public void suppress() {
	      suppressed = true;
	   }
	
    //Initiate line following.
	public void action() {
		
		//Signal that the robot is ready to move.
		Button.LEDPattern(4);     // flash green led and
        //Sound.beepSequenceUp();   // make sound when ready.
        //LCD.drawString("JAST", 7, 3);
        //Button.waitForAnyPress();
        
        suppressed = false;
        
        //Start the motors.
        motorA.forward();
    	motorB.forward();
    	motorA.setSpeed(defaultSpeed);
    	motorB.setSpeed(defaultSpeed);
    	
    	followLine();
	}
	
	private float distanceToObstacle() {
		float []distArr = new float [1];
    	sensorSample.fetchSample(distArr, 0);
    	return distArr[0];	
		  
	}
	
	/*
	 * //Perform obstacle avoidance. private void avoidObstacle() { motorA.stop();
	 * motorB.stop();
	 * 
	 * oAvoid.action();
	 * 
	 * //Restart motors at negligible speed to allow PID to take over later on.
	 * motorA.setSpeed(2); motorB.setSpeed(2); motorA.forward(); motorB.forward();
	 * defaultSpeed = 100; }
	 */
	
	//Return difference between measured light sample and target value.
	private float calculateError() {
		float []arr = new float [2];
		sensorSample.fetchSample(arr, 0);
    	float gyroSample = arr[1];
    	angle = arr[0];
    	System.out.println(gyroSample);
    	//Prevent irregular values.
    	//if(lightSample > 1) {lightSample = 1;}
    	//if(lightSample < 0) {lightSample = 0;}
    	
    	return gyroSample - target;
	}
	
	//Return true if robot is on red paper.
	/*
	 * private boolean robotIsOnRed() { //Measure reflected colour values. float
	 * []colorArr = new float [4]; int colorID = color.getColorID();
	 * sensorRGB.fetchSample(colorArr, 0); float redSample = colorArr[0]; float
	 * greenSample = colorArr[1]; float blueSample = colorArr[2];
	 * 
	 * //Two detection methods/conditions are used to include more shades of red
	 * //and interoperability between different color sensors. colorID returns 0 for
	 * red. return (redSample > 0.22 && blueSample < 0.14) || colorID == 0; }
	 */
	
	//Main line following algorithm which uses a PID controller.
	private void followLine() {
		
		//Initialising variables for I and D terms of the controller.
    	float integral = 0;
    	float derivative = 0;
    	float lastError = 0;
    	
    	//Setting gains for each term in the PID controller.
    	float Kp = (float) 18;
    	float Ki = (float) 2;
    	float Kd = (float) 0;
    	float Ka = (float) 0;
    	
    	//Initialising speeds for the left and right motors respectively.
    	float speedL = 1;
    	float speedR = 1;
    	
        while(!suppressed) {
        	
        	defaultSpeed = 1;
        	
        	
        	//Collect distance samples at each loop to check for the obstacle.
			/*
			 * if (distanceToObstacle() < 0.06) { avoidObstacle();
			 * 
			 * //Ensure robot stays on right side of the line. integral -= 1; }
			 */
        	
        	//Stop the robot if on the red paper, otherwise continue moving.
        	/*if(robotIsOnRed()) {
        		speedL = 1;
        		speedR = 1;
        	}
    		
        	else {*/
    		
        	float error = calculateError();
        	//Calculate integral value of all past errors.
        	//Error is scaled up by 1.7 because of large sample rate.
        	//Using I term helps use a lower Kp and hence improves stability.
        	//integral += (float) error*1.7;
        	
        	//Cap integral value to avoid wind-up.
        	if(integral >= 0.5) {integral =(float) 1;}
        	if(integral <= -0.5) {integral =(float) -1;}
        	
        	//Reduce integral term whenever robot reaches target to 
        	//avoid overshoot to the other side of the line.
        	if(error > 0 && lastError < 0) {integral = (float) (integral*0.1);}
        	if(error < 0 && lastError > 0) {integral = (float) (integral*0.1);}
        	
        	//Calculate the instantaneous change in error for d term.
        	derivative = error - lastError;
        	lastError = error;
        	
        	//Calculate the output values for the PID controller.
        	float pTerm = error*Kp;
        	float iTerm = integral*Ki;
        	float dTerm = derivative*Kd;
        	float aTerm = angle* Ka;
        	//Add P, I, and D terms to form the output.
        	float turn = pTerm + iTerm + dTerm + aTerm;
        	
        	//Prevent the final speed from being 0 or less to avoid restarting
        	//the motors.
			/*
			 * if(turn >= defaultSpeed){ turn = defaultSpeed - 1; } if(turn <= defaultSpeed
			 * * -1){ turn = -(defaultSpeed - 1); }
			 */
        	
        	//Manipulate the speeds for each wheel depending on the PID output.
        	speedL =(float) defaultSpeed - turn;
        	speedR =(float) defaultSpeed - turn;
        	
        	//Capping the maximum speed of the robot helps improve stability.
			/*
			 * if(speedL>maxSpeed){ speedL = maxSpeed; } if(speedR>maxSpeed){ speedR =
			 * maxSpeed; }
			 */
        	//Negative speeds not supported by motor framework.
        	if(speedL<=1 && speedL >= -1 && speedR<=1 && speedR >= -1){
        		speedL = 1;
        		speedR = 1;
        		motorA.setSpeed(speedL);
            	motorB.setSpeed(speedR);
        		motorA.forward();
        		motorB.backward();
        	}
        	else {
        	
        	if(speedL < -1){
        		speedL = -speedL;
        		speedR = -speedR;
        		motorA.setSpeed(speedL);
            	motorB.setSpeed(speedR);
        		motorA.backward();
        		motorB.backward();
        	}
        	else {
        		motorA.setSpeed(speedL);
            	motorB.setSpeed(speedR);
        		motorA.forward();
        		motorB.forward();
        	}
        	}
        	//}
        	
        	//Set motor speeds independently depending on PID output.
        	
        
        }
	}

}
