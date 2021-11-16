import lejos.robotics.Color;
import lejos.robotics.SampleProvider;
import lejos.robotics.subsumption.Behavior;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.*;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
	
public class Task1 {
	//Color sensor and distance sensor objects.
	//public static EV3UltrasonicSensor distance;
	public static EV3GyroSensor gyro;
	
	//Sample providers to extract data from the sensors.
	public static SampleProvider sensorSample;
	public static SensorMode sensor;
	
	//The three primary motors used in the system.
	public static EV3LargeRegulatedMotor motorA;
    public static EV3LargeRegulatedMotor motorB;
    public static EV3MediumRegulatedMotor sensorMotor;
	
	public Task1(){
		//distance = new EV3UltrasonicSensor(SensorPort.S4);
		//color = new EV3ColorSensor(SensorPort.S3);
		gyro = new EV3GyroSensor(SensorPort.S3);
		gyro.reset();
		SampleProvider gyroSample = gyro.getAngleAndRateMode();
		sensorSample = gyroSample;
		//sensor = color.getRedMode();
		//color.setFloodlight(Color.RED);
		//color.setFloodlight(true);
		motorA = new EV3LargeRegulatedMotor(MotorPort.B);
		motorB = new EV3LargeRegulatedMotor(MotorPort.C);
		//sensorMotor = new EV3MediumRegulatedMotor(MotorPort.A);
	}

	public static void main(String[] args) {
		Task1 t1 = new Task1();
		//Behavior obstacleAvoidance = new ObstacleAvoidance(t1.sensorSample, t1.sensor, motorA, motorB, sensorMotor);
		Behavior lineFollow = new LineFollow(t1.sensorSample, motorA, motorB);		
	    
		//Start line following.
		lineFollow.action();

	}

}
