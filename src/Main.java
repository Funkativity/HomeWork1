import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.RegulatedMotor;
import lejos.utility.Delay;
	
public class Main {
	
	final static double RADIUS= .0225; //RADIUS of the tires in meters
	final static double PI = 3.141592653589793;
	final static double SONAR_OFFSET = .03; //how far the sonar is from front of robut
	static double displacement = 0.0;
	
	
	public static void main(String[] args) {
		
		RegulatedMotor mA = new EV3MediumRegulatedMotor(MotorPort.A);
		RegulatedMotor mB = new EV3MediumRegulatedMotor(MotorPort.B);
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultrasensor = new EV3UltrasonicSensor(SensorPort.S4);

		mA.synchronizeWith(new RegulatedMotor[] {mB});
		
		EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
		EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S4);
		SensorMode touch = touchSensor.getTouchMode();
		SensorMode sonic = (SensorMode) ultraSensor.getDistanceMode();
		
		//TODO: Move Forward 150 cm, stop, and beep
		System.out.println("Moving forward into the great big world!");
		double distanceToGo = .50;
		double numRotations = ( distanceToGo / (RADIUS * 2 * PI));
		int angle = (int) (360.0 * numRotations);
		
		mA.startSynchronization();
		mA.rotate(angle, false);
		mB.rotate(angle, false);
		mA.endSynchronization();
		
		displacement = distanceToGo;
		Sound.beep();
		System.out.println("Waiting to proceed to next step");
		Button.ENTER.waitForPressAndRelease();
		
		
		//TODO: press button, stop when sonar reads 45 cm, beep
		System.out.println("\n\n\n\n\n\n\nSonar test!");
		float[] sonarSample = new float[sonic.sampleSize()];
		sonic.fetchSample(sonarSample, 0);
		
		mA.startSynchronization();
		System.out.println("Initial Distance to wall: " + sonarSample[0] + SONAR_OFFSET);
		if(sonarSample[0] > (.45 + SONAR_OFFSET)) {
			mA.forward();
			mB.forward();
		}
		while(sonarSample[0] > (.45 + SONAR_OFFSET)) {
			sonic.fetchSample(sonarSample, 0);
			System.out.println("Distance to wall: " + sonarSample[0] + SONAR_OFFSET);
			Delay.msDelay(750);
		}
		mA.stop();
		mB.stop();
		mA.endSynchronization();
		
		
		Sound.beep();
		Button.ENTER.waitForPressAndRelease();
		
		
		
		//TODO: press button, go until hit wall, return to 45 cm from wall
		
		
		
		mA.close();
		mB.close();
	}
	

}
