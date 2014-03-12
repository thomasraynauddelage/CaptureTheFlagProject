import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;

/**
 * Creates all the different objects and starts the localization and the search.
 * 
 *
 */

public class CaptureTheFlag {
	private final static double WHEEL_RADIUS = 2.122;
	private final static double WHEEL_BASE = 15.53;

	/**Main method.
	 * Creates all the objects necessary for the system to perform localization, travel to the opponent's zone, find and capture the 
	 * flag and travel to the drop off zone and leave the flag there while avoiding obstacles.
	 * It calls the localizations, then travelTo to the opponent's zone, then calls search.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		int buttonChoice;
		UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S1);
		USPoller usPoller = new USPoller(ultrasonicSensor);
		ColorSensor lightSensor = new ColorSensor(SensorPort.S2);
		ColorSensor flagDetector = new ColorSensor(SensorPort.S3);
		TwoWheeledRobot robot = new TwoWheeledRobot(Motor.A, Motor.B, WHEEL_RADIUS, WHEEL_BASE);
		Odometer odometer = new Odometer(robot, true);
		ObjectDetector objectDetector = new ObjectDetector(flagDetector , robot, usPoller);
		Navigation navigation = new Navigation(odometer, objectDetector, usPoller, WHEEL_RADIUS, WHEEL_BASE);
		USLocalizer usLocalizer = new USLocalizer(odometer, usPoller);
		LightLocalizer lightLocalizer = new LightLocalizer(odometer, lightSensor);
		Search search = new Search(navigation, odometer, robot, objectDetector, Motor.C);
		buttonChoice = Button.waitForAnyPress();
		while (buttonChoice != Button.ID_LEFT 
				&& buttonChoice != Button.ID_RIGHT);
			
			if (buttonChoice == Button.ID_LEFT) { 
			
				usLocalizer.doUSLocalization();
				
				
			}

}
}
