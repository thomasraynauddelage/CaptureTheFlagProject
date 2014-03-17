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
	private final static double WHEEL_BASE = 10.24;

	/**Main method.
	 * Creates all the objects necessary for the system to perform localization, travel to the opponent's zone, find and capture the 
	 * flag and travel to the drop off zone and leave the flag there while avoiding obstacles.
	 * It calls the localizations, then travelTo to the opponent's zone, then calls search.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
		int buttonChoice;
		//UltrasonicSensor bottomUltrasonicSensor = new UltrasonicSensor(SensorPort.S3);
		UltrasonicSensor topUltrasonicSensor = new UltrasonicSensor(SensorPort.  S2);
		//USPoller usPollerBottom = new USPoller(topUltrasonicSensor);
		USPoller usPollerTop = new USPoller(topUltrasonicSensor);
		ColorSensor lightSensor = new ColorSensor(SensorPort.S1);
		ColorSensor flagDetector = new ColorSensor(SensorPort.S4);
		TwoWheeledRobot robot = new TwoWheeledRobot(Motor.A, Motor.B, WHEEL_RADIUS, WHEEL_BASE);
		Odometer odometer = new Odometer(robot, true);
		ObjectDetector objectDetector = new ObjectDetector(flagDetector , robot, usPollerTop);
		Navigation navigation = new Navigation(odometer, objectDetector, usPollerTop, WHEEL_RADIUS, WHEEL_BASE);
		Search search = new Search(navigation, odometer, robot, objectDetector, Motor.C);
		buttonChoice = Button.waitForAnyPress();
		while (buttonChoice != Button.ID_LEFT 
				&& buttonChoice != Button.ID_RIGHT);
			
			if (buttonChoice == Button.ID_LEFT) { 
			
				USLocalizer usLocalizer = new USLocalizer(odometer, usPollerTop);
				usLocalizer.doUSLocalization();
				LightLocalizer lightLocalizer = new LightLocalizer(odometer, lightSensor, navigation);
				lightLocalizer.doLightLocalization();
				
			}

}
}
