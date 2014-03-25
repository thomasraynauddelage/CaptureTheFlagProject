import lejos.nxt.Button;
import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.SensorPort;
import lejos.nxt.UltrasonicSensor;
//import bluetooth.*;

/**
 * Creates all the different objects and starts the localization and the search.
 * 
 *
 */

public class CaptureTheFlag {
	private final static double WHEEL_RADIUS = 2.122;
	private final static double WHEEL_BASE = 15.24; //Change to 10.24

	/**Main method.
	 * Creates all the objects necessary for the system to perform localization, travel to the opponent's zone, find and capture the 
	 * flag and travel to the drop off zone and leave the flag there while avoiding obstacles.
	 * It calls the localizations, then travelTo to the opponent's zone, then calls search.
	 * 
	 * @param args
	 */
	public static void main(String[] args) {
/*BluetoothConnection conn = new BluetoothConnection();
		
		// as of this point the bluetooth connection is closed again, and you can pair to another NXT (or PC) if you wish
		
		// example usage of Tranmission class
		Transmission t = conn.getTransmission();
		if (t == null) {
			LCD.drawString("Failed to read transmission", 0, 5);
		} else {
			PlayerRole role = t.role;
			StartCorner corner = t.startingCorner;
			int greenZoneLL_X = t.greenZoneLL_X;
			int greenZoneLL_Y = t.greenZoneLL_Y;
			int redZoneLL_X = t.redZoneLL_X;
			int redZoneLL_Y = t.redZoneLL_Y;
			int greenDZone_X = t.greenDZone_X;
			int greenDZone_Y = t.greenDZone_Y;
			int redDZone_X = t.redDZone_X;
			int redDZone_Y = t.redDZone_Y;
			int greenFlag = t.greenFlag;
			int	redFlag = t.redFlag;
		
			// print out the transmission information
			conn.printTransmission();
		}
		// stall until user decides to end program
		Button.ESCAPE.waitForPress();
	*/

		
		int buttonChoice;
		int flagColor=0;
		//UltrasonicSensor bottomUltrasonicSensor = new UltrasonicSensor(SensorPort.S3);
		ColorSensor aLightSensor = new ColorSensor(SensorPort.S1);
		ColorSensor bLightSensor = new ColorSensor(SensorPort.S2);
		UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.  S3);
		//USPoller usPollerBottom = new USPoller(topUltrasonicSensor);
		USPoller usPollerTop = new USPoller(ultrasonicSensor);
		ColorSensor flagDetector = new ColorSensor(SensorPort.S4);
		TwoWheeledRobot robot = new TwoWheeledRobot(Motor.A, Motor.B, WHEEL_RADIUS, WHEEL_BASE);
		Odometer odometer = new Odometer(robot, true);
		ObjectDetector objectDetector = new ObjectDetector(flagDetector , robot, usPollerTop);
		Navigation navigation = new Navigation(odometer, objectDetector,ultrasonicSensor , WHEEL_RADIUS, WHEEL_BASE);
		Search search = new Search(navigation, odometer, robot, objectDetector, Motor.C, ultrasonicSensor, flagColor);
		buttonChoice = Button.waitForAnyPress();
		while (buttonChoice != Button.ID_LEFT 
				&& buttonChoice != Button.ID_RIGHT);
			
			if (buttonChoice == Button.ID_LEFT) { 
			
				USLocalizer usLocalizer = new USLocalizer(odometer, ultrasonicSensor);
				usLocalizer.doUSLocalization();
				TwoLightsLocalizer tll = new TwoLightsLocalizer(robot,aLightSensor,bLightSensor, odometer);
				tll.doLightLocalization();
				//search.travelToZone(91.44, 60.96, 0, 0);
			}

}
}
