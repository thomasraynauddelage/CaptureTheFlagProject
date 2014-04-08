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
	private final static double WHEEL_BASE = 16.28; 
	private static PlayerRole role;
	private static StartCorner corner;
	private static int greenZoneLL_X;
	private static int greenZoneLL_Y;
	private static int greenZoneUR_X;
	private static int greenZoneUR_Y;
	private static int redZoneLL_X;
	private static int redZoneLL_Y;
	private static int redZoneUR_X;
	private static int redZoneUR_Y;
	private static int greenDZone_X;
	private static int greenDZone_Y;
	private static int redDZone_X;
	private static int redDZone_Y;
	private static int redFlag;
	private static int greenFlag;
	private static int opponentZoneLL_X;
	private static int opponentZoneLL_Y;
	private static int opponentZoneUR_X;
	private static int opponentZoneUR_Y;
	private static int dZone_X;
	private static int dZone_Y;
	private static int flagColor;
	

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
			role = t.role;
			corner = t.startingCorner;

			if(corner.getId() == 1){
				greenZoneLL_X = t.greenZoneLL_X;
				greenZoneLL_Y = t.greenZoneLL_Y;
				greenZoneUR_X = t.greenZoneUR_X;
				greenZoneUR_Y = t.greenZoneUR_Y;
				redZoneLL_X = t.redZoneLL_X;
				redZoneLL_Y = t.redZoneLL_Y;
				redZoneUR_X = t.redZoneUR_X;
				redZoneUR_Y = t.redZoneUR_Y;
				greenDZone_X = t.greenDZone_X;
				greenDZone_Y = t.greenDZone_Y;
				redDZone_X = t.redDZone_X;
				redDZone_Y = t.redDZone_Y;
			}
			else if (corner.getId() == 2){
				 greenZoneLL_X = t.greenZoneLL_Y;
				 greenZoneLL_Y =  10 - t.greenZoneUR_X;
				 greenZoneUR_X = t.greenZoneUR_Y;
				 greenZoneUR_Y = 10 - t.greenZoneLL_X;
				 redZoneLL_X = t.redZoneLL_Y;
				 redZoneLL_Y = 10 - t.redZoneUR_X;
				 redZoneUR_X = t.redZoneUR_Y;
				 redZoneUR_Y = 10 - t.redZoneLL_X;
				 greenDZone_X = t.greenDZone_Y;
				 greenDZone_Y = 10 - t.greenDZone_X -1;
				 redDZone_X = t.redDZone_Y;
				 redDZone_Y = 10 - t.redDZone_X -1;		


			}
			else if (corner.getId() == 3){
				greenZoneLL_X = 10 - t.greenZoneUR_Y;
				 greenZoneLL_Y  = t.greenZoneLL_X;
				 greenZoneUR_X = 10 - t.greenZoneLL_Y;
				 greenZoneUR_Y =  t.greenZoneUR_X;
				 redZoneLL_X = 10 - t.redZoneUR_Y;
				 redZoneLL_Y = t.redZoneLL_X;
				redZoneUR_X = 10 - t.redZoneLL_Y;
				 redZoneUR_Y = t.redZoneUR_X;
				 greenDZone_X = 10 - t.greenDZone_Y - 1;
				greenDZone_Y = t.greenDZone_X;
				 redDZone_X = 10 - t.redDZone_Y -1;
				redDZone_Y = t.redDZone_X;

			} 
			else if (corner.getId() == 4){
				greenZoneLL_X = 10 - t.greenZoneUR_X;
				 greenZoneLL_Y = 10 - t.greenZoneUR_Y;
				 greenZoneUR_X = 10 - t.greenZoneLL_X;
				 greenZoneUR_Y = 10 - t.greenZoneUR_Y;
				 redZoneLL_X = 10 - t.redZoneUR_X;
				 redZoneLL_Y = 10 - t.redZoneUR_Y;
				 redZoneUR_X = 10 - t.redZoneLL_X;
				 redZoneUR_Y = 10 - t.redZoneLL_Y;
				 greenDZone_X = 10 - t.greenDZone_X -1;
				 greenDZone_Y = 10 - t.greenDZone_Y -1;
				 redDZone_X = 10 - t.redDZone_X - 1;
				 redDZone_Y = 10 - t.redDZone_Y -1;

			}

			 greenFlag = t.greenFlag;
				redFlag = t.redFlag;
			// print out the transmission information
			//conn.printTransmission();
		}
		// stall until user decides to end program
		//Button.ESCAPE.waitForPress();
		


		//int buttonChoice;
		if (role.getId() == 1){
			opponentZoneLL_X = redZoneLL_X;
			opponentZoneLL_Y = redZoneLL_Y;
			opponentZoneUR_X = redZoneUR_X;
			opponentZoneUR_Y = redZoneUR_Y;
			flagColor = redFlag;
			dZone_X = greenDZone_X;
			dZone_Y = greenDZone_Y;
			
		}
		else{
			opponentZoneLL_X = greenZoneLL_X;
			opponentZoneLL_Y = greenZoneLL_Y;
			opponentZoneUR_X = greenZoneUR_X;
			opponentZoneUR_Y = greenZoneUR_Y;
			flagColor = greenFlag;
			dZone_X = redDZone_X;
			dZone_Y = redDZone_Y;
			
		}
		*/
		flagColor = 3;

		ColorSensor aLightSensor = new ColorSensor(SensorPort.S1);
		ColorSensor bLightSensor = new ColorSensor(SensorPort.S2);
		UltrasonicSensor ultrasonicSensor = new UltrasonicSensor(SensorPort.S4);
		ColorSensor flagDetector = new ColorSensor(SensorPort.S3);
		TwoWheeledRobot robot = new TwoWheeledRobot(Motor.A, Motor.B, WHEEL_RADIUS, WHEEL_BASE);
		Odometer odometer = new Odometer(robot, true);
		Navigation navigation = new Navigation(odometer,ultrasonicSensor , WHEEL_RADIUS, WHEEL_BASE);
		ObjectDetector objectDetector = new ObjectDetector(flagDetector , robot, ultrasonicSensor, navigation);

		//buttonChoice = Button.waitForAnyPress();
		//while (buttonChoice != Button.ID_LEFT 
		//&& buttonChoice != Button.ID_RIGHT);

		//if (buttonChoice == Button.ID_LEFT) { 
		LCDInfo lcd = new LCDInfo(odometer);
		USLocalizer usLocalizer = new USLocalizer(odometer, ultrasonicSensor);
		usLocalizer.doUSLocalization();
		TwoLightsLocalizer tll = new TwoLightsLocalizer(robot,aLightSensor,bLightSensor, odometer, navigation);
		tll.doLightLocalization();
		Search search = new Search(navigation, odometer, robot, objectDetector, Motor.C, ultrasonicSensor, flagColor, tll);
		search.travelToZone(4, 4);
		//search.travelToZone(opponentZoneLL_X, opponentZoneLL_Y);
		search.doSearch(4,4,6,7);
		//search.doSearch(opponentZoneLL_X, opponentZoneLL_Y, opponentZoneUR_X, opponentZoneUR_Y);
		//search.travelToDropOff(dZone_X, dZone_Y);
		search.travelToDropOff(4, 4, 2, 2);
	}

}

