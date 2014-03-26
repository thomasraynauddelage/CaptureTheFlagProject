
import lejos.nxt.LCD;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

/**
 * Takes care of searching for the flag once we are in the opponent's zone and capturing once it is found.
 *
 *
 */

public class Search {
	private final int LIGHT_BLUE=1;
	private final int RED=2;
	private final int YELLOW=3;
	private final int WHITE=4;
	private final int DARK_BLUE=5;
	public final double TILE_DISTANCE = 30.48;
	private Navigation navigation;
	private Odometer odometer;
	private ObjectDetector objectDetector;
	private TwoWheeledRobot robot;
	private NXTRegulatedMotor clawMotor;
	private UltrasonicSensor us;
	private int flagColor;
	
	
	/**Constructor for the Search.
	 * 
	 * @param nav the navigation
	 * @param odo the odometer
	 * @param robot the TwoWheeledRobot robot
	 * @param od the object detector
	 * @param clawMotor the motor used for the claw
	 */
	public Search(Navigation navigation, Odometer odometer, TwoWheeledRobot robot, ObjectDetector objectDetector, NXTRegulatedMotor clawMotor, UltrasonicSensor us, int flagColor){
		this.navigation = navigation;
		this.odometer = odometer;
		this.robot = robot;
		this.objectDetector = objectDetector;
		this.clawMotor = clawMotor;
		this.us = us;
		this.flagColor = flagColor;
		
	}
	
	/**Makes the robot travel to the bottom left corner of the zone.
	 * 
	 * @param xBottomLeft the x coordinate of the bottom left corner of the zone
	 * @param yBottomLeft the y coordinate of the bottom left corner of the zone
	 * @param xTopRight the x coordinate of the top right corner of the zone 
	 * @param yTopRight the y coordinate of the top right corner of the zone
	 */
	public void travelToZone(double xBottomLeft, double yBottomLeft, double xTopRight, double yTopRight){
		double y = odometer.getY();
		while(y < yBottomLeft){
			navigation.goForward(TILE_DISTANCE);
			if(getFilteredData() <40){
				robot.rotate(90);
				navigation.goForward(TILE_DISTANCE);
				robot.rotate(-90);
				navigation.goForward(TILE_DISTANCE);
			}
			y = odometer.getY();
		}
		
		robot.rotate(90);
		
		double x = odometer.getX();
		while(x < xBottomLeft){
			navigation.goForward(TILE_DISTANCE);
			if(getFilteredData() <40){
				robot.rotate(90);
				navigation.goForward(TILE_DISTANCE);
				robot.rotate(-90);
				navigation.goForward(TILE_DISTANCE);
				robot.rotate(-90);
				navigation.goForward(TILE_DISTANCE);
				robot.rotate(90);
			}
			x = odometer.getX();
		}
		Sound.beep();

	}

	/**
	 * Performs the search. 
	 */
	public void doSearch(){
		if(objectDetector.getObject().equals(ObjectDetector.ObjectType.FLAG)){
			robot.rotate(180);
			navigation.goForward(-15);
			clawMotor.rotate(500);
			   
		}
		else{
			objectDetector.rotateAndPoll(flagColor);
		}
		
	}
	
	private int getFilteredData() {
		int distance;
		// do a ping
		us.ping();
		//Sound.beep();
		
		// wait for the ping to complete
		try { Thread.sleep(50); } catch (InterruptedException e) {}
		
		// there will be a delay here
		distance = us.getDistance();
		if(distance > 50){ // filters values above 50 cm
			distance = 50;
		} 
		LCD.drawString("Distance:   " + distance, 0, 4);
		return distance;
	}
	
	
}

