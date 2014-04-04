
import lejos.nxt.LCD;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

/**
 * Takes care of traveling to the zone and then searching for the flag once we are in the opponent's zone and capturing once it is found.
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
	private TwoLightsLocalizer tll;
	private int y = 0;	// y increment
	private int x = 0;	//x increment
	
	
	/**Constructor for the Search.
	 * 
	 * @param nav the navigation
	 * @param odo the odometer
	 * @param robot the TwoWheeledRobot robot
	 * @param od the object detector
	 * @param clawMotor the motor used for the claw
	 */
	public Search(Navigation navigation, Odometer odometer, TwoWheeledRobot robot, ObjectDetector objectDetector, NXTRegulatedMotor clawMotor, UltrasonicSensor us, int flagColor, TwoLightsLocalizer tll){
		this.navigation = navigation;
		this.odometer = odometer;
		this.robot = robot;
		this.objectDetector = objectDetector;
		this.clawMotor = clawMotor;
		this.us = us;
		this.flagColor = flagColor;	//change to flag color
		this.tll = tll;
		
	}
	
	/**Makes the robot travel to the bottom left corner of the zone.
	 * 
	 * @param xBottomLeft the x coordinate of the bottom left corner of the zone
	 * @param yBottomLeft the y coordinate of the bottom left corner of the zone
	 */
	public void travelToZone(int xBottomLeft, int yBottomLeft){
		int xDestination = xBottomLeft;
		int yDestination = yBottomLeft;
	
		if(!isObstacle() && !yReachedMax(yDestination)){
			goForwardY();
			robot.rotate(90);
			if(!isObstacle() && !xReachedMax(xDestination)){
				tll.goBackwardAndAlign();
				goForwardX();
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft);
			}
			else{
				tll.goBackwardAndAlign();   
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft);
			}
		}
		else if(!yReachedMax(yDestination)){
			tll.goBackwardAndAlign();
			robot.rotate(90);
			if(!isObstacle() && !xReachedMax(xDestination)){
				goForwardX();
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft);
			}
			else if (!xReachedMax(xDestination)){
				boolean obstacle = true;
				while(obstacle){
					robot.rotate(90);
					goBackwardY();
					robot.rotate(-90);
					obstacle = isObstacle();
				}
				goForwardX();
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft);
			}
			else{
				robot.rotate(-90);
				goBackwardX();
				robot.rotate(90);
				travelToZone(xBottomLeft, yBottomLeft);
			}
		}
		else if(!xReachedMax(xDestination)){
			robot.rotate(90);
			tll.goBackwardAndAlign();
			if(!isObstacle()){
				goForwardX();
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft);
			}
			else if(!yReachedMax(yDestination)){
				robot.rotate(-180);
				travelToZone(xBottomLeft, yBottomLeft);
			}       
		}
	}
	
	/**Helper method for travelToZone.
	 * Goes forward x, updates the x tile counter and corrects the odometry.
	 * 
	 */
	public void goForwardX(){
		tll.goToNextLine();	//go forward one tile in x
		x++;	// increment x tile counter
		odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 90}, new boolean [] {true, false, true});	//correct odometry
	}
	
	/**Helper method for travelToZone.
	 * Goes forward y, updates the y tile counter and corrects the odometry.
	 * 
	 */
	public void goForwardY(){
		tll.goToNextLine();	//go forward one tile in y
		y++;	//increment y tile counter
		odometer.setPosition(new double [] {0.0, y*TILE_DISTANCE, 0}, new boolean [] {false, true, true});	//correct odometry
	}
	
	/**Helper method for travelToZone.
	 * Goes backward x, updates the x tile counter and corrects the odometry.
	 * 
	 */
	public void goBackwardX(){
		tll.goToNextLine();	//go backward one tile in x
		x--;	// decrement x tile counter
		odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 270}, new boolean [] {true, false, true});	//correct odometry
	}
	
	/**Helper method for travelToZone.
	 * Goes backward y, updates the y tile counter and corrects the odometry.
	 * 
	 */
	public void goBackwardY(){
		tll.goToNextLine();	//go backward one tile in y
		y--;	// decrement y tile counter
		odometer.setPosition(new double [] {0.0, y*TILE_DISTANCE , 180}, new boolean [] {false, true, true});	//correct odometry
	}

	/**Checks if there is an object at less than 40 cm.
	 * 
	 * @return a boolean (true if obstacle, false otherwise)
	 */
	public boolean isObstacle(){
		if(getFilteredData() < 40){
			return true;
		}
		else return false;
	}
	
	/**Checks if the robot has reached its final x destination.
	 * 
	 * @param xDestination the final x destination.
	 * @return a boolean (true if the final x destination has been reached, false otherwise)
	 */
	public boolean xReachedMax(int xDestination){
		if (x == xDestination){
			return true;
		}
		else return false;
	}
	
	/**Checks if the robot has reached its final y destination.
	 * 
	 * @param yDestination the final y destination
	 * @return a boolean (true if the final y destination is reached, false otherwise)
	 */
	public boolean yReachedMax(int yDestination){
		if (y == yDestination){
			return true;
		}
		else return false;
	}
	
	
	

	/**Performs the search once the zone is reached.
	 * 
	 * @param xBottomLeft
	 * @param yBottomLeft
	 * @param xTopRight
	 * @param yTopRight
	 */
	public void doSearch(int xBottomLeft, int yBottomLeft, int xTopRight, int yTopRight){
		int xChecks = xTopRight - xBottomLeft -1;
		int yChecks = yTopRight - yBottomLeft -1;
		robot.rotate(90);
		//navigation.goForwardWithoutPolling(TILE_DISTANCE);
		tll.goToNextLine();
		xChecks --;
		robot.rotate(-90);
		//navigation.goForwardWithoutPolling(TILE_DISTANCE);
		tll.goToNextLine();
		yChecks --;
		objectDetector.rotateAndPoll(flagColor);
		if(objectDetector.getObject().equals(ObjectDetector.ObjectType.FLAG)){
			navigation.backtrack(20);
			robot.rotate(180);
			clawMotor.rotate(-600);
			navigation.backtrack(25);
			clawMotor.rotate(500);
			robot.rotate(180);
			navigation.backtrack(5);
		}
		else {
			while (xChecks !=0 || yChecks !=0){
				if(yChecks != 0){
					//navigation.goForwardWithoutPolling(TILE_DISTANCE);
					robot.setForwardSpeed(5);
					//navigation.goForwardWithoutPolling(3);
					tll.goBackwardAndAlign();
					tll.goToNextLine();
					yChecks --;
					objectDetector.rotateAndPoll(flagColor);
					if(objectDetector.getObject().equals(ObjectDetector.ObjectType.FLAG)){
						navigation.backtrack(20);
						robot.rotate(180);
						clawMotor.rotate(-600);
						navigation.backtrack(25);
						clawMotor.rotate(500);
						robot.rotate(180);
						navigation.backtrack(5);
					}
				}
			}
			robot.setForwardSpeed(5);
			//navigation.goForwardWithoutPolling(3);
			tll.goBackwardAndAlign();
		}
	}
	
	public void travelToDropOff(int x, int y){
	
		
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

