
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
	 * @param xTopRight the x coordinate of the top right corner of the zone 
	 * @param yTopRight the y coordinate of the top right corner of the zone
	 */
	public void travelToZone(int xBottomLeft, int yBottomLeft, int xTopRight, int yTopRight){
		int xDestination = xBottomLeft;
		int yDestination = yBottomLeft;
		if(!isObstacle() && !yReachedMax(yDestination)){
			goForwardY();
			robot.rotate(90);
			if(!isObstacle() && !xReachedMax(xDestination)){
				goForwardX();
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
			}
			else{
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
			}
		}
		else if(!yReachedMax(yDestination)){
			robot.rotate(90);
			if(!isObstacle() && !xReachedMax(xDestination)){
				goForwardX();
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
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
				travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
			}
		}
		else if(!xReachedMax(xDestination)){
			robot.rotate(90);
			if(!isObstacle()){
				goForwardX();
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
			}
			else if(!yReachedMax(yDestination)){
				robot.rotate(-90);
				travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
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
	
	
	

	/**
	 * Performs the search. 
	 */
	public void doSearch(){
		objectDetector.rotateAndPoll(flagColor);
		if(objectDetector.getObject().equals(ObjectDetector.ObjectType.FLAG)){
			//robot.rotate(objectDetector.getCorrectionAngle());
			navigation.backtrack(20);
			robot.rotate(180);
			clawMotor.rotate(-600);
			navigation.backtrack(25);
			//     robot.rotate(-30);
			//robot.rotate(30);
			clawMotor.rotate(500);


		}
		if(!objectDetector.getObject().equals(ObjectDetector.ObjectType.FLAG)){
			
			navigation.goForward(TILE_DISTANCE);
			objectDetector.rotateAndPoll(flagColor);
			if(objectDetector.getObject().equals(ObjectDetector.ObjectType.FLAG)){
				//robot.rotate(objectDetector.getCorrectionAngle());
				navigation.backtrack(20);
				robot.rotate(180);
				clawMotor.rotate(-600);
				navigation.backtrack(25);
				//robot.rotate(-30);
				//robot.rotate(30);
				clawMotor.rotate(500);
		}
		else{}
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

