import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.UltrasonicSensor;

/**
 * Takes care of recognizing what the object in range is using a color sensor and sets a signal for search to respond in consequence.
 * The signals are WALL, WOOD_BLOCK, and FLAG and are retrieved by getObject().
 *
 *
 */

public class ObjectDetector {

	public enum ObjectType {FLAG, OBSTACLE,NULL};
	private final int LIGHT_BLUE=1;
	private final int RED=2;
	private final int YELLOW=3;
	private final int WHITE=4;
	private final int DARK_BLUE=5;
	private final int OBSTACLE = -1;
	private ObjectType object;
	private UltrasonicSensor us;
	private ColorSensor colorSensor;
	private TwoWheeledRobot robot;
	private USPoller usPoller;

	/**Constructor for ObjectDector
	 * 
	 * @param colorSensor the color sensor used to recognize the objects
	 * @param robot	the TwoWheeledRobot robot
	 * @param usPoller the ultrasonic poller
	 */
	public ObjectDetector(ColorSensor colorSensor,
			TwoWheeledRobot robot, UltrasonicSensor us) {
		this.colorSensor = colorSensor;
		this.robot = robot;
		this.us =  us;
		object= ObjectType.NULL;
	}
	
	/**
	 *When the ultrasonic poller reports a distance under 15 cm, doObjectDetection determines if it is the flag or an obstacle.
	 *It returns an int corresponding to the object it is.
	 */
	public int doObjectDetection(){
		int theObject = 0;
		Color color = colorSensor.getColor();
		int blueComponent = color.getBlue();
		int redComponent = color.getRed();
		int greenComponent = color.getGreen();
		
		if(redComponent >2*blueComponent && redComponent >2*greenComponent){	//red block
			theObject = RED;
			
		}
		
		else if(redComponent> 2*blueComponent && redComponent > greenComponent){	//yellow block
			theObject = YELLOW;	
		}
		
		else if(redComponent>1.5*blueComponent && redComponent> greenComponent){	//obstacle
			theObject = OBSTACLE;
		}
		
		else if(blueComponent> 2*redComponent){		//dark blue block
			theObject = DARK_BLUE;
		}
		
		else if(blueComponent> redComponent){		//light blue block 
			theObject = LIGHT_BLUE;
		}
		else{	//white block
			theObject = WHITE;
		}
		
		return theObject;
		
	}
	
	/**
	 * Sets a signal for the navigation that it needs to avoid an obstacle.
	 */
	public void avoid(){
		object = ObjectType.OBSTACLE;
		
		
	}
	
	/**
	 * Sets a signal for the search that it needs to capture the flag.
	 */
	public void capture(){
		object = ObjectType.FLAG;
		
	}
	
	/**
	 * Once we are on one side of the opponent's zone we rotate to scan all the blocks on that side and if it sees the flag it calls avoid() 
	 * otherwise the robot travels to the other side and rotate and polls again. Takes an int representing the color of the flag as input.
	 */
	public void rotateAndPoll(int flagColor){
		double heading = robot.getHeading();	//get the initial heading of the robot
		
		boolean foundFlag = false;
		while (true) {
			int distance = getFilteredData();
			//LCD.drawString("Angle           ", 0, 2);
			//LCD.drawString("Angle " + robot.getHeading(), 0, 2);
			LCD.drawString("No Object Detected", 0, 0);
			LCD.drawString("          ", 0, 1);

			robot.setRotationSpeed(30);
			if (robot.getHeading() - heading >= 360) {	//if the robot has rotated and polled 360 degrees
				//LCD.drawString("Angle f   " + (robot.getHeading() - heading), 0, 2);
				break;
			} else{
				distance = getFilteredData();	//update distance
			}
			if (distance <= 15) {	//if the robot is close enough to reliably detect the object
				LCD.drawString("Object Detected   ", 0, 0);
				if(doObjectDetection() == flagColor ){
					LCD.drawString("Flag     ", 0, 1);
					foundFlag = true;
					break;

				} else {
					distance = getFilteredData();	//update distance

				}
			}

		}
		robot.setSpeeds(0,0);
		if(foundFlag){
			capture();
		}
	}
		
	
	
	/**
	 * It returns what the object is.
	 * @return the case either WALL, WOOD_BLOCK or FLAG.
	 */
	public ObjectType getObject(){
		return object;
	}
	
	private int getFilteredData() {
		int distance;
		// do a ping
		us.ping();
		// Sound.beep();

		// wait for the ping to complete
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
		}

		// there will be a delay here
		distance = us.getDistance();
		if (distance > 50) { // filters values above 50 cm
			distance = 50;
		}
		LCD.drawString("Dis:      ", 0, 4);
		LCD.drawInt(distance, 4, 4);
		return distance;
	}
	
	
}
