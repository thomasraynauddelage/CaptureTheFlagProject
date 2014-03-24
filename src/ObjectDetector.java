import lejos.nxt.ColorSensor;
import lejos.nxt.ColorSensor.Color;
import lejos.nxt.UltrasonicSensor;

/**
 * Takes care of recognizing what the object in range is using a color sensor and sets a signal for search to respond in consequence.
 * The signals are WALL, WOOD_BLOCK, and FLAG and are retrieved by getObject().
 *
 *
 */

public class ObjectDetector {

	public enum ObjectType {WALL, WOOD_BLOCK, FLAG};
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
			TwoWheeledRobot robot, USPoller usPoller) {
		this.colorSensor = colorSensor;
		this.robot = robot;
		this.usPoller = usPoller;
	}
	
	/**
	 *When the ultrasonic poller reports a distance under 15 cm, doObjectDetection determines if it is the flag or an obstacle.
	 *If it is the block it calls capture(), otherwise it calls avoid().
	 */
	public void doObjectDetection(){
		Color color = colorSensor.getColor();
		int blueComponent = color.getBlue();
		int redComponent = color.getRed();
		int greenComponent = color.getGreen();
		
		
	}
	
	//public void pathInterrupt(){
		
	//}
	
	/**
	 * Sets a signal for the navigation that it needs to avoid an obstacle.
	 */
	public void avoid(){
		
	}
	
	/**
	 * Sets a signal for the search that it needs to capture the flag.
	 */
	public void capture(){
		
	}
	
	/**
	 * Once we are on one side of the opponent's zone we rotate to scan all the blocks on that side and if it sees the flag it calls avoid() 
	 * otherwise the robot travels to the other side and rotate and polls again.
	 */
	public void rotateAndPoll(){
		
	}
	
	/**
	 * It returns what the object is.
	 * @return the case either WALL, WOOD_BLOCK or FLAG.
	 */
	public ObjectType getObject(){
		return object;
	}
	
	
}
