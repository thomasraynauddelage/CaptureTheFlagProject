import lejos.nxt.ColorSensor;
import lejos.nxt.Sound;

/**
 * Handles the light localization by clocking the gridlines.
 *aa
 *
 */
public class LightLocalizer {
	private final static double LIGHT_TO_CENTER = 12; 
	private Odometer odo;
	private TwoWheeledRobot robot;
	private ColorSensor ls;
	private int lightValue;
	
	/**Constructor for the light localizer.
	 * 
	 * @param odo the odometer
	 * @param ls the light sensor used to clock the grid lines
	 */
	public LightLocalizer(Odometer odo, ColorSensor ls) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.ls = ls;
		
		// turn on the light
		ls.setFloodlight(true);
	}
	
	/**
	 * Performs the light localization.
	 * First the robot travels close to the intersection, then it rotates to clock all 4 gridlines, then it positions itself at [0,0,0] and sets the odometer to that value. 
	 */
	public void doLightLocalization() {
		// drive to location listed in tutorial
		double xNegativeHeading, xPositiveHeading, yNegativeHeading, yPositiveHeading; // headings when crossing respective gridlines
		double thetaY, thetaX; // angles subtending the arc connecting the intersections of the respective y/x axis
		double xDisplacement, yDisplacement; // distance from the origin of the respective axis
		double travelToRotationPoint; // travelling distance to get to the point where lightLocalization takes place
		double correctionAngle; // angle to add to current heading in order to get the proper heading
		travelToRotationPoint = odo.getY(); // starts from odo.getY which is 0
		while(travelToRotationPoint<13){ // while loop to travel the distance
			robot.setForwardSpeed(10); 
			travelToRotationPoint = odo.getY();
		}
		robot.setForwardSpeed(0); // stops motors after breaking out of while loop
		odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true}); // odometer reset
		
		// start rotating and clock all 4 gridlines
		
		lightValue = ls.getNormalizedLightValue();
		double[] headings = new double[4]; // array to store different headings (xNegativeHeading, ...)
		int i = 0; // increment for while loop
		boolean polled = false; 
		while(i <4){ // rotate until i = 4, i.e. all 4 gridlines have been clocked
			robot.setRotationSpeed(22);
			lightValue = ls.getNormalizedLightValue();
			if(lightValue < 400 && polled == false){ // if it detects a line that hasn't been detected before
				headings[i] = odo.getAng(); // get heading
				polled = true; // changed value of polled to true to avoid a line from being detected twice 
				Sound.beep();
				i++;
			}
			if(lightValue > 400){ // once passed line, set polled to false once again
				polled = false;
			}
		}
		robot.setSpeeds(0, 0);
		// stores the values in the order in which the lines are clocked in an array
		xNegativeHeading = headings[0]; 
		yPositiveHeading = headings[1];
		xPositiveHeading = headings[2];
		yNegativeHeading = headings[3];
		
		// do trig to compute (0,0) and 0 degrees
		
		//Taken from the tutorial notes
		// For info on variables thetaY,thetaX,etc. refer to top of class
		thetaY = yNegativeHeading - yPositiveHeading; // calculates thetaY
		thetaX = xPositiveHeading - xNegativeHeading; // calculates thetaX
		xDisplacement = -LIGHT_TO_CENTER*Math.cos(Math.toRadians(thetaY/2)); // calculates xDisplacement
		yDisplacement = -LIGHT_TO_CENTER*Math.cos(Math.toRadians(thetaX/2)); // calculates yDisplacement
		correctionAngle = 270 - yNegativeHeading + (thetaY/2); // calculates correction angle
		
		
		double currentAngle = odo.getAng(); // gets current angle after clocking 4 gridlines
		odo.setPosition(new double [] {xDisplacement, yDisplacement, currentAngle + correctionAngle}, new boolean [] {true, true, true}); // sets the odometer to actual values from origin
		double finalAngle = Math.atan((xDisplacement/yDisplacement)) + 45; // angle to rotate in order to point towards the origin
		double distance = Math.sqrt(xDisplacement*xDisplacement + yDisplacement*yDisplacement); // magnitude of distance to travel in order to reach the origin
		robot.turnTo(Odometer.minimumAngleFromTo(odo.getAng(), finalAngle)); // turn to the final angle from the current one
		odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true}); // reset odometer for travel loop
		double travel = odo.getY(); // travel starts from 0
		while(travel<distance){ 
			robot.setForwardSpeed(10);
			travel = odo.getY();
		}
		robot.setSpeeds(0,0);
		
		// when done travel to (0,0) and turn to 0 degrees
		
		// once on the origin, rotate counterclockwise until the robot reaches a gridline
		lightValue = ls.getNormalizedLightValue();
		while( lightValue >400){ // will straighten the robot
			robot.setRotationSpeed(-22);
			lightValue = ls.getNormalizedLightValue();
		}
		robot.setSpeeds(0, 0);
		odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		
	}
	 

}
