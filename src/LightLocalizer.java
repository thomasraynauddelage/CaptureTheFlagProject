import lejos.nxt.ColorSensor;
import lejos.nxt.Sound;

/**
 * Handles the light localization by clocking the gridlines.
 *
 *
 */
public class LightLocalizer {
	private final static double LIGHT_TO_CENTER = 8.5; //MODIFIED -7
	private Odometer odo;
	private TwoWheeledRobot robot;
	private ColorSensor ls;
	private Navigation navigation;
	private int lightValue;
	
	/**Constructor for the light localizer.
	 * 
	 * @param odo the odometer
	 * @param ls the light sensor used to clock the grid lines
	 * @param navigation 
	 */
	public LightLocalizer(Odometer odo, ColorSensor ls, Navigation navigation) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.ls = ls;
		this.navigation = navigation;
		
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
		//double travelToRotationPoint; // travelling distance to get to the point where lightLocalization takes place
		double correctionAngleY; // angle to add to current heading in order to get the proper heading
		double correctionAngleX;
		double correctionAngle;
		
		navigation.goForward(13);
		
		
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
			if(lightValue < 450 && polled == false){ // if it detects a line that hasn't been detected before
				headings[i] = odo.getAng(); // get heading
				polled = true; // changed value of polled to true to avoid a line from being detected twice 
				Sound.beep();
				//try { Thread.sleep(500); } catch (InterruptedException e) {}
				i++;
			}
			if(lightValue > 450){ // once passed line, set polled to false once again
				polled = false;
			}
		}
		robot.setSpeeds(0, 0);
		// stores the values in the order in which the lines are clocked in an array
		
		//MODIFIED
		xPositiveHeading = headings[0]; 
		yNegativeHeading = headings[1];
		xNegativeHeading = headings[2];
		yPositiveHeading = headings[3];
		
		// do trig to compute (0,0) and 0 degrees
		
		//Taken from the tutorial notes
		// For info on variables thetaY,thetaX,etc. refer to top of class
		thetaY = yNegativeHeading - yPositiveHeading; // calculates thetaY
		thetaX = xPositiveHeading - xNegativeHeading; // calculates thetaX
		xDisplacement = -LIGHT_TO_CENTER*Math.cos(Math.toRadians(thetaY/2)); // calculates xDisplacement
		yDisplacement = -LIGHT_TO_CENTER*Math.cos(Math.toRadians(thetaX/2)); // calculates yDisplacement
		correctionAngleX = 270 - xNegativeHeading + (thetaX/2);
		correctionAngleY = 270 - yNegativeHeading + (thetaY/2); // calculates correction angle
		correctionAngle = (correctionAngleX+correctionAngleY)/2;
		double currentAngle = odo.getAng();
		navigation.travelTo(xDisplacement, yDisplacement);
		navigation.turnTo(correctionAngleY);
		/*
		double currentAngle = odo.getAng(); // gets current angle after clocking 4 gridlines
		odo.setPosition(new double [] {xDisplacement, yDisplacement, currentAngle + correctionAngleY}, new boolean [] {true, true, true}); // sets the odometer to actual values from origin
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
		while( lightValue >450){ // will straighten the robot
			robot.setRotationSpeed(-22);
			lightValue = ls.getNormalizedLightValue();
		}
		robot.setSpeeds(0, 0);
		odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		*/
	}
	 

}
