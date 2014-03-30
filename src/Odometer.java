import lejos.util.Timer;
import lejos.util.TimerListener;

/**
 *Allows to track the position of the robot at all time, the x position, the y position and the angle.
 *(provided in the labs, little to no change)
 *
 *
 */
public class Odometer implements TimerListener {
	public static final int DEFAULT_PERIOD = 25;
	private TwoWheeledRobot robot;
	private Timer odometerTimer;
	// position data
	private Object lock;
	private double x, y, theta;
	private double [] oldDH, dDH;

	/**Constructor for the odometer
	 * 
	 * @param robot the TwoWheeled robot
	 * @param period the period between each update of the odometer
	 * @param start the boolean to start the odometer
	 */
	public Odometer(TwoWheeledRobot robot, int period, boolean start) {
		// initialise variables
		
		this.robot = robot;
		//this.nav = new Navigation(this);
		odometerTimer = new Timer(period, this);
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		oldDH = new double [2];
		dDH = new double [2];
		lock = new Object();
		
		// start the odometer immediately, if necessary
		if (start)
			odometerTimer.start();
	}
	
	public Odometer(TwoWheeledRobot robot) {
		this(robot, DEFAULT_PERIOD, false);
	}
	
	public Odometer(TwoWheeledRobot robot, boolean start) {
		this(robot, DEFAULT_PERIOD, start);
	}
	
	public Odometer(TwoWheeledRobot robot, int period) {
		this(robot, period, false);
	}
	
	/**
	 * Executes after each DEFAULT_PERIOD of the timer.
	 * It updates the x coordinate, y coordinate and angle.
	 */
	public void timedOut() {
		robot.getDisplacementAndHeading(dDH);
		dDH[0] -= oldDH[0];
		dDH[1] -= oldDH[1];
		
		// update the position in a critical region
		synchronized (lock) {
			theta += dDH[1];
			theta = fixDegAngle(theta);
			
			x += dDH[0] * Math.sin(Math.toRadians(theta));
			y += dDH[0] * Math.cos(Math.toRadians(theta));
		}
		
		oldDH[0] += dDH[0];
		oldDH[1] += dDH[1];
	}
	
	// accessors
	/**
	 * Puts the position of the robot in the array pos.
	 * @param pos an array to store the x coordinate, y coordinate and angle of the robot
	 */
	public void getPosition(double [] pos) {
		synchronized (lock) {
			pos[0] = x;
			pos[1] = y;
			pos[2] = theta;
		}
	}
	/**
	 * Getter for the robot.
	 * @return the robot
	 */
	public TwoWheeledRobot getTwoWheeledRobot() {
		return robot;
	}

	// mutators
	/**
	 * Setter for the position, updates the pos array.
	 * @param pos the array holding the position of the robot
	 * @param update the boolean array to select which component need to be updated
	 */
	public void setPosition(double [] pos, boolean [] update) {
		synchronized (lock) {
			if (update[0]) x = pos[0];
			if (update[1]) y = pos[1];
			if (update[2]) theta = pos[2];
		}
	}
	
	// static 'helper' methods
	/**
	 * Changes the value of the angle so that it is always between 0 degrees and 360 degrees
	 * @param angle the angle to change
	 * @return the new angle
	 */
	public static double fixDegAngle(double angle) {		
		if (angle < 0.0)
			angle = 360.0 + (angle % 360.0);
		
		return angle % 360.0;
	}
	
	/**
	 * Determines the minimum angle to get from a heading to another one
	 * @param a the current angle angle 
	 * @param b the angle we want to turn to
	 * @return the minimum angle
	 */
	public static double minimumAngleFromTo(double a, double b) {
		double d = fixDegAngle(b - a);
		
		if (d < 180.0)
			return d;
		else
			return d - 360.0;
	}


//return X value
	/**
	 * Getter for the x coordinate.
	 * @return the x coordinate in cm
	 */
	public double getX() {
		synchronized (this) {
			return x;
		}
	}

	// return Y value
	/**
	 * Getter for the y coordinate.
	 * @return the y coordinate in cm
	 */
	public double getY() {
		synchronized (this) {
			return y;
		}
	}

	// return theta value
	/**
	 * Getter for the angle of the robot.
	 * @return the angle of the robot
	 */
	public double getAng() {
		synchronized (this) {
			return theta;
		}
	}
}
