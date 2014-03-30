import lejos.nxt.LCD;
import lejos.nxt.UltrasonicSensor;

/**
 * Handles the ultrasonic localization using Falling Edge detection.
 * 
 *
 */
public class USLocalizer {

	private static final double ROTATION_SPEED = 50;
	private Odometer odo;
	private double currentAng;
	private TwoWheeledRobot robot;
	//private UltrasonicSensor us; // use for us.getFilteredData() if necessary
	//private USPoller usPoller;
	private UltrasonicSensor us;
	public int distance;
	private double deltaT;
	private double newAngle;

/**Constructor for the USLocalizer
 * 
 * @param odo the odometer
 * @param usPoller the usPoller
 */
	public USLocalizer(Odometer odo, UltrasonicSensor us) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.us = us;

		// switch off the ultrasonic sensor
		//us.off();
	}

	/**
	 * Performs an ultrasonic localization using falling edge detection (i.e. most of the time facing opposite the wall). 
	 * At the end of the localization the robot will be facing at an angle of 45 degrees.
	 */
	public void doUSLocalization() {
		double [] pos = new double [3];
		double angleA =0.0, angleB =0.0;
		// rotate the robot until it sees no wall
		distance = getFilteredData();
		if (distance < 50 ){ // if facing a wall
			while(distance < 50 ){ // while facing a wall
				robot.setRotationSpeed(ROTATION_SPEED);
				distance = getFilteredData();
			}
		}
		try { Thread.sleep(1000); } catch (InterruptedException e) {} //sleep time 250 ms for US
		// keep rotating until the robot sees a wall, then latch the angle

		while(distance ==50){ // while not facing a wall
			robot.setRotationSpeed(ROTATION_SPEED);
			distance = getFilteredData();
		}
		angleA = Math.abs(odo.getAng() - 360); // gets the bottom wall heading of robot 
		LCD.drawString("a:   " + angleA, 0, 6);
		// switch direction and wait until it sees no wall

		while (distance < 50){ // while facing wall
			robot.setRotationSpeed(-ROTATION_SPEED);
			distance = getFilteredData();
		}
		try { Thread.sleep(1000); } catch (InterruptedException e) {}
		// keep rotating until the robot sees a wall, then latch the angle

		if (distance == 50) { // if not facing wall
			while (distance == 50) { // while not facing wall
				robot.setRotationSpeed(-ROTATION_SPEED);
				distance = getFilteredData();
			}
			angleB = Math.abs(odo.getAng() - 360); // gets the left wall heading of the robot
			LCD.drawString("b:   " + angleB, 0, 6);
			//try { Thread.sleep(1000); } catch (InterruptedException e) {}
		}

		// angleA is clockwise from angleB, so assume the average of the
		// angles to the right of angleB is 45 degrees past 'north'

		// calculates the corrective angle
		if (angleA < angleB) { 
			deltaT = 45 - (angleA + angleB)/2;
		}
		else {
			deltaT = 225 - (angleA + angleB)/2;
		}


		currentAng = odo.getAng();
		//CHANGE TO 45 degrees
		newAngle =deltaT - 90;
		robot.turnTo(Odometer.minimumAngleFromTo(currentAng, newAngle));
		robot.setSpeeds(0.0, 0.0);

		odo.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
		
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

