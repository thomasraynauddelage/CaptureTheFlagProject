import lejos.nxt.LCD;
import lejos.nxt.UltrasonicSensor;

/**
 * Takes care of the ultrasonic pings for every class that requires it.
 *
 *
 */
public class USPoller {
private UltrasonicSensor us;
	/**Constructor for the ultrasonic poller
	 * 
	 * @param us the ultrasonic sensor used to get the distance to an object
	 */
	public USPoller (UltrasonicSensor us){
		this.us = us;
	}
	/**
	 * Sends a ping every 50 ms to get the distance to an object.
	 * All the readings above 50 cm are set to 50 cm.
	 * @return the distance in cm
	 */
	public synchronized int getFilteredData() {
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
		LCD.drawString("Dis:      ", 0, 4);
		LCD.drawInt(distance, 4 , 4);
		return distance;
	}

}
