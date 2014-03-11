import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;

/**
 * Handles the displacements of the robot, allowing it to go forward, backward and to travel to specified coordinates.
 * 
 *
 */

public class Navigation extends Thread {

	private static final int FORWARD_SPEED = 150;
	private static final int ROTATE_SPEED = 90;
	private double wheelRadius, wheelBase;
	private double theta;
	private double newAngle;
	private double currentAngle;
	private double xTravelled, yTravelled;
	private double distanceTravelled;
	private final NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;
	private Odometer odometer;
	private USPoller usPoller;
	private ObjectDetector objectDetector;
	
	public Navigation (Odometer odometer, ObjectDetector objectDetector, USPoller usPoller, double wheelRadius, double wheelBase){
		this.odometer = odometer;
		this.objectDetector = objectDetector;
		this.usPoller = usPoller;
		this.wheelRadius = wheelRadius;
		this.wheelBase = wheelBase;
	}
	
	// TravelTo method that allows navigation to take place. 
	/**
	 *Makes the robot travel to the specified coordinates (x,y)
	 * @param x the x coordinate in cm
	 * @param y the y coordinate in cm
	 */
		public void travelTo(double x, double y){

			double xDistance = x - odometer.getX();//the distance the robot has to travel in x to get to its destination from current position.
			double yDistance = y - odometer.getY();//the distance the robot has to travel in y to get to its destination from current position.
			double distance = Math.sqrt(xDistance*xDistance + yDistance*yDistance);//the overall distance the robot has to travel from current position.
			
			newAngle = Math.atan(xDistance/yDistance); // if the yDistance is negative and xDistance superior to 0, it will provide the right angle for the nxt by adding pi to it's turn angle, 
			
			currentAngle = odometer.getAng(); // gets current angle of nxt
			theta = Math.toDegrees(newAngle) - currentAngle; 
			
			// if true, turn to theta and run the trajectory without obstacleAvoidance
			LCD.drawString("theta: "+ theta, 0, 1);
			turnTo(theta);
				
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);

			leftMotor.rotate(convertDistance(wheelRadius, distance), true);
			rightMotor.rotate(convertDistance(wheelRadius, distance), false);
		}
		
		/**
		 * Helper method for the backtrack method.
		 * Travels to the specified coordinates.
		 * @param x the x coordinate provided by backtrack in cm
		 * @param y the y coordinate provided by backtrack in cm
		 */
		public void backwards(double x, double y){

			double xDistance = x - odometer.getX();//the distance the robot has to travel in x to get to its destination from current position.
			double yDistance = y - odometer.getY();//the distance the robot has to travel in y to get to its destination from current position.
			double distance = Math.sqrt(xDistance*xDistance + yDistance*yDistance);//the overall distance the robot has to travel from current position.
				
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);

			leftMotor.rotate(-convertDistance(wheelRadius, distance), true);
			rightMotor.rotate(-convertDistance(wheelRadius, distance), false);
		}
		
		/*public void travelForwardWithoutPolling(double x, double y){

			double xDistance = x - odometer.getX();//the distance the robot has to travel in x to get to its destination from current position.
			double yDistance = y - odometer.getY();//the distance the robot has to travel in y to get to its destination from current position.
			double distance = Math.sqrt(xDistance*xDistance + yDistance*yDistance);//the overall distance the robot has to travel from current position.
			
			leftMotor.setSpeed(FORWARD_SPEED);
			rightMotor.setSpeed(FORWARD_SPEED);
			leftMotor.rotate(convertDistance(wheelRadius, distance), true);
			rightMotor.rotate(convertDistance(wheelRadius, distance), false);
		}
		
		*/
		
		/**Helper method for the goForward method.
		 * Travels to the specified coordinates.
		 * 
		 * @param x the x coordinate provided by goForward in cm
		 * @param y the y coordinate provided by goForward in cm
		 */
		public void travelForward (double x, double y){

			double xDistance = x - odometer.getX();//the distance the robot has to travel in x to get to its destination from current position.
			double yDistance = y - odometer.getY();//the distance the robot has to travel in y to get to its destination from current position.
			double distance = Math.sqrt(xDistance*xDistance + yDistance*yDistance);//the overall distance the robot has to travel from current position.
		
			double xStart = odometer.getX();
			double yStart = odometer.getY();
			
			boolean isTravelling = true;
			
			while(isTravelling){
				
				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);

				leftMotor.rotate(convertDistance(wheelRadius, distance - distanceTravelled), true);
				rightMotor.rotate(convertDistance(wheelRadius, distance - distanceTravelled), true);
				
				xTravelled  =(odometer.getX() - xStart); //update xTravelled
				yTravelled  =(odometer.getY() - yStart);	//update yTravelled
				
				distanceTravelled = Math.sqrt(xTravelled*xTravelled +yTravelled*yTravelled); //update the distance travelled
				if (distanceTravelled >= distance){
					break;
				}
				if(usPoller.getFilteredData() < 15 ){ // if the robot sees an object on the path
					isTravelling = false; //set to false so we exit the loop 
				}
				LCD.drawString("x     ", 0, 5);
				LCD.drawString("x "+odometer.getX(), 0, 5);
				LCD.drawString("y     ", 0, 6);
				LCD.drawString("y "+odometer.getY(), 0, 6);
			}
			
			leftMotor.setSpeed(0);
			rightMotor.setSpeed(0);
			objectDetector.doObjectDetection();	//call object detection to do the rest
				
			}
		
		
		
		
		//changes the direction of the robot to theta degrees
		/**
		 * Helper method for the travelTo method. 
		 * Turns the robot to the specified angle in degrees.
		 * 
		 * @param theta the angle in degrees to turn to
		 */
		public void turnTo(double theta){//takes theta in degrees
			//set the motor speeds to the rotate speed
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);

			//makes the left wheel rotate forward theta
			//and the right wheel rotate backwards theta
			leftMotor.rotate(convertAngle(wheelRadius, wheelBase, theta), true);
			rightMotor.rotate(-convertAngle(wheelRadius, wheelBase, theta), false);
			
		}
		
		/*//returns true if the robot is navigating (ie not avoiding an obstacle)
		public boolean isNavigating(){
			return navigating;
			
		}
		*/
		
		// method that converts distance into wheel rotations based on wheel radius and distance.
		/**
		 * Converts distance into wheel rotations based on wheel radius and distance.
		 * @param radius the wheel radius of the robot in cm
		 * @param distance the distance to be travelled in cm
		 * @return an int number of wheel rotations
		 */
		private static int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}
		
		// method that converts the distance we want to travel to an angle based on the wheel radius and the wheel base.
		/**
		 * Converts the angle to a number of wheel rotations to determine how much the wheels have to rotate for the robot to turn to a certain angle.
		 * @param radius the wheel radius of the robot  in cm
		 * @param width the wheel base of the robot in cm
		 * @param angle the angle in degrees the robot has to rotate
		 * @return an int number of wheel rotations
		 */
		private static int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, Math.PI * width * angle / 360.0);
		}
		
		//	goForward(double distance) is boolean and always returns true allowing for the doSearch() method to be called again
		/**
		 * Makes the robot go forward a certain distance.
		 * @param distance the distance to move forward in cm
		 */
		public void goForward(double distance) {// boolean
			travelForward(odometer.getX()+Math.sin(Math.toRadians(odometer.getAng())) * distance,odometer.getY()+ Math.cos(Math.toRadians(odometer.getAng())) * distance);
			//return true;
		}
		/**
		 * Makes the robot move backward a certain distance.
		 * @param distance the distance to move backward in cm
		 */
		public void backtrack(double distance){
		backwards((odometer.getX()+Math.sin(Math.toRadians(odometer.getAng())) * distance),(odometer.getY()+ Math.cos(Math.toRadians(odometer.getAng())) * distance));
		}
		
		/*public void goForwardWithoutPolling(double distance){
			travelForwardWithoutPolling(odometer.getX()+Math.sin(Math.toRadians(odometer.getAng())) * distance,odometer.getY()+ Math.cos(Math.toRadians(odometer.getAng())) * distance);
		}
		*/

}
