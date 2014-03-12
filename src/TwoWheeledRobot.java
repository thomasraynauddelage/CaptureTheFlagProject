import lejos.nxt.NXTRegulatedMotor;

/**
 * Allows the robot to be referred to as a single entity as opposed to sending messages to the 2 wheel motors.
 * (provided in the labs, little to no change)
 * 
 *
 */
public class TwoWheeledRobot {
	private NXTRegulatedMotor leftMotor;
	private NXTRegulatedMotor rightMotor;
	private double wheelRadius, wheelBase;
	private double rotationSpeed, forwardSpeed;

	/**Constructor for the TwoWheeeledRobot
	 * 
	 * @param leftMotor the motor that controls the left wheel
	 * @param rightMotor the motor that controls the right wheel
	 * @param wheelRadius the robot's wheel radius in cm
	 * @param wheelBase the robot's wheelbase in cm
	 */
	public TwoWheeledRobot(NXTRegulatedMotor leftMotor,
			NXTRegulatedMotor rightMotor,double wheelRadius,
			double wheelBase) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.wheelRadius = wheelRadius;
		this.wheelBase = wheelBase;
	}
	
	// accessors
		/**
		 * Getter for the displacement of the robot.
		 * @return the displacement in cm
		 */
		public double getDisplacement() {
			return (leftMotor.getTachoCount() * wheelRadius +
					rightMotor.getTachoCount() * wheelRadius) *
					Math.PI / 360.0;
		}
		
		/**
		 * Getter for the heading of the robot
		 * @return the heading of the robot in degrees
		 */
		public double getHeading() {
			return (leftMotor.getTachoCount() * wheelRadius -
					rightMotor.getTachoCount() * wheelRadius) / wheelBase;
		}
		
		/**
		 * Fills the data array with both the displacement in cm and the heading in degrees of the robot
		 * @param data the array to store the displacement and the heading
		 */
		public void getDisplacementAndHeading(double [] data) {
			int leftTacho, rightTacho;
			leftTacho = leftMotor.getTachoCount();
			rightTacho = rightMotor.getTachoCount();
			
			data[0] = (leftTacho * wheelRadius + rightTacho * wheelRadius) *	Math.PI / 360.0;
			data[1] = (leftTacho * wheelRadius - rightTacho * wheelRadius) / wheelBase;
		}
		
		// mutators
		
		/**
		 * Setter for the forward speed.
		 * @param speed the forward speed
		 */
		public void setForwardSpeed(double speed) {
			forwardSpeed = speed;
			setSpeeds(forwardSpeed, rotationSpeed);
		}
		/**
		 * Setter for the rotation speed.
		 * @param speed the rotation speed
		 */
		public void setRotationSpeed(double speed) {
			rotationSpeed = speed;
			setSpeeds(forwardSpeed, rotationSpeed);
		}
		
		/**
		 * Setter for the speeds, both forward and rotation speeds
		 * @param forwardSpeed the forward speed
		 * @param rotationalSpeed the rotation speed
		 */
		public void setSpeeds(double forwardSpeed, double rotationalSpeed) {
			double leftSpeed, rightSpeed;

			this.forwardSpeed = forwardSpeed;
			this.rotationSpeed = rotationalSpeed; 

			leftSpeed = (forwardSpeed + rotationalSpeed * wheelBase * Math.PI / 360.0) *
					180.0 / (wheelRadius * Math.PI);
			rightSpeed = (forwardSpeed - rotationalSpeed * wheelBase * Math.PI / 360.0) *
					180.0 / (wheelRadius * Math.PI);

			// set motor directions
			if (leftSpeed > 0.0)
				leftMotor.forward();
			else {
				leftMotor.backward();
				leftSpeed = -leftSpeed;
			}
			
			if (rightSpeed > 0.0)
				rightMotor.forward();
			else {
				rightMotor.backward();
				rightSpeed = -rightSpeed;
			}
			
			// set motor speeds
			if (leftSpeed > 900.0)
				leftMotor.setSpeed(900);
			else
				leftMotor.setSpeed((int)leftSpeed);
			
			if (rightSpeed > 900.0)
				rightMotor.setSpeed(900);
			else
				rightMotor.setSpeed((int)rightSpeed);
		}
		
		/**
		 * Makes the robot turn to the specified angle.
		 * @param theta the angle specified
		 */
		public void turnTo(double theta){//takes theta in degrees
			//set the motor speeds to the rotate speed
			setRotationSpeed(30);

			//makes the left wheel rotate forward theta
			//and the right wheel rotate backwards theta
			leftMotor.rotate(convertAngle(wheelRadius, wheelBase, theta), true);
			rightMotor.rotate(-convertAngle(wheelRadius, wheelBase, theta), false);
			setRotationSpeed(0);
		}
		
		/**
		 * Makes the robot rotate a specified angle.
		 * @param angle the angle specified
		 */
		public void rotate( int angle){
			leftMotor.setSpeed(150);
			rightMotor.setSpeed(150);

			leftMotor.rotate(convertAngle(wheelRadius, wheelBase, angle), true);
			rightMotor.rotate(-convertAngle(wheelRadius, wheelBase, angle), false);
			
		}
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
}


