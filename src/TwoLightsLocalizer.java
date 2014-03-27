import lejos.nxt.ColorSensor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;

/**Class used to do the light localization using 2 light sensors placed at a fixed distance from the axle track (3.7 cm).
 * 
 * 
 * 
 * @author Thomas Raynaud de Lage
 *
 */
public class TwoLightsLocalizer {
private final double LIGHT_SENSOR_TO_AXLE_TRACK = 3.7;
private ColorSensor leftLightSensor; 
private ColorSensor rightLightSensor; 
private TwoWheeledRobot robot;
private Odometer odometer;
private Navigation navigation;
public enum Sensor {LEFT,RIGHT, NULL};


	/**Constructor for the TwoLightsLocalizer.
	 * 
	 * @param robot the robot 
	 * @param aLightSensor the light sensor on the left of the axle track
	 * @param bLightSensor the light sensor on the right of the axle track
	 * @param odometer the odometer
	 */
	public TwoLightsLocalizer(TwoWheeledRobot robot, ColorSensor leftLightSensor, ColorSensor rightLightSensor, Odometer odometer, Navigation navigation){
		this.robot = robot;
		this.leftLightSensor = leftLightSensor;
		this.rightLightSensor = rightLightSensor;
		this.odometer = odometer;
		this.navigation = navigation;
		
	}
	
	/**Performs the light localization.
	 * Ensures the robot is positioned at {0,0,0}.
	 * 
	 */
	public void doLightLocalization(){
		robot.setForwardSpeed(10);
		goForwardAndAlign();
		robot.rotate(90);
		goForwardAndAlign();
		robot.rotate(-90);
		odometer.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
	}  
	
	/**The robot goes forward until one of the light sensors catches a line. When it does it rotates until the other light sensor finds catches the line.
	 * 
	 */
	public void goForwardAndAlign(){
		robot.setForwardSpeed(10);
		boolean firstLineFound = false;
		
		Sensor sensor = Sensor.NULL;
		
		while(!firstLineFound){
			if (leftLightSensor.getNormalizedLightValue()<435){
				sensor = Sensor.LEFT;
				firstLineFound = true;
				Sound.beep();
			}
			else if(rightLightSensor.getNormalizedLightValue()<390){
				sensor = Sensor.RIGHT;
				firstLineFound = true;
				Sound.beep();
				
			}
		}
		robot.setForwardSpeed(0);
		boolean secondLineFound = false;
		if(sensor == Sensor.LEFT){

			while(!secondLineFound){
				robot.setRightMotorSpeed(300);
				if(rightLightSensor.getNormalizedLightValue()<390){
					secondLineFound = true;
					Sound.beep();
				}
			}
		}

		else if(sensor == Sensor.RIGHT){

			while(!secondLineFound){
				robot.setLeftMotorSpeed(300);
				if(leftLightSensor.getNormalizedLightValue()<435){
					secondLineFound = true;
					Sound.beep();
				}
			}

		}

		robot.setRotationSpeed(0);
		navigation.goForward(LIGHT_SENSOR_TO_AXLE_TRACK);
		
	}
	
	public void goToNextLine(){
		robot.setForwardSpeed(10);
		boolean firstLineFound = false;

		Sensor sensor = Sensor.NULL;
		//try {
		//	Thread.sleep(2200);
		//} catch (InterruptedException e) {

		while(!firstLineFound){
			if (leftLightSensor.getNormalizedLightValue()<435){
				sensor = Sensor.LEFT;
				firstLineFound = true;
				Sound.beep();
			}
			else if(rightLightSensor.getNormalizedLightValue()<390){
				sensor = Sensor.RIGHT;
				firstLineFound = true;
				Sound.beep();

			}
		}
		robot.setForwardSpeed(0);
		boolean secondLineFound = false;
		if(sensor == Sensor.LEFT){

			while(!secondLineFound){
				robot.setRightMotorSpeed(300);
				if(rightLightSensor.getNormalizedLightValue()<390){
					secondLineFound = true;
					Sound.beep();
				}
			}
		}

		else if(sensor == Sensor.RIGHT){

			while(!secondLineFound){
				robot.setLeftMotorSpeed(300);
				if(leftLightSensor.getNormalizedLightValue()<435){
					secondLineFound = true;
					Sound.beep();
				}
			}

		}

		robot.setRotationSpeed(0);
		navigation.goForward(LIGHT_SENSOR_TO_AXLE_TRACK);


	}
}

