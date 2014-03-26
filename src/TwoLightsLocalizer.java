import lejos.nxt.ColorSensor;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;

/**Class used to do the light localization using 2 light sensors placed at the axle track level.
 * 
 * 
 * 
 * @author Thomas Raynaud de Lage
 *
 */
public class TwoLightsLocalizer {
private final double LIGHT_SENSOR_TO_AXLE_TRACK = 3.7;
private ColorSensor aLightSensor; 
private ColorSensor bLightSensor; 
private TwoWheeledRobot robot;
private Odometer odometer;
private Navigation navigation;
public enum Sensor {A,B, NULL};


	/**Constructor for the TwoLightsLocalizer.
	 * 
	 * @param robot the robot 
	 * @param aLightSensor the light sensor on the left of the axle track
	 * @param bLightSensor the light sensor on the right of the axle track
	 * @param odometer the odometer
	 */
	public TwoLightsLocalizer(TwoWheeledRobot robot, ColorSensor aLightSensor, ColorSensor bLightSensor, Odometer odometer, Navigation navigation){
		this.robot = robot;
		this.aLightSensor = aLightSensor;
		this.bLightSensor = bLightSensor;
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
		navigation.goForward(LIGHT_SENSOR_TO_AXLE_TRACK);
		robot.rotate(90);
		goForwardAndAlign();
		navigation.goForward(LIGHT_SENSOR_TO_AXLE_TRACK);
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
			if (aLightSensor.getNormalizedLightValue()<400){
				sensor = Sensor.A;
				firstLineFound = true;
				Sound.beep();
			}
			if(bLightSensor.getNormalizedLightValue()<400){
				sensor = Sensor.B;
				firstLineFound = true;
				Sound.beep();
				
			}
		}
		robot.setForwardSpeed(0);
		boolean secondLineFound = false;
		if(sensor == Sensor.A){

			while(!secondLineFound){
				robot.setRotationSpeed(-10);
				if(bLightSensor.getNormalizedLightValue()<400){
					secondLineFound = true;
					Sound.beep();
				}
			}
		}

		else if(sensor == Sensor.B){

			while(!secondLineFound){
				robot.setRotationSpeed(10);
				if(aLightSensor.getNormalizedLightValue()<400){
					secondLineFound = true;
					Sound.beep();
				}
			}

		}

		robot.setRotationSpeed(0);
		
		
	}
}