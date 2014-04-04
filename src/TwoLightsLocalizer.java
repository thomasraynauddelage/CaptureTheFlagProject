import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
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
private int leftFirst;
private int leftSecond;
private int leftThird;
private int rightFirst;
private int rightSecond;
private int rightThird;



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
		leftFirst = leftLightSensor.getNormalizedLightValue();
		leftSecond = leftLightSensor.getNormalizedLightValue();
		leftThird = leftLightSensor.getNormalizedLightValue();
		rightFirst = rightLightSensor.getNormalizedLightValue();
		rightSecond = rightLightSensor.getNormalizedLightValue();
		rightLightSensor.getNormalizedLightValue();
		
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
		goBackwardAndAlign();
		odometer.setPosition(new double [] {0.0, 0.0, 0.0}, new boolean [] {true, true, true});
	}  
	
	/**The robot moves forward until one of the light sensors catches a line. When it does it rotates until the other light sensor finds catches the line.
	 * 
	 */
	public void goForwardAndAlign(){
		robot.setForwardSpeed(10);
		boolean firstLineFound = false;
		
		Sensor sensor = Sensor.NULL;
		
		while(!firstLineFound){
			//if (leftLightSensor.getNormalizedLightValue()<435){
			if(leftSensorFoundLine()){
				sensor = Sensor.LEFT;
				firstLineFound = true;
				Sound.beep();
			}
			//else if(rightLightSensor.getNormalizedLightValue()<390){
			else if(rightSensorFoundLine()){
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
				//if(rightLightSensor.getNormalizedLightValue()<390){
				if(rightSensorFoundLine()){
					secondLineFound = true;
					Sound.beep();
				}
			}
		}

		else if(sensor == Sensor.RIGHT){

			while(!secondLineFound){
				robot.setLeftMotorSpeed(300);
				//if(leftLightSensor.getNormalizedLightValue()<435){
				if(leftSensorFoundLine()){
					secondLineFound = true;
					Sound.beep();
				}
			}

		}

		robot.setRotationSpeed(0);
		navigation.goForwardWithoutPolling(LIGHT_SENSOR_TO_AXLE_TRACK);
		
	}
	
	public void goToNextLine(){
		robot.setForwardSpeed(10);
		boolean firstLineFound = false;

		Sensor sensor = Sensor.NULL;
		//try {
		//	Thread.sleep(2200);
		//} catch (InterruptedException e) {

		while(!firstLineFound){
			//if (leftLightSensor.getNormalizedLightValue()<435){
			if(leftSensorFoundLine()){
				sensor = Sensor.LEFT;
				firstLineFound = true;
				Sound.beep();
			}
			//else if(rightLightSensor.getNormalizedLightValue()<390){
			else if (rightSensorFoundLine()){
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
				//if(rightLightSensor.getNormalizedLightValue()<390){
				if(rightSensorFoundLine()){
					secondLineFound = true;
					Sound.beep();
				}
			}
			robot.setRightMotorSpeed(0);
		}

		else if(sensor == Sensor.RIGHT){

			while(!secondLineFound){
				robot.setLeftMotorSpeed(300);
				//if(leftLightSensor.getNormalizedLightValue()<435){
				if(leftSensorFoundLine()){
					secondLineFound = true;
					Sound.beep();
				}
			}
			robot.setLeftMotorSpeed(0);
		}

		navigation.goForwardWithoutPolling(LIGHT_SENSOR_TO_AXLE_TRACK);

	}
	
	/**The robot moves backward until one of the light sensors catches a line. When it does it rotates until the other light sensor finds catches the line.
	 * 
	 */
	public void goBackwardAndAlign(){
		robot.setForwardSpeed(-10);
		boolean firstLineFound = false;

		Sensor sensor = Sensor.NULL;
		
		while(!firstLineFound){
			//if (leftLightSensor.getNormalizedLightValue()<435){
			if(leftSensorFoundLine()){
				sensor = Sensor.LEFT;
				firstLineFound = true;
				Sound.beep();
			}
			//else if(rightLightSensor.getNormalizedLightValue()<390){
			else if (rightSensorFoundLine()){
				sensor = Sensor.RIGHT;
				firstLineFound = true;
				Sound.beep();

			}
		}
		robot.setForwardSpeed(0);
		boolean secondLineFound = false;
		if(sensor == Sensor.LEFT){

			while(!secondLineFound){
				robot.setRightMotorSpeed(-300);
				//if(rightLightSensor.getNormalizedLightValue()<390){
				if(rightSensorFoundLine()){
					secondLineFound = true;
					Sound.beep();
				}
			}
			robot.setRightMotorSpeed(0);
		}

		else if(sensor == Sensor.RIGHT){

			while(!secondLineFound){
				robot.setLeftMotorSpeed(-300);
				//if(leftLightSensor.getNormalizedLightValue()<435){
				if(leftSensorFoundLine()){
					secondLineFound = true;
					Sound.beep();
				}
			}
			robot.setLeftMotorSpeed(0);
		}
		navigation.goForwardWithoutPolling(LIGHT_SENSOR_TO_AXLE_TRACK);
	}
	
	/*public boolean leftSensorFoundLine(){
		boolean found = false;
		int lightValues[] = new int [4];
		lightValues[0] = leftFirst;
		lightValues[1] = leftSecond;
		lightValues[2] =leftThird;
		lightValues[3] = leftLightSensor.getNormalizedLightValue();
		leftFirst = lightValues[1];
		leftSecond = lightValues[2];
		leftThird = lightValues[3];
		if(lightValues[0] - lightValues[1] < -4 && lightValues[1] - lightValues[2] < -4 && lightValues[2] -lightValues[3]<-4){
			found = true;
		}
		return found;
	}
	
	public boolean rightSensorFoundLine(){
		boolean found = false;
		int lightValues[] = new int [4];
		lightValues[0] = rightFirst;
		lightValues[1] = rightSecond;
		lightValues[2] = rightThird;
		lightValues[3] = rightLightSensor.getNormalizedLightValue();
		rightFirst = lightValues[1];
		rightSecond = lightValues[2];
		rightThird = lightValues[3];
		if(lightValues[0] - lightValues[1] < -4 && lightValues[1] - lightValues[2] < -4 && lightValues[2] -lightValues[3]<-4){
			found = true;
		}
		return found;
	}
	*/
	
	
	public boolean leftSensorFoundLine(){
		boolean found = false;
		int lightValues[] = new int [3];
		for(int i = 0; i <3 ; i++){
			lightValues[i] = leftLightSensor.getNormalizedLightValue();	
			LCD.drawString(""+i+": "+ lightValues[i], 0, i +3);
			
		}
		if(lightValues[0] - lightValues[1] < -3 && lightValues[1] - lightValues[2] < -4 ){
			found = true;
		}
		return found;
	}
	
	
	public boolean rightSensorFoundLine(){
		boolean found = false;
		int lightValues[] = new int [3];
		for(int i = 0; i < 3; i++){
			lightValues[i] = rightLightSensor.getNormalizedLightValue();	
		}
		if(lightValues[0] - lightValues[1] < -3 && lightValues[1] - lightValues[2] < -4){
			found = true;
		}
		return found;
	}
	
}

