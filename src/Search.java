
import lejos.nxt.LCD;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

/**
 * Takes care of searching for the flag once we are in the opponent's zone and capturing once it is found.
 *
 *
 */

public class Search {
	private final int LIGHT_BLUE=1;
	private final int RED=2;
	private final int YELLOW=3;
	private final int WHITE=4;
	private final int DARK_BLUE=5;
	public final double TILE_DISTANCE = 30.48;
	private Navigation navigation;
	private Odometer odometer;
	private ObjectDetector objectDetector;
	private TwoWheeledRobot robot;
	private NXTRegulatedMotor clawMotor;
	private UltrasonicSensor us;
	private int flagColor;
	private TwoLightsLocalizer tll;
	private int y = 0;	// y increment
	private int x = 0;	//x increment
	
	
	/**Constructor for the Search.
	 * 
	 * @param nav the navigation
	 * @param odo the odometer
	 * @param robot the TwoWheeledRobot robot
	 * @param od the object detector
	 * @param clawMotor the motor used for the claw
	 */
	public Search(Navigation navigation, Odometer odometer, TwoWheeledRobot robot, ObjectDetector objectDetector, NXTRegulatedMotor clawMotor, UltrasonicSensor us, int flagColor, TwoLightsLocalizer tll){
		this.navigation = navigation;
		this.odometer = odometer;
		this.robot = robot;
		this.objectDetector = objectDetector;
		this.clawMotor = clawMotor;
		this.us = us;
		this.flagColor = flagColor;	//change to flag color
		this.tll = tll;
		
	}
	
	/**Makes the robot travel to the bottom left corner of the zone.
	 * 
	 * @param xBottomLeft the x coordinate of the bottom left corner of the zone
	 * @param yBottomLeft the y coordinate of the bottom left corner of the zone
	 * @param xTopRight the x coordinate of the top right corner of the zone 
	 * @param yTopRight the y coordinate of the top right corner of the zone
	 */
	public void travelToZone(int xBottomLeft, int yBottomLeft, int xTopRight, int yTopRight){
		//int xDestination = (xBottomLeft + xTopRight)/2;
		//int yDestination = (yBottomLeft + yTopRight)/2;
		int xDestination = xBottomLeft;
		int yDestination = yBottomLeft;
		if(y < yDestination){	//if the y position of the robot is not the final y position 
			if(getFilteredData() > 40){		//if there is not obstacle in the y direction
				tll.goToNextLine();	//go forward one tile in y
				y++;	//increment y tile counter
				odometer.setPosition(new double [] {0.0, y*TILE_DISTANCE, 0.0}, new boolean [] {false, true, false});	//correct odometry
				robot.rotate(90);	//rotate to face positive x direction

				if(x < xDestination && getFilteredData() > 40){	//if the x position of the robot is not the final x position and there is no obstacle 
					tll.goToNextLine();	//go forward one tile in x
					x++;	// increment x tile counter
					odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 0.0}, new boolean [] {true, false, false});	//correct odometry
					robot.rotate(-90);	//rotate to face positive y direction
					travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);	//recursive call
				}
				else if (x< xDestination){	//if the x position of the robot is not the final x position but there is an obstacle
					if(y< yDestination){	//if the y position of the robot is not the final y position
						robot.rotate(-90);	//rotate to face positive y direction
						travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);	//recursive call
					}
					else{	//if the y position of the robot is the final y position
						robot.rotate(90);	// rotate to face negative y direction
						tll.goToNextLine();	//go forward - one tile in y
						y--;	// decrement y tile counter
						odometer.setPosition(new double [] {0.0, y*TILE_DISTANCE, 0.0}, new boolean [] {false, true, false});	//correct odometry
						robot.rotate(-90);	//rotate to face positive y direction
						tll.goToNextLine();	// go forward one tile in x
						x++;	// increment x tile counter
						odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 0.0}, new boolean [] {true, false, false});	//correct odometry
						travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);	// recursive call

					}
				}
				else{	//if the x position of the robot is the final x position
					robot.rotate(-90);	//rotate to face positive y direction
					travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);	//recursive call
				}

			}
			else{
				robot.rotate(90);	//rotate to face positive x direction
				if(x< xDestination && getFilteredData() >40){	//if the x position of the robot is not the final x position and there is no obstacle
					tll.goToNextLine();		//
					x++;
					odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 0.0}, new boolean [] {true, false, false});
					robot.rotate(-90);
					travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
				}
				else if( x< xDestination){
					robot.rotate(90);
					tll.goToNextLine();
					y--;
					odometer.setPosition(new double [] {0.0, y*TILE_DISTANCE, 0.0}, new boolean [] {false, true, false});
					robot.rotate(-90);
					tll.goToNextLine();
					x++;
					odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 0.0}, new boolean [] {true, false, false});
					robot.rotate(-90);
					travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
				}

			}
		}
		else if(x < xDestination){
			robot.rotate(90);
			if(getFilteredData() > 40){
				tll.goToNextLine();
				x++;
				odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 0.0}, new boolean [] {true, false, false});
				travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
				

			}
			else{
				robot.rotate(90);
				if(getFilteredData() > 40){
					tll.goToNextLine();
					y--;
					odometer.setPosition(new double [] {0.0, y*TILE_DISTANCE, 0.0}, new boolean [] {false, true, false});
					robot.rotate(-90);
					tll.goToNextLine();
					x++;
					odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 0.0}, new boolean [] {true, false, false});
					travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);
				}
				else{
					robot.rotate(90);
					tll.goToNextLine();
					x--;
					odometer.setPosition(new double [] {x*TILE_DISTANCE, 0.0 , 0.0}, new boolean [] {true, false, false});
					robot.rotate(-90);
					tll.goToNextLine();
					y--;
					odometer.setPosition(new double [] {0.0, y*TILE_DISTANCE, 0.0}, new boolean [] {false, true, false});
					robot.rotate(-90);
					if(getFilteredData() < 40){
						robot.rotate(90);
						tll.goToNextLine();
						y--;
						odometer.setPosition(new double [] {0.0, y*TILE_DISTANCE, 0.0}, new boolean [] {false, true, false});
					}

					robot.rotate(-90);
					tll.goToNextLine();
					x++;
					robot.rotate(-90);
					travelToZone(xBottomLeft, yBottomLeft, xTopRight, yTopRight);

				}

			}
		}
		else{	//navigation is done
			Sound.beep();
		}
		tll.goToNextLine();
		robot.rotate(90);
		tll.goToNextLine();
	}

	

	/**
	 * Performs the search. 
	 */
	public void doSearch(){
		objectDetector.rotateAndPoll(flagColor);
		if(objectDetector.getObject().equals(ObjectDetector.ObjectType.FLAG)){
			robot.rotate(objectDetector.getCorrectionAngle());
			navigation.backtrack(15);
			robot.rotate(180);
			clawMotor.rotate(-500);
			navigation.backtrack(20);
			clawMotor.rotate(500);


		}
		else{

		}

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

