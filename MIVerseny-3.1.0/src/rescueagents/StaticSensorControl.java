package rescueagents;

import interfaces.RobotPerception;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import rescueframework.MainFrame;
import world.Robot;

/**
 * StaticSensorControl
 * Control for static sensors.
 * Strategy: Passive data collection and energy optimization.
 * Since the framework automatically updates the shared map with the sensor's field of view,
 * the sensor's only task is to stay alive (IDLE) until it runs out of battery.
 */
public class StaticSensorControl extends AbstractRobotControl {
	private AMSService amsService;

	/**
	 * Constructor
	 * @param robot The robot object
	 * @param perception Perception interface
	 */
	public StaticSensorControl(Robot robot, RobotPerception perception) {
		super(robot, perception);
		this.amsService = AMSService.getAMSService();
		this.setRobotName("Sensor");
	}

	/**
	 * The sensor performs no active action, just observes.
	 * The IDLE state consumes the least energy, 
	 * maximizing lifespan and data collection time.
	 */
	@Override
	public Action step() {
		// We do not log every turn to avoid cluttering the console.
		// In the background, `perception` updates the common map.
		return Action.IDLE;
	}
	
	/**
	 * Called during map initialization to determine sensor X coordinate.
	 */
	public static int generateXCoord(int mapWidth, int mapHeight) {	
		return MainFrame.randomBetween(0, mapWidth); 
	}
	
	/**
	 * Called during map initialization to determine sensor Y coordinate.
	 */
	public static int generateYCoord(int mapWidth, int mapHeight) {
		return MainFrame.randomBetween(0, mapHeight);
	}
}