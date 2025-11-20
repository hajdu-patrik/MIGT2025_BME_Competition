package rescueagents;

import interfaces.RobotPerception;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import rescueframework.MainFrame;
import world.Robot;

/**
 * StaticSensorControl
 * Purpose: Passive observation. Positioning is important.
 */
public class StaticSensorControl extends AbstractRobotControl {
    private AMSService amsService;

    public StaticSensorControl(Robot robot, RobotPerception perception) {
        super(robot, perception);
        this.amsService = AMSService.getAMSService();
        this.setRobotName("Sensor");
    }

    @Override
    public Action step() {
        return Action.IDLE;
    }
    
    public static int generateXCoord(int mapWidth, int mapHeight) { 
        return MainFrame.randomBetween(1, mapWidth - 2); 
    }
    
    public static int generateYCoord(int mapWidth, int mapHeight) {
        return MainFrame.randomBetween(1, mapHeight - 2);
    }
}