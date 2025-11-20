package rescueagents;

import interfaces.RobotPerception;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import world.Robot;

/**
 * FlyingDroneControl
 * Drones are fast explorers. They don't get stuck in rubble (though in this simulation
 * walls are walls for them too, but movement is cheaper energy-wise).
 * Goal: Discover 'unknown' areas as quickly as possible.
 */
public class FlyingDroneControl extends AbstractRobotControl {
    private AMSService amsService;
    private RobotPerception internalWorldMap;

    public FlyingDroneControl(Robot robot, RobotPerception perception) {
        super(robot, perception);
        this.amsService = AMSService.getAMSService();
        internalWorldMap = AMSService.getInternalMap();
        this.setRobotName("Drone");
    }

    @Override
    public Action step() {
        // 1. Exploration: Find the nearest unknown area.
        path = internalWorldMap.getShortestUnknownPath(robot.getLocation());
        
        if (path != null) {
            // AMSService.log(this, "Exploring...");
            return amsService.moveRobotAlongPath(robot, path);
        } 
        
        // 2. If no unknown area (map fully explored),
        // do not waste energy hovering, go to exit and stop.
        if (!robot.getLocation().isExit()) {
            path = internalWorldMap.getShortestExitPath(robot.getLocation());
            if (path != null) {
                return amsService.moveRobotAlongPath(robot, path);
            }
        }
        
        // If at exit or no path, rest.
        return Action.IDLE;
    }
}