package rescueagents;

import java.util.HashMap;
import java.util.Map;

import interfaces.CellInfo;
import interfaces.InjuredInfo;
import interfaces.RobotInterface;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import rescueframework.RescueFramework;
import world.Path;

/**
 * Agent Management Service (AMS)
 * Central coordination class for robots.
 * Maintains the shared world model and handles task allocation.
 */
public class AMSService {

    static AMSService amsService;
    public static world.Map internalWorldModel;

    // Track which injured person is "claimed" by which robot.
    // Key: Injured ID, Value: Robot Instance ID
    private Map<Integer, Integer> claimedInjureds = new HashMap<>();

    private AMSService() {
    }

    public static AMSService getAMSService() {
        if (amsService == null) {
            amsService = new AMSService();
        }
        return amsService;
    }

    /**
     * Returns the internal world model.
     */
    public static world.Map getInternalMap() {
        return internalWorldModel;
    }

    /**
     * Initializes the internal world model. Called by the framework.
     */
    public static void setInternalWorldModel(world.Map initialWorldMap) {
        internalWorldModel = initialWorldMap;
    }

    public static void log(AbstractRobotControl control, String message) {
        RescueFramework.log(control.getRobotName() + " " + message);
    }
    
    public static void log(RobotInterface rob, String message) {
        RescueFramework.log(rob.getName() + " " + message);
    }

    // --- Coordination Methods ---

    /**
     * Attempts to claim an injured person for a specific robot.
     * @param injuredId The ID of the injured person
     * @param robotId The ID of the robot
     * @return true if the claim is successful (or already claimed by this robot), false if claimed by another.
     */
    public synchronized boolean claimInjured(int injuredId, int robotId) {
        if (claimedInjureds.containsKey(injuredId)) {
            // If already claimed, return true only if it is claimed by us
            return claimedInjureds.get(injuredId) == robotId;
        }
        // Free target, claim it
        claimedInjureds.put(injuredId, robotId);
        return true;
    }

    /**
     * Releases a claimed injured person (e.g., if the robot dies or abandons the target).
     */
    public synchronized void releaseInjured(int injuredId, int robotId) {
        if (claimedInjureds.containsKey(injuredId) && claimedInjureds.get(injuredId) == robotId) {
            claimedInjureds.remove(injuredId);
        }
    }
    
    /**
     * Checks if the injured person is already saved or claimed by someone else.
     * @param injuredInfo The injured person to check
     * @param myRobotId The querying robot's ID
     * @return true if it is a valid, available target
     */
    public synchronized boolean isAvailableTarget(InjuredInfo injuredInfo, int myRobotId) {
        if (injuredInfo == null) return false;
        if (injuredInfo.isSaved()) return false; // Already outside
        // Note: We act on dead bodies too because they give points (+200), but they are lower priority.
        
        // Check the map to see if the internal model still thinks they are there
        CellInfo cell = injuredInfo.getLocation();
        if (cell == null || !cell.hasInjured()) return false;

        // Check the claim table
        // Trick: Accessing the ID from the world.Injured object as it's not directly on the interface
        if (cell.getInjured() != null) {
            int id = cell.getInjured().id;
            if (claimedInjureds.containsKey(id)) {
                return claimedInjureds.get(id) == myRobotId; // Only available if ours or unclaimed
            }
        }
        return true;
    }

    // --- Movement Helper Methods ---

    /**
     * Calculates the next step along a path.
     * Handles basic collision avoidance (waiting).
     */
    public Action moveRobotAlongPath(RobotInterface robot, Path path) {
        if (path == null) return Action.IDLE;
        
        CellInfo nextCell = path.getNextCell(robot.getLocation());
        
        // If there is no next cell (e.g., reached destination) or path is invalid
        if (nextCell == null) {
            return Action.IDLE;
        }

        // Collision Check: Obstacle or another Robot
        // Note: AStarSearch usually avoids walls, but we handle dynamic obstacles here.
        if (nextCell.hasObstacle()) {
            log(robot, "Obstacle in the way! Re-planning might be needed.");
            return Action.IDLE; 
        }
        
        if (nextCell.hasRobot()) {
            // If there is a robot, wait a turn.
            // Advanced logic: if waiting too long, re-plan.
            robot.incWaitingTime();
            if (robot.getWaited() > 2) {
               // log(robot, "Waiting too long, I am blocked.");
            }
            return Action.IDLE;
        } else {
            robot.resetWait();
        }

        return nextCell.directionFrom(robot.getLocation());
    }
}