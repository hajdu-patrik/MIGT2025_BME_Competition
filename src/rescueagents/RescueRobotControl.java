package rescueagents;

import java.util.List;

import interfaces.InjuredInfo;
import interfaces.RobotPerception;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;
import world.Injured;
import world.Path;
import world.Robot;

/**
 * RescueRobotControl
 * Logic for rescue robots. Goal: Discover injured people and transport them to exits.
 * Coordinates via AMSService to avoid duplicated work.
 */
public class RescueRobotControl extends AbstractRobotControl {
    private AMSService amsService;
    private RobotPerception internalWorldMap;
    
    // The ID of the currently targeted injured person (-1 if none)
    private int currentTargetInjuredId = -1;

    public RescueRobotControl(Robot robot, RobotPerception perception) {
        super(robot, perception);
        this.amsService = AMSService.getAMSService();
        internalWorldMap = AMSService.getInternalMap();
        this.setRobotName("Rescue");
    }

    @Override
    public Action step() {
        // 1. If we are carrying an injured person, take them to the nearest exit
        if (robot.hasInjured()) {
            // We are no longer chasing a target on the map since we picked one up
            if (currentTargetInjuredId != -1) {
                amsService.releaseInjured(currentTargetInjuredId, robot.getInstanceId());
                currentTargetInjuredId = -1;
            }

            if (robot.getLocation().isExit()) {
                AMSService.log(this, "Dropping off injured at exit.");
                return Action.PUT_DOWN;
            } else {
                // Path to nearest exit
                path = internalWorldMap.getShortestExitPath(robot.getLocation());
                if (path != null) {
                    return amsService.moveRobotAlongPath(robot, path);
                } else {
                    AMSService.log(this, "Cannot find path to exit! Exploring...");
                    path = internalWorldMap.getShortestUnknownPath(robot.getLocation());
                     return amsService.moveRobotAlongPath(robot, path);
                }
            }
        }

        // 2. If not carrying anyone, check if we are standing on one
        if (robot.getLocation().hasInjured()) {
            Injured inj = robot.getLocation().getInjured();
            // Only pick up if no one else claimed it, OR we claimed it
            if (amsService.claimInjured(inj.id, robot.getInstanceId())) {
                AMSService.log(this, "Picking up injured.");
                return Action.PICK_UP;
            } else {
                 AMSService.log(this, "Injured here, but claimed by another robot. Searching for others.");
            }
        }

        // 3. Find a target: The best available (unclaimed) injured person
        InjuredInfo bestTarget = findBestInjuredTarget();

        if (bestTarget != null) {
            // Claim it
            int id = ((Injured)bestTarget).id; // Casting needed to access specific ID in this framework
            amsService.claimInjured(id, robot.getInstanceId());
            currentTargetInjuredId = id;

            // Go there using A* Search
            path = world.AStarSearch.search((world.Cell)robot.getLocation(), (world.Cell)bestTarget.getLocation(), -1);
            
            if (path != null) {
                return amsService.moveRobotAlongPath(robot, path);
            }
        }

        // 4. If no known injured or unreachable, then EXPLORE
        if (currentTargetInjuredId != -1) {
             amsService.releaseInjured(currentTargetInjuredId, robot.getInstanceId());
             currentTargetInjuredId = -1;
        }
        
        path = internalWorldMap.getShortestUnknownPath(robot.getLocation());
        if (path != null) {
            return amsService.moveRobotAlongPath(robot, path);
        }

        // 5. If everything is explored and no work left, go to exit and rest
        if (!robot.getLocation().isExit()) {
            path = internalWorldMap.getShortestExitPath(robot.getLocation());
            return amsService.moveRobotAlongPath(robot, path);
        }

        return Action.IDLE;
    }

    private InjuredInfo findBestInjuredTarget() {
        List<InjuredInfo> knownInjureds = internalWorldMap.getDiscoveredInjureds();
        
        InjuredInfo bestAlive = null;
        int minAliveDist = Integer.MAX_VALUE;
        
        InjuredInfo bestDead = null;
        int minDeadDist = Integer.MAX_VALUE;

        for (InjuredInfo info : knownInjureds) {
            if (amsService.isAvailableTarget(info, robot.getInstanceId())) {
                int dist = robot.getLocation().rawDistanceFrom(info.getLocation());
                
                if (info.isAlive()) {
                    if (dist < minAliveDist) {
                        minAliveDist = dist;
                        bestAlive = info;
                    }
                } else {
                    if (dist < minDeadDist) {
                        minDeadDist = dist;
                        bestDead = info;
                    }
                }
            }
        }
        
        // Priority to alive patients
        if (bestAlive != null) {
            return bestAlive;
        }
        
        return bestDead;
    }
}