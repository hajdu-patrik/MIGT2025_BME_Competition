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
 * MedicalRobotControl
 * The goal of the medical robot is to keep injured people alive.
 * Priority is given to patients in critical condition.
 */
public class MedicalRobotControl extends AbstractRobotControl {
    // Fallback behavior if out of medicine
    RescueRobotControl fallbackRobotControl;
    
    private AMSService amsService;
    private RobotPerception internalWorldMap;

    public MedicalRobotControl(Robot robot, RobotPerception perception) {
        super(robot, perception);
        // Even if the medic acts as "Rescue" (when out of medicine), it needs the rescue logic
        fallbackRobotControl = new RescueRobotControl(robot, perception);
        this.amsService = AMSService.getAMSService();
        internalWorldMap = AMSService.getInternalMap();
        this.setRobotName("Medic");
    }

    @Override
    public Action step() {
        // 1. If out of medicine, behave like a rescue robot (e.g., carry patients)
        if (!robot.hasMedicine()) {
            return fallbackRobotControl.step();
        }

        // 2. If standing on an injured person who is alive and needs healing
        if (robot.getLocation().hasInjured()) {
            Injured inj = robot.getLocation().getInjured();
            if (inj.isAlive() && inj.getHealth() < Injured.MAXHEALTH) {
                // Heal!
                // Tactic: Don't strictly require MAX/3. If we are already here,
                // healing adds health which buffers against the time it takes to transport them.
                AMSService.log(this, "Healing in progress. HP: " + inj.getHealth());
                return Action.HEAL;
            }
        }

        // 3. Search for a patient who is in critical condition but salvageable
        // Critical: e.g., HP is low but > 0.
        InjuredInfo target = findCriticalPatient();
        
        if (target != null) {
             // Go to the patient
             path = world.AStarSearch.search((world.Cell)robot.getLocation(), (world.Cell)target.getLocation(), -1);
             if (path != null) {
                 return amsService.moveRobotAlongPath(robot, path);
             }
        }

        // 4. If no critical patient found, behave like a rescue robot (search/carry)
        // This ensures the medic doesn't stay idle if everyone is healthy enough.
        return fallbackRobotControl.step();
    }

    /**
     * Finds the patient in the most critical condition.
     */
    private InjuredInfo findCriticalPatient() {
        List<InjuredInfo> knownInjureds = internalWorldMap.getDiscoveredInjureds();
        InjuredInfo best = null;
        // Looking for lowest HP but still alive
        int minHealth = Integer.MAX_VALUE;

        for (InjuredInfo info : knownInjureds) {
            if (!info.isSaved() && info.isAlive()) {
                // Only worth going if health is not full
                if (info.getHealth() < Injured.MAXHEALTH) {
                    // Distance also matters: if far away and HP is low, they might die before arrival.
                    // Simplified heuristic: Decide based on Health.
                    if (info.getHealth() < minHealth) {
                        minHealth = info.getHealth();
                        best = info;
                    }
                }
            }
        }
        return best;
    }
}