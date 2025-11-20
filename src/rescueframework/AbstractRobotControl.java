package rescueframework;

import interfaces.RobotInterface;
import interfaces.RobotPerception;
import world.Path;
import world.Robot;

/**
 * Abstract class for numbering the robot control objects
 */
public abstract class AbstractRobotControl {
    /** Percepcion of the agent */
    protected RobotPerception perception;
    
    /** The robot object in the world */
    protected RobotInterface robot;
    
    /** The path the robot is following right now */
    protected Path path;

    /**
     * Default constructor 
     * 
     * @param robot         The robot object in the world
     * @param perception    The perception of the robots of the world
     */
    public AbstractRobotControl(Robot robot, RobotPerception perception) {
        // Save perception and robot objects
        this.perception = perception;
        this.robot = robot;
    }
    
    /**
     * Returns the path the robot is following
     *
     * @return  The path the robot is following
     */
    public Path getPath() {
        return path;
    }

    /**
     * Abstract step method to determine the moving direction of the robot
     *
     * @return  Returns null to stay in place or 0-3 as a moving direction
     */
    public abstract Action step();

    public int getInstanceId() { return robot.getInstanceId(); }

    public String getRobotName() { return robot.getName(); }

    public void setRobotName(String newname) { robot.setName(newname); }

}
