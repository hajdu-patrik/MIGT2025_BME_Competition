package world;

import interfaces.InjuredInfo;
import interfaces.RobotInterface;
import rescueframework.AbstractRobotControl;
import rescueframework.Action;


/**
 * Robot on the map
 */
public abstract class Robot implements RobotInterface{   
	
	public enum Type {ROBOT, SENSOR, DRONE}

    /** The name of the robot */
    protected String name;

    /** ID of the robot instance */
    protected int instanceId;

    /** next available robot ID */
    static int nextInstanceID = 0;
	
    /** Location of the robot */
    private Cell location;

    /**
     * The injured being carried by the robot
     */
    protected Injured injured = null;

    /**
     * The control object of th robot
     */
    private AbstractRobotControl control;

    protected int medicine = 0;
    
	/** Visibility distance of the robots */
	public int visibilityRange = 3;
	
	protected int batteryLifeTime = 5000;
	
	protected Type type = Type.ROBOT;
    /** Static ID value of the next injured */
    private static int nextID = 1;
    /** ID of the injured object */
    public int id;

    private int waited = 0;
    /**
     * Default constructor
     *
     * @param startCell The start cell of the robot
     */
    public Robot(Cell startCell) {
        location = startCell;
        startCell.discover();
        this.control = null;
        this.name = type.name(); // set the default name based on the type
        instanceId = nextInstanceID++;
        id=nextID;
        nextID++;
    }

    /**
     * Set the Control for the robot
     *
     * @param control
     */
    protected void setControl(AbstractRobotControl control) {
        this.control = control;
    }

    /**
     * Return the robot location
     *
     * @return The robot location
     */
    public Cell getLocation() {
        return location;
    }

    /**
     * Set the location of the robot
     *
     * @param newLocation The new location of the robot
     */
    public void setCell(Cell newLocation) {
        location = newLocation;
    }

    /**
     * Return true if the robot is currently carrying an injured
     *
     * @return True if the robot is carrying an injured
     */
    public boolean hasInjured() {
        return injured != null;
    }

    /**
     * Return the injured being carried by the robot
     *
     * @return The injured being carried by the robot
     */
    public Injured getInjured() {
        return injured;
    }

    @Override
    public InjuredInfo getInjuredInfo() {
        return getInjured();
    }

    /**
     * Call the AbstractRobotControl to decide the next step of the robot
     *
     * @return Stepping direction of the robot or NULL to stay in place
     */
    public Action step() {
        if (control == null) return null;

        return control.step();
    }

    /**
     * Returns the path the robot is following
     *
     * @return The path the robot is following
     */
    public Path getPath() {
        if (control != null) {
            return control.getPath();
        } else {
            return null;
        }
    }

    /**
     * Set the ijured being carried by the robot
     *
     * @param injured The injured to be carried
     */
    public void setInjured(Injured injured) {
        if (this.injured == null || injured == null) {
            // Do not allow overwriting the currently carried injured
            this.injured = injured;
        }
    }

    /**
     * Set the ijured being carried by the robot
     *
     * @param injured The injured to be carried
     */
    public void setInjured(InjuredInfo injured) {
        setInjured(injured);
    }

    /**
     * Returns the name of the image which displays the robot
     *
     * @return Image name
     */
    public abstract String getImageResourceName();


    /**
     * Retuns true if the robot has medicine
     *
     * @return True if the robot has medicine
     */
    public boolean hasMedicine() {
        return medicine > 0;
    }

    /**
     * Returns how many medicine the robot has
     *
     * @return The amount of remaining medicine
     */
    public int getMedicine() {
        return medicine;
    }

    public void useMedicine(Injured injured) {
        if (injured.getLocation().equals(location)) {
            int amount = Math.min(50, medicine);
            medicine -= amount;
            injured.addHealth(amount);
        }
    }

    public int getBatteryLifeTime() {
        return batteryLifeTime;
    }

    public void decreaseBatteryLifeTime(int val) {
        batteryLifeTime -= val;
    }

    public Type getType() {
        return type;
    }
    
    @Override
    /**
     * Returns the time, the robot waited in place
     * 
     * @return  waited
     */
    public int getWaited() {
    	return waited;
    }
    @Override
    /**
     * Increases the waiting time
     */
    public void incWaitingTime() {
    	waited++;
    }

    public int getInstanceId() { return instanceId; }

    public String getName() { return name + "[" + instanceId + "]"; }

    public void setName(String newname) { name = newname; }

    @Override
    /**
     * Resets the time that the robot waited
     */    
    public void resetWait() {
    	waited=0;
    }
    
}
