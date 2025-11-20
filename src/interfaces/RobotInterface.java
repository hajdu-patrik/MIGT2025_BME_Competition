package interfaces;

import world.Path;
import world.Robot;

public interface RobotInterface {

	public CellInfo getLocation();
	
	public boolean hasInjured();
	
	public InjuredInfo getInjuredInfo();
	
	public Path getPath();
	
	public void setInjured(InjuredInfo injured);
	
	public boolean hasMedicine();
	
	public int getMedicine();

    public Robot.Type getType();

	public String getName();

	public void setName(String newname);

    public int getInstanceId();
		
    /**
     * Returns the time, the robot waited in place
     * 
     * @return  waited
     */
    public int getWaited() ;
    /**
     * Increases the waiting time
     */
    public void incWaitingTime();
    
    /**
     * Resets the time that the robot waited
     */    
    public void resetWait();
}
