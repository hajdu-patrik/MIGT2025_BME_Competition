package interfaces;

import rescueframework.Action;
import world.Injured;

public interface CellInfo {

	public boolean hasInjured();
	
	public Injured getInjured();
	
	public CellInfo getAccessibleNeigbour(Integer dir);
	
	public boolean isDiscovered();
	
	public boolean robotSeesIt();
	
	public boolean hasObstacle();
	
	public boolean isExit();
	
	public boolean isStart();
	
	public Action directionFrom(CellInfo otherCell);
	
	public int getX();
	
	public int getY();
	
	public int rawDistanceFrom(CellInfo other);
	
	public boolean isBetweenCells(CellInfo other1, CellInfo other2);
	
	public boolean hasRobot();

}
