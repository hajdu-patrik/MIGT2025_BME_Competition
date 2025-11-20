package interfaces;

public interface InjuredInfo {

	public CellInfo getLocation();
	
	public int getHealth();
	
	public float getHealthRatio();
	
	public boolean isSaved();
	
	public boolean isDiscovered();
	
	public boolean isAlive();

}
