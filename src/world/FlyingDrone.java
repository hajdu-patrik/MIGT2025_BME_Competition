package world;

import interfaces.RobotPerception;
import rescueagents.FlyingDroneControl;
import rescueframework.MainFrame;

public class FlyingDrone extends Robot {
    private final static String imgResource = "drone";
    
	public FlyingDrone(Cell startCell, RobotPerception percepcion) {
        super(startCell);
        super.setControl(new FlyingDroneControl(this, percepcion));
		
        visibilityRange = 3;   
        
        type = Type.DRONE;
		
        batteryLifeTime = (Injured.MAXHEALTH / 3) + (MainFrame.nextRandInt(200) - 100); //333 + random between [-100, 100[
	}

	@Override
	public String getImageResourceName() {
		return imgResource;
	}
}
