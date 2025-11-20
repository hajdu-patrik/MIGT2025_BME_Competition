package world;

import interfaces.RobotPerception;
import rescueagents.StaticSensorControl;
import rescueframework.MainFrame;

public class StaticSensor extends Robot {

    private final static String imgResource = "static";
    
	public StaticSensor(Cell startCell, RobotPerception percepcion) {
        super(startCell);
        super.setControl(new StaticSensorControl(this, percepcion));	
        
		visibilityRange = 3;    	
		
		type = Type.SENSOR;
		
		batteryLifeTime = (Injured.MAXHEALTH / 4) + (MainFrame.nextRandInt(200) - 100); //250 + random between [-100, 100[
	}

	@Override
	public String getImageResourceName() {		
		return imgResource;
	}
}
