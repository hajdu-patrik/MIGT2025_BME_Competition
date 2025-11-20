package world;

import interfaces.RobotPerception;
import rescueagents.MedicalRobotControl;

/**
 * @author MarxAttila
 */
public class MedicalRobot extends Robot{
    private final static String imgResource = "robot2";

    /**
     * Default constructor
     * 
     * @param startCell     The start cell of the robot
     * @param percepcion    Percepcion of the robot
     */
    public MedicalRobot(Cell startCell, RobotPerception percepcion, int medicine) {
        super(startCell);
        super.setControl(new MedicalRobotControl(this, percepcion));
        this.medicine = medicine;
        id+=1000;
    }
    
    /**
     * Returns the name of the image which displays the robot
     * 
     * @return  Image name
     */
    @Override
    public String getImageResourceName() {
        return imgResource;
    }   


}
