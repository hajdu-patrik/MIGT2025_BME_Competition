/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package world;

import interfaces.RobotPerception;
import rescueagents.RescueRobotControl;

/**
 *
 * @author MarxAttila
 */
public class RescueRobot extends Robot{
    
    private final static String imgResource = "robot1";
    
    /**
     * 
     * @param startCell     The start cell of the robot
     * @param percepcion    Percepcion of the robot
     */
    public RescueRobot(Cell startCell, RobotPerception percepcion) {
        super(startCell);
        super.setControl(new RescueRobotControl(this, percepcion));
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
