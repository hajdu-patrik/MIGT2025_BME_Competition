package world_debug;

/**
 * Viewline of the robot
 */
public class ViewLine {
    // Endpoint coordinates of the viewline
    public double x1, y1, x2, y2;
    
    // True if there is no wall in the way
    public boolean success = false;
    
    /**
     * Default consructor
     * 
     * @param x1        X1 coordinate of the line
     * @param y1        Y1 coordinate of the line
     * @param x2        X2 coordinate of the line
     * @param y2        Y2 coordinate of the line
     * @param success   True if there is no wall in the way
     */
    public ViewLine(double x1, double y1, double x2, double y2, boolean success) {
        this.x1 = x1;
        this.y1 = y1;
        this.x2 = x2;
        this.y2 = y2;
        this.success = success;
    }
}
