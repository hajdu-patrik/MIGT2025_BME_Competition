package world_debug;

import java.awt.Color;

/**
 * Robot line of visibility break point
 */
public class ViewLineBreakPoint {
    // X coordinate
    public double x;
    
    // Y coordinate
    public double y;
    
    // Point color
    public Color color;
    
    /**
     * Default constructor
     * 
     * @param x         X coordinate
     * @param y         Y coordinate
     * @param color     Color of the point
     */
    public ViewLineBreakPoint(double x, double y, Color color) {
        this.x = x;
        this.y = y;
        this.color = color;
    }
    
}
