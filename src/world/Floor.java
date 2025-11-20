package world;

/**
 * Floor object representing the floor color of an area
 */
public class Floor {
    /** X coordinate of the cell */
    private int x;
    /** Y coordinate of the cell */
    private int y;
    /** Color of the cell */
    private int colorCode;

    /**
     * Return the X coordinate of the cell
     * 
     * @return      X coordinate of the cell
     */
    public int getX() {
        return x;
    }

    /**
     * Return the Y coordinate of the cell
     * 
     * @return      Y coordinate of the cell
     */
    public int getY() {
        return y;
    }

    /**
     * Return the color of the cell
     * 
     * @return      Color of the cell
     */
    public int getColorCode() {
        return colorCode;
    }

    /**
     * Default constructor
     * 
     * @param x             X coordinate of the cell
     * @param y             Y coordinate of the cell
     * @param colorCode     Color of the cell
     */
    public Floor(int x, int y, int colorCode) {
        this.x = x;
        this.y = y;
        this.colorCode = colorCode;
    }
}
