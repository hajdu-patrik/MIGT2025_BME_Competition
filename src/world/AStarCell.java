package world;

/**
 * A cell on the open/closed list of the A* search
 */
public class AStarCell {
    /** The cell on the map */ 
    private Cell cell;
    /** Total path to the cell */
    private int sumG = 0;
    /** Heuristics for the cell */
    private double h = 0;
    /** Parent of the cell */
    private AStarCell parent = null;
    
    /**
     * Default constructor
     * @param cell          The cell on the map
     * @param sumG          Total path to the cell
     * @param h             Heuristics for the cell
     * @param parent        Parent of the cell
     */
    public AStarCell(Cell cell, int sumG, double h, AStarCell parent) {
        this.cell = cell;
        this.sumG = sumG;
        this.h = h;
        this.parent = parent;
    }
    
    /**
     * Return the path+heuristics value
     * @return      The path+heuristics value 
     */
    public double getF() {
        return sumG+h;
    }
    
    /**
     * Return the parent of the cell
     * @return      The parent of the cell
     */
    public AStarCell getParent() {
        return parent;
    }
    
    /**
     * Return the total path to the cell
     * @return      The total path to the cell 
     */
    public int getSumG() {
        return sumG;
    }
    
    /**
     * Return the heuristics value of the cell
     * @return      The heuristics value of the cell
     */
    public double getH() {
        return h;
    }
    
    /**
     * Return the cell on the map
     * @return      The cell on the map
     */
    public Cell getCell() {
        return cell;
    }
}
