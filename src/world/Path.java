package world;

import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import interfaces.CellInfo;

/**
 * Path object representing a path between two cells
 */
public class Path {
    /** The cells building up the path */
    private List<CellInfo> path = new ArrayList<>();
    /** Display color of the path */
    private Color color = Color.MAGENTA;

    /**
     * Return the color setting of the path
     * @return          The color setting of the path
     */
    public Color getColor() {
        return color;
    }

    /**
     * Set the color setting of the path
     * @param color     The new color of the path
     */
    public void setColor(Color color) {
        this.color = color;
    }
    
    /** 
     * Return cells building up the path
     * @return          Cells on the path
     */
    public List<CellInfo> getPath() {
        return path;
    }
    
    /**
     * Return the first cell along the path
     * @return          The first cell along the path
     */
    public CellInfo getFirstCell() {
        if (path.size()>1) return path.get(1);
        else return null;
    }

    
    /**
     * Return the last cell along the path
     * @return          The last cell along the path
     */
    public CellInfo getLastCell() {
        if (path.size()>0) return path.get(path.size()-1);
        else return null;
    }
    
    /**
     * Return the first cell along the path
     * @return          The first cell along the path
     */
    public CellInfo getNextCell(CellInfo from) {
        for (int i=0; i<path.size(); i++) {
            if (path.get(i).equals(from)) {
                if (i<path.size()-1) {
                    return path.get(i+1);
                }
                break;
            }
        }
        
        return null;
    }
    
    /**
     * Return the start cell of the path
     * @return          The start cell of the path
     */
    public CellInfo getStartCell() {
        if (path.size()>0) return path.get(0);
        else return null;
    }
    
    /**
     * Add new cell to the end of the path
     * @param cell      The cell to add
     */
    public void addLastCell(CellInfo cell) {
        path.add(cell);
    }
    
    /**
     * Add new cell to the beginning of the cell
     * @param cell      The new cell to add
     */
    public void addFirstCell(CellInfo cell) {
        path.add(0, cell);
    }
    
    /**
     * Return the length of the path
     * @return          The length of the path
     */
    public int getLength() {
        return path.size();
    }
}
