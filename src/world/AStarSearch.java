package world;

import java.awt.Color;
import java.util.ArrayList;

import rescueframework.RescueFramework;

/**
 * Static class performing A* search
 */
public class AStarSearch {
    /** Open list of the nodes */
    static ArrayList<AStarCell> openList = new ArrayList<>();
    /** Closed list of the nodes */
    static ArrayList<AStarCell> closedList = new ArrayList<>();
    
    /**
     * Find path between start and target no longer than maxDistance
     * 
     * @param start         The start cell to search from
     * @param target        The target cell to search to
     * @param maxDistance   The maximum length of the path to consider
     * @param color         Color of the path
     * @return              A Path if exists between start and target or NULL
     */
    public static Path search(Cell start, Cell target, int maxDistance, Color color) {
        Path result = search(start,target, maxDistance, false);
        if (result != null) result.setColor(color);
        return result;
    }
    
    /**
     * Find path between start and target no longer than maxDistance
     * 
     * @param start         The start cell to search from
     * @param target        The target cell to search to
     * @param maxDistance   The maximum length of the path to consider
     * @return              A Path if exists between start and target or NULL
     */
    public static Path search(Cell start, Cell target, int maxDistance) {
    	return search(start, target, maxDistance, false);
    }
    
    /**
     * Find path between start and target no longer than maxDistance.
     * Gives path through obstacles and other robots.
     * 
     * @param start         The start cell to search from
     * @param target        The target cell to search to
     * @param maxDistance   The maximum length of the path to consider
     * @param color         Color of the path
     * @return              A Path if exists between start and target or NULL
     */
    public static Path searchPathForDrone(Cell start, Cell target, int maxDistance, Color color) {
    	Path result = search(start,target, maxDistance, true);
        if (result != null) result.setColor(color);
        return result;
    }
    

    private static Path search(Cell start, Cell target, int maxDistance, boolean isPathForDrone) {
    	isPathForDrone=false;
        // Ignore targets too far
        if (maxDistance >0 && start.rawDistanceFrom(target) > maxDistance) return null;
        
        // Try trivial solutions first
        /*Path trivialPath = trivialSearch(start,target);
        if (trivialPath != null) return trivialPath;*/
        
        // Disable console logging
        boolean verbose = false; 
        if (verbose) RescueFramework.log("AStarSearch between "+start.getX()+"x"+start.getY()+" and "+target.getX()+"x"+target.getY()+" max distance "+maxDistance);
        
        // Init open and closed lists
        openList.clear();
        closedList.clear();
        
        // Init the start cell and add to the open list
        AStarCell startCell = new AStarCell(start, 0, heuristics(start,target), null);
        openList.add(startCell);
        
        // Loop while the open list is not empty
        int iteration = 0;
        while (openList.size()>0) {
            iteration++;
            if (verbose) {
                System.out.print("--- Iteration #"+iteration+". Openlist: ");
                RescueFramework.log(listToString(openList));
                System.out.print("Closed list: ");
                RescueFramework.log(listToString(closedList));
            }
            
            // Choose next node
            int bestIndex = 0;
            double bestF = openList.get(bestIndex).getF();
            for (int i=1; i<openList.size();i++) {
                if (openList.get(i).getF()<bestF) {                    
                    bestIndex = i;
                    bestF = openList.get(i).getF();
                }
            }
            
            // Expand node with best F
            AStarCell selectedCell = openList.get(bestIndex);
            if (verbose) RescueFramework.log("  Processing node "+selectedCell.getCell().getX()+" x "+selectedCell.getCell().getY()+" ("+(Math.round(100*(selectedCell.getSumG()+selectedCell.getH()))/100d)+")");
            
            // Check max distance constraint
            if (maxDistance>0 && selectedCell.getSumG()>maxDistance) {
                if (verbose) RescueFramework.log("    Max distance reached ("+maxDistance+"), giving up!");
                return null;
            }
            
            // Check if target reached
            if (selectedCell.getCell().equals(target)) {
                if (verbose) RescueFramework.log("    Target reached: "+target.getX()+" x "+target.getY()+". Building path back:");
                Path result = new Path();
                while (selectedCell != null) {
                    if (verbose) System.out.print(selectedCell.getCell().getX()+" x "+selectedCell.getCell().getY()+" --> ");
                    result.addFirstCell(selectedCell.getCell());
                    selectedCell = selectedCell.getParent();
                }
                if (verbose) RescueFramework.log("");
                return result;
            }
            
            // Add known neighbours to the open list
            for (int dir = 0; dir<4; dir++) {
                Cell possibleNeighbour = selectedCell.getCell().getAccessibleNeigbour(dir);
                if (possibleNeighbour != null) {
                    if ((possibleNeighbour.isDiscovered() || possibleNeighbour.equals(target)) && possibleNeighbour.seenSinceDest){
                        //if (possibleNeighbour.equals(target) || !possibleNeighbour.hasInjured()) {
                            if (possibleNeighbour.equals(target) || !possibleNeighbour.hasObstacle() || isPathForDrone) {
                                if (possibleNeighbour.equals(target) 
                                		|| !possibleNeighbour.hasRobot()
                                		|| isPathForDrone) {
                                
                                    boolean skip = false;

                                    // Check on open list
                                    for (int i=0; i<openList.size(); i++) {
                                        if (openList.get(i).getCell().equals(possibleNeighbour)) {
                                            skip = true;
                                            break;
                                        }
                                    }
                                    if (skip) {
                                        if (verbose) RescueFramework.log("    Already on open list.");
                                        continue;
                                    }

                                    // Check on closed list
                                    for (int i=0; i<closedList.size(); i++) {
                                        if (closedList.get(i).getCell().equals(possibleNeighbour)) {
                                            skip = true;
                                            break;
                                        }
                                    }
                                    if (skip) {
                                        if (verbose) RescueFramework.log("    Already on closed list.");
                                        continue;
                                    }

                                    // Add possible neighbour to open list
                                    if (verbose) RescueFramework.log("    Adding to open list!");
                                    openList.add(new AStarCell(possibleNeighbour, selectedCell.getSumG()+1, heuristics(possibleNeighbour,target), selectedCell));
                                } else {
                                    if (verbose) RescueFramework.log("    Occupied by a robot."); 
                                }
                            } else {
                               if (verbose) RescueFramework.log("    Occupied by an obstacle."); 
                            }
                        /*} else {
                           if (verbose) RescueFramework.log("    Occupied by a injured."); 
                        }*/
                    } else {
                        if (verbose) RescueFramework.log("    Not discovered yet.");
                    }
                } else {
                     if (verbose) RescueFramework.log("    No neighbour in dir "+dir+".");
                }
            }
            
            // Add selected node to the closed list
            openList.remove(selectedCell);
            closedList.add(selectedCell);
        }
        
        return null;
    }
    
    /**
     * Calculate heuristics value between two cells
     * 
     * @param c1        The first cell
     * @param c2        The second cell
     * @return          The heuristics value between the two cells
     */
    private static double heuristics(Cell c1, Cell c2) {
        //return Math.sqrt(Math.pow(c1.getX()-c2.getX(),2)+Math.pow(c1.getY()-c2.getY(),2));
        return Math.abs(c1.getX()-c2.getX())+Math.abs(c1.getY()-c2.getY());
    }
    
    /**
     * Print node list as String for console debugging 
     * @param list      The list of cells
     * @return          The string list of the cells
     */
    public static String listToString(ArrayList<AStarCell> list) {
        String result = "";
        int g; 
        double h;
        for (int i=0; i<list.size(); i++) {
            AStarCell cell = list.get(i);
            g = cell.getSumG();
            h = cell.getH();
                    
            result = result + cell.getCell().getX()+"x"+cell.getCell().getY()+"("+g+"+"+(Math.round(100*(h))/100d)+"="+(Math.round(100*(g+h))/100d)+"), ";
        }
        return result;
    }
    
    
    public static Path trivialSearch(Cell start, Cell target) {
        if (start.getX() == target.getX()) {
            if (start.getY() > target.getY()) {
                return trivialSearch(start, target, 0, null);
            } else {
                return trivialSearch(start, target, 2, null);
            }
        } else if (start.getY() == target.getY()) {
            if (start.getX() > target.getX()) {
                return trivialSearch(start, target, 3, null);
            } else {
                return trivialSearch(start, target, 1, null);
            }
        }
        
        Integer dir1, dir2;
        if (start.getY() > target.getY()) {
            dir1 = 0;
        } else {
            dir1 = 2;
        }
        if (start.getX() > target.getX()) {
            dir2 = 3;
        } else {
            dir2 = 1;
        }
        Path result = trivialSearch(start, target, dir1, dir2);
        if (result != null) {
            return result;
        } else {
            return trivialSearch(start, target, dir2, dir1);
        }
    }
    
    public static Path trivialSearch(Cell start, Cell target, Integer startDir, Integer otherDir) {
        boolean verbose = false;
        int dirIndex = 0;
        Integer[] dirOptions = new Integer[2];
        dirOptions[0] = startDir;
        dirOptions[1] = otherDir;
        
        if (verbose) RescueFramework.log("Running TrivialSearch betweene "+start.toString()+" and "+target.toString()+" with startDir "+startDir+"...");
        
        Path result = new Path();
        result.addFirstCell(start);
        
        Cell currentCell = start;
        
        
        Cell nextCell = currentCell.getAccessibleNeigbour(dirOptions[dirIndex]);
        if (nextCell != null && (!nextCell.isBetweenCells(start, target) || nextCell.hasObstacle()  || nextCell.hasRobot())) {
            if (verbose && nextCell != null) RescueFramework.log("  First cell is "+nextCell.toString()+" out of range or has obstacle.");
            return null;
        }
        if (verbose && nextCell != null) RescueFramework.log("  First cell is "+nextCell.toString());
        
        while (nextCell != null) {   
            if (verbose) RescueFramework.log("  Adding cell to path: "+nextCell.toString());
            result.addLastCell(nextCell);
            
            if (nextCell.equals(target)) {
                // Found trivial path!
                if (verbose) RescueFramework.log("  Target reached: returning the path.");
                return result;
            }
            
            currentCell = nextCell;
            nextCell = currentCell.getAccessibleNeigbour(dirOptions[dirIndex]);
            if (nextCell != null && (!nextCell.isBetweenCells(start, target) || nextCell.hasObstacle() || nextCell.hasRobot())) nextCell = null;
            
            if (nextCell == null) {
                // Found wall in the last direction!
                if (verbose) RescueFramework.log("  No allowed neighbour in direction: "+dirOptions[dirIndex]);
                
                nextCell = currentCell.getAccessibleNeigbour(dirOptions[otherIndex(dirIndex)]);
                if (nextCell != null && (!nextCell.isBetweenCells(start, target) || nextCell.hasObstacle() || nextCell.hasRobot())) nextCell = null;
                
                if (nextCell == null) {
                    // No trivial path found!
                    if (verbose) RescueFramework.log("  No allowed neighbour in direction: "+dirOptions[dirIndex]+" either. Terminating search.");
                    return null;
                } else {
                    // Keep moving into the new direction
                    dirIndex = otherIndex(dirIndex);
                    if (verbose) RescueFramework.log("  Changed current direction to: "+dirOptions[dirIndex]+".");
                }
            }
        }
        
        // No path found
        if (verbose) RescueFramework.log("  Next cell is null, search failed.");
        return null;
    }
    
    private static Integer otherIndex(Integer currentIndex) {
        if (currentIndex == 0) return 1;
        return 0;
    }
}
