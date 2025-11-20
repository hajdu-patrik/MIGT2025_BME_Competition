package world;

import interfaces.CellInfo;
import rescueframework.Action;
import rescueframework.MainFrame;
import rescueframework.RescueFramework;

/**
 * Cell object that the map is built up from
 */
public class Cell implements CellInfo {
	/** Coordinates of the cell */
	private int x, y;
	/** Walls in all four directions */
	boolean[] walls = new boolean[4];
	/** Image file of the obstacle on the cell */
	private String obstacleImage = "";
	/** True if the cell is already discovered by the robots */
	private boolean discovered = false;
	/** The injured on this cell */
	Injured injured = null;
	/** True if the robot currently sees this cell */
	private boolean robotSees = false;
	/** Type of the cell (0 = default; 1 = exit; 2 = start) */
	private int cellType = 0;
	/** Color code index of the floor */
	private int floorColorIndex = -1;
	/** True if the cell is a door (stopping flooded floor coloring) */
	private boolean door = false;
	/** List of neighbour cell in all directions */
	public Cell layoutNeigbours[] = new Cell[4];
	/** List of accessible cells in all direction */
	public Cell accessNeigbours[] = new Cell[4];

	private boolean hasRobot = false;
	/** Determines if it has a destroyed wall or not */
	public boolean destroyed = false;
	/** True if the cell has been seen since it's wall destroyed */
	public boolean seenSinceDest = true;
	public int destr_dir = 4;

	public void copy(Cell into) {
		if (into != null) {
			into.x = x;
			into.y = y;
			for (int i = 0; i < 4; i++)
				if (walls[i])
					into.walls[i] = true;
				else
					into.walls[i] = false;
			into.obstacleImage = obstacleImage;
			into.discovered = discovered;
			into.robotSees = robotSees;
			into.cellType = cellType;
			into.floorColorIndex = floorColorIndex;
			into.door = door;
			into.hasRobot = hasRobot;
			into.destroyed = destroyed;
			into.destr_dir = destr_dir;
			into.seenSinceDest = seenSinceDest;
		}
	}

	/**
	 * Constructor of the cell
	 * 
	 * @param x        X coordinate of the cell on the map
	 * @param y        Y coordinate of the cell on the map
	 * @param wallCode Binary representation of the walls around this cell
	 */
	public Cell(int x, int y, String wallCode) {
		this.x = x;
		this.y = y;
		int wallCodeInt = -1;

		// Determine special cell marks or wall code
		if (wallCode.equals("X")) {
			// Exit cell - type=1
			wallCodeInt = 0;
			cellType = 1;
		} else if (wallCode.equals("S")) {
			// Start cell - type=2
			wallCodeInt = 0;
			cellType = 2;
		} else if (wallCode.equals("_")) {
			// Door cell
			door = true;
			wallCodeInt = 0;
		} else if (wallCode.equals(".")) {
			wallCodeInt = 0;
		} else {
			wallCodeInt = Integer.parseInt(wallCode, 16);
		}

		// Set the wall code based on bitmask compare
		walls[0] = (wallCodeInt & 1) > 0;
		walls[1] = (wallCodeInt & 2) > 0;
		walls[2] = (wallCodeInt & 4) > 0;
		walls[3] = (wallCodeInt & 8) > 0;
	}

	/**
	 * Returns true if the cell has wall in the provided direction
	 * 
	 * @param direction The direction to check
	 * @return True if there is a wall in the selected direction
	 */
	public boolean hasWall(int direction) {
		if (isDiscovered()) {
			return hasWallEx(direction);
		} else {
			RescueFramework.log("Access denied: calling hasWall for undiscovered cell!");
			return false;
		}
	}

	public boolean hasWallEx(int direction) {
		// Normalize direction value
		while (direction < 0)
			direction += 4;
		while (direction > 3)
			direction -= 4;
		// Return the wall
		return walls[direction];
	}

	/**
	 * Return the type code of the cell
	 * 
	 * @return The type code of the cell
	 */
	protected int getCellType() {
		return cellType;
	}

	/**
	 * Exchange walls between cells next to each other
	 * 
	 * @param top    Top neighbour cell
	 * @param right  Right neighbour cell
	 * @param bottom Bottom neighbour cell
	 * @param left   Left neighbour cell
	 */
	protected void shareWalls(Cell top, Cell right, Cell bottom, Cell left) {
		// Save neighbours
		layoutNeigbours[0] = top;
		layoutNeigbours[1] = right;
		layoutNeigbours[2] = bottom;
		layoutNeigbours[3] = left;

		// Add local walls to neighbours
		if (walls[0] && top != null)
			top.addWall(2);
		if (walls[1] && right != null)
			right.addWall(3);
		if (walls[2] && bottom != null)
			bottom.addWall(0);
		if (walls[3] && left != null)
			left.addWall(1);
	}

	/**
	 * Determine accessible neighbour cells
	 */
	protected void updateAccessibleNeighbours() {
		for (int i = 0; i < 4; i++)
			if (!walls[i])
				accessNeigbours[i] = layoutNeigbours[i];

	}

	/**
	 * Add wall to cell
	 * 
	 * @param direction The direction to add the wall to
	 */
	protected void addWall(int direction) {
		walls[direction] = true;
	}

	/**
	 * Toggle cell wall
	 * 
	 * @param direction The direction to toggle wall to
	 */
	protected void toggleWall(int direction) {
		walls[direction] = !walls[direction];
		Cell neighbour = layoutNeigbours[direction];
		if (neighbour != null) {
			neighbour.setWall((direction + 2) % 4, walls[direction]);
		}
	}

	/**
	 * Remove cell wall
	 * 
	 * @param direction The direction to remove wall from
	 */
	public void removeWall(int direction) {
		destroyed = true;
		seenSinceDest = false;
		destr_dir = direction;
		walls[direction] = false;
		// this.hide();
		Cell neighbour = layoutNeigbours[direction];
		if (neighbour != null) {
			neighbour.setWall((direction + 2) % 4, false);
			neighbour.destroyed = true;

			updateAccessibleNeighbours();
			neighbour.updateAccessibleNeighbours();
		}

		if (MainFrame.nextRandInt(100) < 66) {
			if (!hasRobot && !hasInjured()) {
				setObstacleImage("junk10");
			}
		}
	}

	/**
	 * Set cell wall to value
	 * 
	 * @param direction The direction to change wall
	 * @param value     The new wall value
	 */
	protected void setWall(int direction, boolean value) {
		walls[direction] = value;
	}

	/**
	 * Return the accessible neighbour if exists
	 * 
	 * @param direction The direction requested
	 * @return The neighbour if exists
	 */
	public Cell getAccessibleNeigbour(Integer direction) {
		if (isDiscovered()) {
			return getAccessibleNeigbourEx(direction);
		} else {
			RescueFramework.log("Access denied: calling getAccessibleNeigbour for undiscovered cell!");
			return null;
		}
	}

	public Cell getAccessibleNeigbourEx(Integer direction) {
		if (direction == null)
			return null;
		updateAccessibleNeighbours();
		return accessNeigbours[direction];
	}

	/**
	 * Return true if the cell is a door
	 * 
	 * @return True if the cell is a door
	 */
	protected boolean isDoor() {
		return door;
	}

	/**
	 * Set the floor color index
	 * 
	 * @param colorIndex The floor color index
	 */
	protected void setFloorColorIndex(int colorIndex) {
		floorColorIndex = colorIndex;
		if (colorIndex > -1)
			discovered = false;
	}

	/**
	 * Returns the floor color index
	 * 
	 * @return The floor color index
	 */
	protected int getFloorColorIndex() {
		return floorColorIndex;
	}

	/**
	 * Set new obstacle image name
	 * 
	 * @param obstacleImage The new obstacle image name
	 */
	protected void setObstacleImage(String obstacleImage) {
		this.obstacleImage = obstacleImage;
	}

	/**
	 * Return the obstacle image name
	 * 
	 * @return The obstacle image name
	 */
	protected String getObstacleImage() {
		return obstacleImage;
	}

	/**
	 * Mark the cell as discovered
	 */
	protected void discover() {
		discovered = true;
		seenSinceDest = true;
	}

	/**
	 * Remove the discovered mark
	 */
	protected void hide() {
		if (discovered) {
			discovered = false;
			seenSinceDest = true;
		}
	}

	/**
	 * Return true if the cell is discovered by the robots
	 * 
	 * @return True if the cell is alredy discovered
	 */
	public boolean isDiscovered() {
		return discovered;
	}

	/**
	 * Add injured to the cell
	 * 
	 * @param injured The injured to be put on the cell
	 */
	public void setInjured(Injured injured) {
		this.injured = injured;
		// MainFrame.injureds.add(injured);
	}

	/**
	 * Return the injured on the cell
	 * 
	 * @return The injured on the cell
	 */
	public Injured getInjured() {
		if (isDiscovered()) {
			return getInjuredEx();
		} else {
			RescueFramework.log("Access denied: calling getInjured for undiscovered cell!");
			return null;
		}
	}

	protected Injured getInjuredEx() {
		return injured;
	}

	/**
	 * Change the robot visibility value of the cell
	 * 
	 * @param newValue The new value
	 */
	protected void setRobotVisibility(boolean newValue) {
		// Save the new value
		robotSees = newValue;

		// Update the cell and injured discovered status upon discovery
		if (newValue) {
			discovered = true;
			/*
			 * if (injured != null) { injured.setDiscovered(true); }
			 */
		}
	}

	/**
	 * Return true if a robot actually sees this cell
	 * 
	 * @return True if a robot actually sees this cell
	 */
	public boolean robotSeesIt() {
		return robotSees;
	}

	/**
	 * Returns true if the cell has an obstacle on it
	 * 
	 * @return True if the cell has an obstacle on it
	 */
	public boolean hasObstacle() {
		if (isDiscovered()) {
			return hasObstacleEx();
		} else {
			RescueFramework.log("Access denied: calling hasObstacle for undiscovered cell!");
			return false;
		}
	}

	public boolean hasObstacleEx() {
		return !obstacleImage.isEmpty();
	}

	/**
	 * Returns true if the cell has an injured on it
	 * 
	 * @return True if the cell has an injured on it
	 */
	public boolean hasInjured() {
		if (isDiscovered()) {
			return hasInjuredEx();
		} else {
			RescueFramework.log("Access denied: calling hasInjured for undiscovered cell!");
			return false;
		}
	}

	public boolean hasInjuredEx() {
		return injured != null;
	}

	/**
	 * Returns true if the cell is an exit cell
	 * 
	 * @return True if the cell is an exit cell
	 */
	public boolean isExit() {
		return cellType == 1;
	}

	/**
	 * Returns true if the cell is a start cell
	 * 
	 * @return True if the cell is a start cell
	 */
	public boolean isStart() {
		return cellType == 2;
	}

	/**
	 * Set exit cell
	 * 
	 * @param True if the cell is an exit cell
	 */
	protected void setExit(boolean value) {
		if (value) {
			cellType = 1;
		} else {
			cellType = 0;
		}
	}

	/**
	 * Set as start cell
	 * 
	 * @param True if the cell is the start cell
	 */
	protected void setStart(boolean value) {
		if (value) {
			cellType = 2;
		} else {
			cellType = 0;
		}
	}

	/**
	 * Compare cell equality by checking coordinates
	 * 
	 * @param other The other cell to compare to
	 * @return
	 */
	public boolean equals(Cell other) {
		return other.x == x && other.y == y;
	}

	/**
	 * Calculate direction from an other cell
	 * 
	 * @param otherCell The other cell to calculate directions from
	 * @return which way the robot should move
	 */
	public Action directionFrom(CellInfo otherCell) {
		if (otherCell.getY() > y)
			return Action.STEP_UP;
		if (otherCell.getY() < y)
			return Action.STEP_DOWN;
		if (otherCell.getX() > x)
			return Action.STEP_LEFT;
		return Action.STEP_RIGHT;
	}

	/**
	 * Display cell coordinates as string
	 * 
	 * @return Cell coordinates as string
	 */
	public String toString() {
		return x + "x" + y;
	}

	/**
	 * Returns the X coordinate of the cell
	 * 
	 * @return X coordinate of the cell
	 */
	public int getX() {
		return x;
	}

	/**
	 * Returns the Y coordinate of the cell
	 * 
	 * @return Y coordinate of the cell
	 */
	public int getY() {
		return y;
	}

	protected String getWallCode() {
		int code = 0;
		if (walls[0])
			code = code + 1;
		if (walls[1])
			code = code + 2;
		if (walls[2])
			code = code + 4;
		if (walls[3])
			code = code + 8;

		if (code == 0) {
			return ".";
		} else if (code < 10) {
			return String.valueOf(code);
		} else {
			switch (code) {
			case 10:
				return "A";
			case 11:
				return "B";
			case 12:
				return "C";
			case 13:
				return "D";
			case 14:
				return "E";
			case 15:
				return "F";
			}
		}

		return ".";
	}

	public int rawDistanceFrom(CellInfo other) {
		return (Math.abs(other.getX() - x) + Math.abs(other.getY() - y));
	}

	public boolean isBetweenCells(CellInfo other1, CellInfo other2) {
		return ((other1.getX() <= x && other2.getX() >= x) || (other2.getX() <= x && other1.getX() >= x))
				&& ((other1.getY() <= y && other2.getY() >= y) || (other2.getY() <= y && other1.getY() >= y));
	}

	public boolean hasRobot() {
		return hasRobot;
	}

	protected void setRobotPresence(boolean value) {
		hasRobot = value;
	}

}
