package world;

import java.awt.Color;
import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

import javax.imageio.ImageIO;

import interfaces.CellInfo;
import interfaces.InjuredInfo;
import interfaces.RobotInterface;
import interfaces.RobotPerception;
import rescueframework.MainFrame;
import rescueframework.RescueFramework;
import world.Robot.Type;
import world_debug.ViewLine;
import world_debug.ViewLineBreakPoint;

/**
 * The map representing the state of the world
 */
public class Map implements RobotPerception {
	private final int RESULT_SAVED_INJURED_FACTOR = 500;
	private final int RESULT_DEAD_OUTSIDE_FACTOR = 200;
	private final int RESULT_RESCUE_ROBOT_FACTOR = 300;
	private final int RESULT_MEDICAL_ROBOT_FACTOR = 400;
	private final int RESULT_DRONE_FACTOR = 50;
	private final int RESULT_STATIC_SENSOR_FACTOR = 10;

	/** Cell matrix of the map */
	public Cell cells[][];
	public int maxEnergy;
	/** Dimensions of the map */
	private int height = 0, width = 0;
	/** Image cache for loading every image only once */
	private HashMap<String, BufferedImage> imageCache = new HashMap<>();
	/** Exit cell of the map to transfer injureds to */
	private ArrayList<Cell> exitCells = new ArrayList<>();
	/** Viewlines of the robots (for robot view debug) */
	private ArrayList<ViewLine> viewLines = new ArrayList<>();
	/** Viewline break points of the robots (for robot view debug) */
	private ArrayList<ViewLineBreakPoint> viewLineBreakPoints = new ArrayList<>();
	/** Path to be displayed on the GUI */
	public ArrayList<Path> displayPaths = new ArrayList<>();
	/** Start cell specified for the robots */
	public Cell startCell = null;
	/** Name of the map file */
	private String fileName = "";
	/** List of floor definitions */
	private ArrayList<Floor> floorList = new ArrayList<>();
	/** Number of discovered cells */
	private int discoveredCellCount = 0;
	/** Injured people on the map */
	public ArrayList<Injured> injureds = new ArrayList<>();

	private int rescueRobotCount = 0;
	private int medicalRobotCount = 0;
	private int droneCount = 0;
	private int staticSensorCount = 0;

	/**
	 * Default constructor
	 * 
	 * @param fileName
	 *            Text file to load the map from
	 */
	public Map(String fileName, int rescueRobotCountIn, int medicalRobotCountIn, int droneAgentCountIn,
			int staticSensorCountIn, boolean loadInj) {
		String line;
		String[] array;
		int mode = 0;
		int row = 0;

		this.fileName = fileName;
		rescueRobotCount = rescueRobotCountIn;
		medicalRobotCount = medicalRobotCountIn;
		droneCount = droneAgentCountIn;
		staticSensorCount = staticSensorCountIn;

		try(BufferedReader reader = new BufferedReader(new FileReader("maps/" + fileName))) {
			while ((line = reader.readLine()) != null) {
				line = line.trim();
				// Skip comments
				if (line.startsWith("#") || line.startsWith("//") || line.isEmpty())
					continue;

				if (mode == 0) {
					// First line specifies map size
					array = line.split(" ");
					width = Integer.valueOf(array[0]);
					height = Integer.valueOf(array[1]);

					cells = new Cell[width][height];

					mode = 1;
				} else if (mode == 1) {
					// Process row definitions
					array = line.split(" ");
					if (array.length != width) {
						throw new Exception("Invalid row specificaion, row width differs: " + width + " =/= "
								+ array.length + " on line :" + line);
					} else {
						for (int i = 0; i < width; i++) {
							cells[i][row] = new Cell(i, row, array[i]);
							if (array[i].equals("S"))
								MainFrame.startCell = cells[i][row];
							else if (array[i].equals("X"))
								exitCells.add(cells[i][row]);
						}
					}

					row++;
					if (row >= height)
						mode = 2;

				} else if (mode == 2) {
					// Process other objects on the map (obstacles, injured, floor definitions)
					array = line.split(" ");
					if (array.length >= 4 && array[0].startsWith("Floor")) {
						// Floor definition found
						floorList.add(new Floor(Integer.valueOf(array[1]), Integer.valueOf(array[2]),
								Integer.valueOf(array[3])));
					} else if (array.length >= 4 && array[0].startsWith("Obstacle")) {
						// Obstacle defined
						crateObstacle(Integer.valueOf(array[1]), Integer.valueOf(array[2]), array[3]);
					} else if (array.length >= 3 && array[0].startsWith("Injured")) {
						// Injured defined
						int injuries;
						if (array.length >= 4) {
							// Load health level from file
							injuries = Integer.valueOf(array[3]);
						} else {
							// Generate random health level
							injuries = (int) ((float) MainFrame.random() * 1000F);
						}

						// Find affected cell
						int x = Integer.valueOf(array[1]);
						int y = Integer.valueOf(array[2]);
						Cell cell = getCell(x, y);

						// Create new injured and add to cell
						if (!cell.hasInjuredEx()) {
							Injured inj = new Injured(injuries);
							cell.setInjured(inj);
							inj.setLocation(cell);
							injureds.add(inj);
						}
					} else {
						RescueFramework.log("Unknown object definition skipped: " + line);
					}
				}
			}
		} catch (Exception e) {
			RescueFramework.log("Failed to load map from file: " + fileName);
			e.printStackTrace();
		}

		wallsAndNeighbors();

		// Default floor
		Floor defaultFloor = new Floor(5, 5, 0);
		floodFillFloor(defaultFloor);

		// Color floor for rooms
		for (int i = 0; i < floorList.size(); i++) {
			floodFillFloor(floorList.get(i));
		}

		// Update agent visibility and repaint GUI
		updateAllRobotVisibleCells(true);
	}

	/**
	 * Constructor for the unexplored world
	 * 
	 * @param fileName
	 *            Text file to load the map from
	 */
	public Map(String fileName, int rescueRobotCountIn, int medicalRobotCountIn, int droneAgentCountIn,
			int staticSensorCountIn) {
		String line;
		String[] array;
		boolean done = false;
		this.fileName = fileName;
		rescueRobotCount = rescueRobotCountIn;
		medicalRobotCount = medicalRobotCountIn;
		droneCount = droneAgentCountIn;
		staticSensorCount = staticSensorCountIn;

		try {
			BufferedReader reader = new BufferedReader(new FileReader("maps/" + fileName));

			while (!done && (line = reader.readLine()) != null) {
				line = line.trim();
				// Skip comments
				if (line.startsWith("#") || line.startsWith("//") || line.isEmpty())
					continue;

				// First line specifies map size
				array = line.split(" ");
				width = Integer.valueOf(array[0]);
				height = Integer.valueOf(array[1]);

				cells = new Cell[width][height];

				for (int i = 0; i < width; i++) {
					for (int j = 0; j < height; j++) {
						cells[i][j] = new Cell(i, j, ".");
						cells[i][j].hide();
					}
				}

				done = true;
			}

			reader.close();
		} catch (Exception e) {
			RescueFramework.log("Failed to load map from file: " + fileName);
			e.printStackTrace();
		}

		// Default floor
		Floor defaultFloor = new Floor(5, 5, 0);
		floodFillFloor(defaultFloor);

		// Color floor for rooms
		for (int i = 0; i < floorList.size(); i++) {
			floodFillFloor(floorList.get(i));
		}
	}

	public void wallsAndNeighbors() {
		// Share walls between cells
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {

				cells[x][y].shareWalls(getCell(x, y - 1), getCell(x + 1, y), getCell(x, y + 1), getCell(x - 1, y));
			}
		}

		// Update cell neighbours
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				cells[x][y].updateAccessibleNeighbours();
			}
		}
	}

	/**
	 * Get cached image identified by the string definition
	 * 
	 * @param image
	 *            String image definition
	 * @return Cached image
	 */
	public BufferedImage getCachedImage(String image) {
		if (!imageCache.containsKey(image)) {
			// Image not yet cached
			try {
				BufferedImage img = ImageIO.read(new File("images/" + image + ".png"));
				imageCache.put(image, img);
				return img;
			} catch (IOException ex) {
				ex.printStackTrace();
				return null;
			}
		} else {
			// Image found in the cache
			return imageCache.get(image);
		}
	}

	/**
	 * Create new obstacle on the map
	 * 
	 * @param x
	 *            X coordinate
	 * @param y
	 *            Y coordinate
	 * @param image
	 *            The obstacle image
	 */
	public void crateObstacle(int x, int y, String image) {
		cells[x][y].setObstacleImage(image);
	}

	/**
	 * Delets the injureds from cells
	 */
	public void deleteInjureds() {
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				cells[x][y].setInjured(null);
			}
		}
	}

	/**
	 * Return the height of the map
	 * 
	 * @return The height of the map
	 */
	public int getHeight() {
		return height;
	}

	/**
	 * Return the width of the map
	 * 
	 * @return The width of the map
	 */
	public int getWidth() {
		return width;
	}

	/**
	 * Return cell at a given position
	 * 
	 * @param x
	 *            X coordinate
	 * @param y
	 *            Y coordinate
	 * @return The cell or null if the coordinates are invalid
	 */
	public Cell getCell(int x, int y) {
		if (x >= 0 && x < width && y >= 0 && y < height)
			return cells[x][y];
		else
			return null;
	}

	/**
	 * Flood fill block of cells
	 * 
	 * @param floor
	 *            The floor object to start filling from
	 */
	private void floodFillFloor(Floor floor) {
		int index = 0;
		ArrayList<Cell> cells = new ArrayList<Cell>();
		cells.add(getCell(floor.getX(), floor.getY()));

		// Loop through all acessible cells from the floor definition
		while (index < cells.size()) {
			Cell cell = cells.get(index);
			if (cell == null || cell.isDoor() || cell.isExit()) {
				// Stop at doors
				index++;
				continue;
			}

			for (int direction = 0; direction < 4; direction++) {
				Cell neighbour = cell.getAccessibleNeigbourEx(direction);
//				Cell neighbour = cell.layoutNeigbours[direction];
				if (neighbour != null && cells.indexOf(neighbour) == -1) {
					// Add all accessible neighbour
					cells.add(neighbour);
				}
			}

			index++;
		}

		// Apply floor coloring to all cells found
		for (index = 0; index < cells.size(); index++) {
			Cell cell = cells.get(index);
			if (cell != null) {
				cell.setFloorColorIndex(floor.getColorCode());
			}
		}
	}

	/**
	 * Change the robot loation
	 * 
	 * @param robot
	 *            The robot to move
	 * @param dir
	 *            The direction to move to
	 * @return True if the robot is able to move to the specified direction
	 */
	/*
	 * public boolean moveRobot(Robot robot, Integer dir) { if
	 * (robot.getBatteryLifeTime() <= 0) { getRobots().remove(robot);
	 * RescueFramework.log("A(n) " + robot.getType().name() +
	 * " has run out of energy."); return false; }
	 * 
	 * if (dir == null || Type.SENSOR == robot.getType()) {
	 * RescueFramework.log("Agent staying in place."); return false; }
	 * 
	 * if (robot.getLocation().getAccessibleNeigbour(dir) != null) { return
	 * moveRobot(robot, robot.getLocation().getAccessibleNeigbour(dir)); } else {
	 * RescueFramework.log("Move failed: " + dir + " is inaccessible."); return
	 * false; } }
	 */
	/**
	 * Change the robot location
	 * 
	 * @param robot
	 *            The robot to move
	 * @param cell
	 *            The target cell to move the robot to
	 * @return True if the robot is able to move to the specified cell
	 */

	/*
	 * public boolean moveRobot(Robot robot, Cell cell) { if (cell == null || robot
	 * == null) return false;
	 * 
	 * // Avoid obstacles if (cell.hasObstacleEx() || Type.ROBOT == robot.getType())
	 * { RescueFramework.log("Move failed: " + cell.getX() + " x " + cell.getY() +
	 * " is occupied by an obstacle."); return false; }
	 * 
	 * if (cell.hasRobot()) { RescueFramework.log("Move failed: " + cell.getX() +
	 * " x " + cell.getY() + " is occupied by an agent."); return false; }
	 * 
	 * // Change location Cell previousCell = robot.getLocation();
	 * robot.setCell(cell); updateRobotPresence(previousCell);
	 * updateRobotPresence(cell);
	 * 
	 * // Update robot visibility and GUI updateAllRobotVisibleCells();
	 * RescueFramework.refresh(); return true; }
	 */
	/**
	 * Update the visibility of all robots operating on the map
	 */
	public void updateAllRobotVisibleCells(boolean copy) {
		// long startTime = System.currentTimeMillis();
		// RescueFramework.log("[0] Visibility start.");

		// Reset visibility
		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				cells[x][y].setRobotVisibility(false);
			}
		}

		// Reset viewlines and break points
		viewLines.clear();
		viewLineBreakPoints.clear();

		// Update robot visibility one by one
		for (int index = 0; index < MainFrame.robots.size(); index++) {
			updateRobotVisibleCells(MainFrame.robots.get(index), copy);
		}
	}

	/**
	 * Update the visibility of a single robot
	 * 
	 * @param r The robot to update visibility for
	 */
	public void updateRobotVisibleCells(Robot r, boolean copy) {
		Cell c = r.getLocation();
		Cell targetCell = null;

		if (c == null)
			return;

		// Loop through all cells around
		for (int x = c.getX() - r.visibilityRange; x <= c.getX() + r.visibilityRange; x++) {
			for (int y = c.getY() - r.visibilityRange; y <= c.getY() + r.visibilityRange; y++) {
				targetCell = getCell(x, y);
				if (targetCell != null) {
					// Skip cells to far away
					if (Math.sqrt((Math.pow(targetCell.getX() - c.getX(), 2))
							+ Math.pow(targetCell.getY() - c.getY(), 2)) > r.visibilityRange + 0.5)
						continue;

					// Check visibility
					boolean visible = checkCellVisibility(c.getX(), c.getY(), targetCell.getX(), targetCell.getY());
					if (visible) {
						targetCell.setRobotVisibility(visible);
					}
					
					for (Cell exitCell: MainFrame.map.exitCells) {
						if(exitCell.equals(targetCell)&&!(MainFrame.discovered.exitCells.contains(targetCell)))
							MainFrame.discovered.exitCells.add(targetCell);
					}

					// viewLines.add(new ViewLine(c.getX()+0.5, c.getY()+0.5,targetCell.getX()+0.5,
					// targetCell.getY()+0.5, visible));

					// if (visible)
					// targetCell.copy(MainFrame.discovered.cells[x][y]);
					if (copy) {
						targetCell.copy(MainFrame.discovered.getCell(x, y));
						MainFrame.discovered.cells[x][y].shareWalls(MainFrame.discovered.getCell(x, y - 1), MainFrame.discovered.getCell(x + 1, y),
								MainFrame.discovered.getCell(x, y + 1), MainFrame.discovered.getCell(x - 1, y));
						MainFrame.discovered.cells[x][y].updateAccessibleNeighbours();

						if (visible && targetCell.hasInjured()) {
							if (!targetCell.getInjured().isDiscovered()) {
								targetCell.injured.setDiscovered(true);
								findInjured(targetCell);
							} else {
								updateInjured(targetCell);
							}
						}

						if (targetCell.destroyed && visible && targetCell.destr_dir < 4) {
							MainFrame.discovered.cells[x][y].walls[targetCell.destr_dir] = false;
							Cell neighbour = MainFrame.discovered.cells[x][y].layoutNeigbours[targetCell.destr_dir];
							if (neighbour != null) {
								neighbour.setWall((targetCell.destr_dir + 2) % 4, false);
								neighbour.destroyed = true;
								getCell(x, y).seenSinceDest = true;
//								MainFrame.discovered.cells[x][y].seenSinceDest = true;
								MainFrame.discovered.cells[x][y].layoutNeigbours[targetCell.destr_dir]=neighbour;
								MainFrame.discovered.cells[x][y].updateAccessibleNeighbours();
								MainFrame.discovered.cells[x][y].layoutNeigbours[targetCell.destr_dir].updateAccessibleNeighbours();
								if (!targetCell.getObstacleImage().equals(""))
									MainFrame.discovered.cells[x][y].setObstacleImage("junk10");
							}
						}
					}
				}
			}
		}
	}

	public void findInjured(Cell pos) {
		Injured inj = new Injured(pos.getInjured().getHealth());
		inj.setDiscovered(true);
		inj.id = pos.getInjured().id;

		inj.setLocation(MainFrame.discovered.getCell(pos.getX(), pos.getY()));
		MainFrame.discovered.injureds.add(inj);
		MainFrame.discovered.getCell(pos.getX(), pos.getY()).setInjured(inj);
	}

	public void updateInjured(Cell cell) {
		Injured inj = cell.getInjured();
		inj.id = cell.getInjured().id;
		int index = -1;
		for (Injured iter : MainFrame.discovered.injureds) {
			if (iter.getLocation().equals(inj.getLocation())) {
				index = MainFrame.discovered.injureds.lastIndexOf(iter);
			}
		}
		if (index > -1) {
			MainFrame.discovered.removeInjured(MainFrame.discovered.injureds.get(index));
			MainFrame.discovered.injureds.remove(index);
			findInjured(this.getCell(inj.getLocation().getX(), inj.getLocation().getY()));
		}
	}

	/**
	 * Check visibility between two points in the X+ direction
	 * 
	 * @param x1_in
	 *            First point X coordinate
	 * @param y1_in
	 *            First point Y coordinate
	 * @param x2_in
	 *            Second point X coordinate
	 * @param y2_in
	 *            Second point Y coordinate
	 * @return True if there is no wall in the way
	 */
	public boolean checkCellVisibilityXPlus(int x1_in, int y1_in, int x2_in, int y2_in) {
		boolean logging = false;
		if (logging)
			RescueFramework.log(
					"-------- Visibility check between " + x1_in + " x " + y1_in + " and " + x2_in + " x " + y2_in);

		double dx, dy, a, b;

		double x1 = x1_in + 0.5;
		double y1 = y1_in + 0.5;
		double x2 = x2_in + 0.5;
		double y2 = y2_in + 0.5;

		// y = a*x+b
		// x = (y-b)/a
		dx = x2 - x1;
		dy = y2 - y1;
		if (dx != 0) {
			a = (double) dy / (double) dx;
		} else {
			a = 0;
		}
		b = (double) y1 - ((double) x1 * (double) a);

		int ydir = -1;
		if (y1 < y2)
			ydir = 1;
		if (logging)
			RescueFramework.log("dx = " + dx + " dy = " + dy + ";   " + b + " = " + y1 + "-" + x1 + "*" + a + ";   y = "
					+ a + "*x+" + b + ";   x = (y-" + b + ")/" + a + ";   ydir=" + ydir);
		if (logging)
			RescueFramework.log("Horizontal and corner check...");

		int xCell, yCell;
		double xx = 0, yy = 0;
		// Vertical wall test (only for non vertical lines)
		if (dx != 0) {
			xCell = x1_in + 1;
			while (xCell <= x2_in) {
				yy = a * xCell + b;

				if (Math.abs(yy - Math.round(yy)) < 0.01) {
					// Corner crossing
					yCell = (int) Math.round(yy);
					if (logging)
						RescueFramework
								.log("Checking x = " + xCell + " -> y=" + yy + " -> corner crossing (" + yCell + ")");

					if (ydir > 0) {
						// Direction \
						if (cells[xCell - 1][yCell - 1].hasWallEx(1) && cells[xCell - 1][yCell - 1].hasWallEx(2)) {
							if (logging) {
								RescueFramework.log("Bottom right corner hit of " + (xCell - 1) + " x " + (yCell - 1));
								viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.BLUE));
							}
							return false;
						} else if (cells[xCell][yCell].hasWallEx(0) && cells[xCell][yCell].hasWallEx(3)) {
							if (logging) {
								RescueFramework.log("Top left corner hit of " + (xCell) + " x " + (yCell));
								viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.CYAN));
							}
							return false;
						} else if (cells[xCell - 1][yCell - 1].hasWallEx(1) && cells[xCell][yCell].hasWallEx(3)) {
							if (logging) {
								RescueFramework.log("Vertical wall hit on " + xCell + " x " + yCell);
								viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.YELLOW));
							}
							return false;
						} else if (cells[xCell - 1][yCell].hasWallEx(0) && cells[xCell][yCell].hasWallEx(0)) {
							if (logging) {
								RescueFramework.log("Horizontal wall hit on " + xCell + " x " + yCell);
								viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.YELLOW));
							}
							return false;
						}
					} else {
						// Direction /
						if (cells[xCell - 1][yCell].hasWallEx(0) && cells[xCell - 1][yCell].hasWallEx(1)) {
							if (logging) {
								RescueFramework.log("Top right corner hit of " + (xCell - 1) + " x " + (yCell));
								viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.BLUE));
							}
							return false;
						} else if (cells[xCell][yCell - 1].hasWallEx(2) && cells[xCell][yCell - 1].hasWallEx(3)) {
							if (logging) {
								RescueFramework.log("Bottom left corner hit of " + (xCell) + " x " + (yCell - 1));
								viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.CYAN));
							}
							return false;
						} else if (cells[xCell][yCell].hasWallEx(3) && cells[xCell][yCell - 1].hasWallEx(3)) {
							if (logging) {
								RescueFramework.log("Vertical wall hit on " + xCell + " x " + yCell);
								viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.YELLOW));
							}
							return false;
						} else if (cells[xCell - 1][yCell].hasWallEx(0) && cells[xCell][yCell].hasWallEx(0)) {
							if (logging) {
								RescueFramework.log("Horizontal wall hit on " + xCell + " x " + yCell);
								viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.YELLOW));
							}
							return false;
						}
					}
				} else {
					// Wall crossing
					yCell = (int) Math.floor(yy);
					if (logging)
						RescueFramework
								.log("Checking x = " + xCell + " -> y=" + yy + " -> regular crossing. (" + yCell + ")");

					if (cells[xCell][yCell].hasWallEx(3)) {
						if (logging) {
							RescueFramework.log("Vertical wall hit at x = " + xCell + " -> y=" + yy);
							viewLineBreakPoints.add(new ViewLineBreakPoint(xCell, yy, Color.RED));
						}
						return false;
					}
				}

				xCell++;
			}
		} else {
			if (logging)
				RescueFramework.log("Skipping horizontal line.");
		}

		// Horizontal wall test (only for non vertical lines)
		if (logging)
			RescueFramework.log("Vertical check...");
		if (dy != 0) {
			if (ydir > 0) {
				// Direction \

				yCell = y1_in + 1;
				while (yCell <= y2_in) {
					if (a != 0) {
						xx = (yCell - b) / a;
					} else {
						xx = x1_in + 0.5;
					}

					xCell = (int) Math.floor(xx);
					if (logging)
						RescueFramework.log("Checking y=" + yCell + " -> x = " + xx + " -> regular crossing. (" + xCell
								+ " x " + yCell + ")");

					if (Math.abs(xCell - xx) > 0.01) {
						if (cells[xCell][yCell].hasWallEx(0)) {
							if (logging) {
								RescueFramework.log("Top wall hit on " + xCell + " x " + yCell);
								viewLineBreakPoints.add(new ViewLineBreakPoint(xx, yCell, Color.lightGray));
							}
							return false;
						}
					}

					yCell++;
				}

			} else {
				// Direction /

				yCell = y1_in;
				while (yCell > y2_in) {
					if (a != 0) {
						xx = (yCell - b) / a;
					} else {
						xx = x1_in + 0.5;
					}

					xCell = (int) Math.floor(xx);
					if (logging)
						RescueFramework.log("Checking y=" + yCell + " -> x = " + xx + " -> regular crossing. (" + xCell
								+ " x " + yCell + ")");

					if (Math.abs(xCell - xx) > 0.001) {
						if (cells[xCell][yCell].hasWallEx(0)) {
							if (logging) {
								RescueFramework.log("Bottom wall hit on " + xCell + " x " + yCell);
								viewLineBreakPoints.add(new ViewLineBreakPoint(xx, yCell, Color.lightGray));
							}

							return false;
						}
					}
					yCell--;
				}
			}
		} else {
			if (logging)
				RescueFramework.log("Skipping vertical line.");
		}

		return true;
	}

	/**
	 * Check visibility between two points
	 * 
	 * @param x1_in
	 *            First point X coordinate
	 * @param y1_in
	 *            First point Y coordinate
	 * @param x2_in
	 *            Second point X coordinate
	 * @param y2_in
	 *            Second point Y coordinate
	 * @return True if there is no wall in the way
	 */
	public boolean checkCellVisibility(int x1_in, int y1_in, int x2_in, int y2_in) {
		// The cell always sees itself
		if (x1_in == x2_in && y1_in == y2_in)
			return true;

		// Points above each other
		if (x1_in == x2_in) {
			if (y1_in < y2_in) {
				return checkCellVisibilityXPlus(x1_in, y1_in, x2_in, y2_in);
			} else {
				return checkCellVisibilityXPlus(x2_in, y2_in, x1_in, y1_in);
			}
		}

		// Points next to each other
		if (x1_in <= x2_in) {
			return checkCellVisibilityXPlus(x1_in, y1_in, x2_in, y2_in);
		} else {
			return checkCellVisibilityXPlus(x2_in, y2_in, x1_in, y1_in);
		}
	}

	public Cell getPathFirstCell(Cell from, Cell to) {
		return null;
	}

	public List<CellInfo> getExitCells() {
		return Collections.unmodifiableList(exitCells);
	}

	public List<InjuredInfo> getDiscoveredInjureds() {
		ArrayList<InjuredInfo> result = new ArrayList<InjuredInfo>();
		for (Injured i : MainFrame.discovered.injureds) {
			result.add(i);
		}
		return result;
	}

	public List<InjuredInfo> getDiscoveredInjureds(int maxHealth, int minHealth) {
		ArrayList<InjuredInfo> result = new ArrayList<InjuredInfo>();
		for (Injured i : MainFrame.discovered.injureds) {
			if (i.getHealth() <= maxHealth && i.getHealth() >= minHealth && !i.isSaved())
				result.add(i);
		}
		return result;
	}

	public ArrayList<Cell> getUnknownCells() {
		ArrayList<Cell> result = new ArrayList<Cell>();

		for (int x = 0; x < width; x++) {
			for (int y = 0; y < height; y++) {
				if (!cells[x][y].isDiscovered()) {
					result.add(cells[x][y]);
				}
			}
		}

		return result;
	}

	public Path getShortestPath(CellInfo start, List<Cell> targetCells, boolean isPathForDrone) {
		if (targetCells.size() == 0) {
			return null;
		}

		Cell startCell = (Cell) start;

		Collections.sort(targetCells, new Comparator<Cell>() {
			@Override
			public int compare(Cell cell1, Cell cell2) {
				if (cell1.rawDistanceFrom(startCell) < cell2.rawDistanceFrom(startCell)) {
					return -1;
				} else if (cell1.rawDistanceFrom(startCell) > cell2.rawDistanceFrom(startCell)) {
					return 1;
				} else {
					return 0;
				}
			}
		});

		int bestLength = -1;
		Path bestPath = AStarSearch.search(startCell, targetCells.get(0), -1);
		if (bestPath != null)
			bestLength = bestPath.getLength();

		for (int i = 1; i < targetCells.size(); i++) {
			Cell target = targetCells.get(i);
			int manhattanDistance = startCell.rawDistanceFrom(target);
			if (bestLength > -1) {
				if (manhattanDistance > bestLength)
					break;
			}

			Path thisPath = AStarSearch.search(startCell, target, bestLength);
			if (thisPath != null && (bestPath == null || thisPath.getLength() < bestPath.getLength())) {
				bestPath = thisPath;
				bestLength = bestPath.getLength();
			}
		}
		return bestPath;
	}

	@Override
	public Path getShortestExitPath(CellInfo start) {
		return getShortestPath(start, exitCells, false);
	}

	@Override
	public Path getShortestUnknownPath(CellInfo start) {
		return getShortestPath(start, getUnknownCells(), false);
	}

	@Override
	public Path getShortestInjuredPath(CellInfo start) {
		return getShortestInjuredPath(start, Injured.MAXHEALTH, -1, false);
	}

	@Override
	public Path getShortestInjuredPath(CellInfo start, int maxHealth, int minHealth) {
		return getShortestInjuredPath(start, maxHealth, minHealth, false);
	}

	private Path getShortestInjuredPath(CellInfo start, int maxHealth, int minHealth, boolean isPathForDrone) {
		List<InjuredInfo> knownInjuredList = getDiscoveredInjureds(maxHealth, minHealth);
		List<Cell> cellList = new ArrayList<>();
		for (int i = 0; i < knownInjuredList.size(); i++) {
			Cell location = (Cell) knownInjuredList.get(i).getLocation();
			if (location != null) {
				cellList.add(location);
			}
		}
		Path bestPath = getShortestPath(start, cellList, isPathForDrone);
		return bestPath;
	}

	public List<Robot> getRobots() {
		return MainFrame.robots;
	}

	public List<RobotInterface> getRobotsList() {
		return Collections.unmodifiableList(getRobots());
	}

	public ArrayList<Injured> getSavedInjureds() {
		return MainFrame.savedInjureds;
	}

	public ArrayList<Path> getDisplayPaths() {
		return displayPaths;
	}

	protected void addInjured(Injured injured, Cell cell) {
		injureds.add(injured);
		injured.setLocation(cell);
		cell.setInjured(injured);
	}

	public void removeInjured(Injured injured) {
		if (injured.getLocation() != null)
			injured.getLocation().setInjured(null);
	}

	protected void saveToFile() {
		try {
			BufferedWriter writer = new BufferedWriter(new FileWriter("maps/" + fileName));
			writer.write("// Map size\n");
			writer.write(width + " " + height + "\n");

			String obstacles = "";
			writer.write("\n// Cell borders\n");
			for (int y = 0; y < height; y++) {
				String line = "";
				for (int x = 0; x < width; x++) {
					String wallCode = "";

					if (cells[x][y].isExit()) {
						wallCode = "X";
					} else if (cells[x][y].isStart()) {
						wallCode = "S";
					} else if (cells[x][y].isDoor()) {
						wallCode = "_";
					} else {
						wallCode = cells[x][y].getWallCode();
					}

					line = line + wallCode + " ";

					if (cells[x][y].hasObstacleEx()) {
						obstacles = obstacles + "Obstacle " + x + " " + y + " " + cells[x][y].getObstacleImage() + "\n";
					}
				}
				writer.write(line + "\n");
			}

			writer.write("\n// Floors\n");
			for (int i = 0; i < floorList.size(); i++) {
				Floor floor = floorList.get(i);
				writer.write("Floor " + floor.getX() + " " + floor.getY() + " " + floor.getColorCode() + "\n");
			}

			writer.write("\n// Obstacles\n");
			writer.write(obstacles);

			writer.write("\n// Injureds\n");
			for (int i = 0; i < injureds.size(); i++) {
				Injured injured = injureds.get(i);
				writer.write("Injured " + injured.getLocation().getX() + " " + injured.getLocation().getY() + " "
						+ injured.getHealth() + "\n");
			}

			writer.close();
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Returns the number of discovered cells on the map
	 * 
	 * @return Number of discovered cells
	 */
	public int getDiscoveredCellCount() {
		return discoveredCellCount;
	}

	/**
	 * Sets the number of discovered cells on the map
	 * 
	 * @param discoveredCellCount
	 *            Number of discovered cells
	 */
	public void setDiscoveredCellCount(int discoveredCellCount) {
		this.discoveredCellCount = discoveredCellCount;
	}

	public int getRobotsEnergyLevel() {
		int retval = 0;

		for (Robot robot: MainFrame.robots) {
			retval += robot.getBatteryLifeTime();
		}

		return retval;
	}

	public int getRobotCount(boolean rescue) {
		if (rescue)
			return rescueRobotCount;
		else
			return medicalRobotCount;
	}

	public String getTotalScore() {
		int aliveOutside = 0;
		int deadOutside = 0;
		int injuredInside = 0;
		int totalInjured = injureds.size();

		ArrayList<Injured> inTransitInjureds = new ArrayList<>();
		for (int i = 0; i < MainFrame.robots.size(); i++) {
			if (MainFrame.robots.get(i).hasInjured()) {
				inTransitInjureds.add(MainFrame.robots.get(i).getInjured());
			}
		}

		for (int i = 0; i < totalInjured; i++) {
			Injured injured = injureds.get(i);
			if (injured.getLocation() == null && inTransitInjureds.indexOf(injured) == -1) {
				// Already outside
				if (injured.isAlive()) {
					aliveOutside++;
				} else {
					deadOutside++;
				}
			} else
				injuredInside++;
		}

		int rescueScore = (aliveOutside * RESULT_SAVED_INJURED_FACTOR + deadOutside * RESULT_DEAD_OUTSIDE_FACTOR);

		String s = "Time: " + MainFrame.getTime() + " | ";

		s +=  totalInjured + " injured (inside: " + injuredInside + ", outside: " + aliveOutside
				+ " alive, " + deadOutside + " dead)  = " + rescueScore;

		int infrastructureCost = (rescueRobotCount * RESULT_RESCUE_ROBOT_FACTOR
				+ medicalRobotCount * RESULT_MEDICAL_ROBOT_FACTOR + droneCount * RESULT_DRONE_FACTOR
				+ staticSensorCount * RESULT_STATIC_SENSOR_FACTOR);

		s +=  "  |  robot cost: " + infrastructureCost;

		int consumedEnergy = maxEnergy - getRobotsEnergyLevel();

		s +=  "  |  Energy: " + consumedEnergy + " used";

		int finalScore = rescueScore - infrastructureCost - consumedEnergy - MainFrame.getTime();

		s +=  "  |  Score: " + finalScore + "";

		s +=  "  |  Simulation " + MainFrame.status;

		return s;
	}

	public void updateRobotPresence(Cell cell) {
		for (int i = 0; i < MainFrame.robots.size(); i++) {
			final Robot robot = MainFrame.robots.get(i);

			if (robot.getLocation().equals(cell) && Type.ROBOT == robot.getType()) {
				cell.setRobotPresence(true);
				return;
			}
		}
		cell.setRobotPresence(false);
	}

	public String getFileName() {
		return fileName;
	}

}
