package rescueframework;

import java.util.List;

import javax.swing.SwingUtilities;

import world.Cell;
import world.Injured;
import world.Map;
import world.Robot;
import world.Robot.Type;

/**
 * Autostep thread of the simulation
 */
public class StepThread extends Thread {
	/** The thread does not change the time while false */
	private boolean enabled = false;
	/** Time left of the current step */
	private int timeLeft = 0;
	/** The time of a whole step */
	private int timeStep = 500;

	/**
	 * Main method of the thread
	 */

	public void run() {
		// Endless loop
		while (true) {
			// Sleep first
			try {
				Thread.sleep(1);
			} catch (Exception e) {
			}

			// Only change time when enabled
			if (enabled) {
				// Decrease time left
				timeLeft--;

				// Initiate repaint if no repaint is in progress
				if (timeLeft <= 0 && (!RescueFramework.mainFrame.getPaintPanel().isPaintingInProgress())) {
					RescueFramework.mainFrame.getPaintPanel().setPaintingInProgress(true);
					SwingUtilities.invokeLater(new Runnable() {
						public void run() {
							stepTime();
						}
					});
					timeLeft = 1000 - timeStep;
				}
			}
		}
	}

	/*
	 * Enable the step thread
	 */
	public void enable() {
		enabled = true;
		timeLeft = 0;
	}

	public void stepTime() {

		// RescueFramework.map.stepTime(true);
		MainFrame.time++;
		RescueFramework.log(" ---  Step " + MainFrame.time + "");

		MainFrame.removeRandomWall();

		// Calculate injured states
		for (int i = 0; i < MainFrame.map.injureds.size(); i++) {
			Injured injured = MainFrame.map.injureds.get(i);
			if (!injured.isSaved()) {
				int prevHealth = injured.getHealth();
				if (prevHealth > 0) {
					prevHealth--;
					injured.setHealth(prevHealth);
				}
			}
		}

		// Display robot paths
		MainFrame.map.displayPaths.clear();
		// long start = System.currentTimeMillis();
		boolean movingRobot = false;
		for (int i = 0; i < MainFrame.robots.size(); i++) {
			Robot robot = MainFrame.robots.get(i);
			
			if (robot.getBatteryLifeTime() <= 0) {
				// Don't remove the robot, just disable it.
				// MainFrame.discovered.getRobots().remove(robot);
				// MainFrame.robots.remove(robot);
				robot.visibilityRange = -1;
				RescueFramework.log(robot.getName() + " is out of energy.");
				continue;
			}

			Action stepResult = Action.IDLE;
			try {
				stepResult = robot.step();
			} catch (Exception e) {
				RescueFramework.log("Exception in robot.step(): " + e.getMessage());
				e.printStackTrace();
				if (RescueFramework.autoClose) {
					RescueFramework.log("Batch run terminated on exception.");
					System.exit(-1);
				}
			}
			if (stepResult == Action.IDLE) {
				if (Type.SENSOR == robot.getType()) {
					robot.decreaseBatteryLifeTime(1);
				}
				if(Type.SENSOR != robot.getType()) {
					RescueFramework.log(robot.getName() + " @ " + robot.getLocation().toString() + " -> sleep");
				}
			} else if (stepResult == Action.STEP_UP || stepResult == Action.STEP_RIGHT 
					|| stepResult == Action.STEP_DOWN || stepResult == Action.STEP_LEFT) {
				RescueFramework.log(
						robot.getName() + " @ " + robot.getLocation().toString() + " -> " + stepResult);
				
				if(Type.ROBOT == robot.getType()) {
					robot.decreaseBatteryLifeTime(2); //moving costs 2 units of energy for a robot
				} else {
					robot.decreaseBatteryLifeTime(1);//moving costs 1 units of energy for a drone
				}
				
				Cell dest = null;
				if (robot.getLocation().accessNeigbours[stepResult.getValue()] != null) 
					dest = robot.getLocation().getAccessibleNeigbour(stepResult.getValue());
				 else
					RescueFramework.log("Move failed: " + stepResult + " is inaccessible.");

				if (dest != null) {
					final Robot robotAtThisCell = getRobotAtThisCell(MainFrame.robots, dest);
					
					if (dest.hasObstacleEx() && Type.ROBOT == robot.getType()) {
						RescueFramework.log("Move failed: " + dest.getX() + " x " + dest.getY()
								+ " is occupied by an obstacle.");
					} else if (robotAtThisCell != null
							&& Type.ROBOT == robotAtThisCell.getType()
							&& Type.ROBOT == robot.getType()) {
						RescueFramework.log("Move failed: " + dest.getX() + " x " + dest.getY()
								+ " is occupied by a robot.");
					} else {
						// Change location
						Cell previousCell = robot.getLocation();
						robot.setCell(dest);
						MainFrame.map.updateRobotPresence(previousCell);
						MainFrame.discovered.updateRobotPresence(previousCell);
						MainFrame.map.updateRobotPresence(dest);
						MainFrame.discovered.updateRobotPresence(dest);

						// Update robot visibility and GUI

						MainFrame.map.updateAllRobotVisibleCells(true);
						MainFrame.discovered.updateAllRobotVisibleCells(false);
						RescueFramework.refresh();
						movingRobot = true;
						dest = null;
					}
				}
			} else if (Type.ROBOT == robot.getType() 
					&& (stepResult == Action.PICK_UP || stepResult == Action.PUT_DOWN)) {
				movingRobot = true;

				robot.decreaseBatteryLifeTime(1);

				if (robot.hasInjured()) {
					// Put down injured

					if (!robot.getLocation().hasInjuredEx()) {

						if (robot.getLocation().isExit()) {
							RescueFramework.log(robot.getName() + " @ " + robot.getLocation().toString()
									+ " -> puts down injured on exit cell");
							Injured savedInjured = robot.getInjured();
							savedInjured.id=robot.getInjured().id;
							int index=-1;
							
							for (Injured iter : MainFrame.discovered.injureds) {															
								if (iter.id== robot.getInjured().id) {
									index=MainFrame.discovered.injureds.indexOf(iter);
								}
							}
							if (index > -1) {							
								MainFrame.discovered.removeInjured(MainFrame.discovered.injureds.get(index));
								MainFrame.discovered.injureds.remove(index);
								robot.setInjured(null);
								savedInjured.setSaved();
								MainFrame.savedInjureds.add(savedInjured);	
							}				
						} else {
							RescueFramework.log(robot.getName() + " @ " + robot.getLocation().toString()
									+ " -> puts down injured");
							int x = robot.getLocation().getX();
							int y = robot.getLocation().getY();
							MainFrame.discovered.cells[x][y].setInjured(robot.getInjured());
							robot.getInjured().setLocation(robot.getLocation());
							robot.getLocation().setInjured(robot.getInjured());
							robot.setInjured(null);
						}

					} else {
						RescueFramework.log(robot.getName() + " @ " + robot.getLocation().toString()
								+ " -> unable to put down injured, the cell already has one!");
					}

				} else if (robot.getLocation().hasInjuredEx()) {
					// Pick up injured

					RescueFramework.log(robot.getName() + " @ " + robot.getLocation().toString()
							+ " -> picks up injured");
					Injured injured = robot.getLocation().getInjured();
					int x = robot.getLocation().getX();
					int y = robot.getLocation().getY();
					robot.setInjured(injured);
					injured.getLocation().setInjured(null);					
					injured.setLocation(null);
					MainFrame.discovered.cells[x][y].setInjured(null);
					MainFrame.map.cells[x][y].setInjured(null);
				}

			} else if (stepResult == Action.HEAL && Type.ROBOT == robot.getType()) {
				// Heal if possible
				robot.decreaseBatteryLifeTime(1);
				movingRobot = true;

				if (robot.hasMedicine()) {
					if (robot.getLocation().hasInjuredEx()) {
						Injured injured = robot.getLocation().getInjured();
						robot.useMedicine(injured);

					} else {
						RescueFramework.log(robot.getName() + " @ " + robot.getLocation().toString()
								+ " -> healing failed: no injured on cell");
					}
				} else {
					RescueFramework.log(robot.getName() + " @ " + robot.getLocation().toString()
							+ " -> healing failed: out of medicine");
				}
			} else if (stepResult != null) {
				RescueFramework.log(robot.getName() + " @ " + robot.getLocation().toString()
						+ " -> invalid step action: " + stepResult);
			}

			/*
			 * Path p = getShortestExitPath(robot.getLocation()); if (p != null) {
			 * p.setColor(Color.GREEN); displayPaths.add(p); }
			 * 
			 * p = getShortestUnknownPath(robots.get(i).getLocation()); if (p != null) {
			 * p.setColor(Color.DARK_GRAY); displayPaths.add(p); }
			 * 
			 * p = getShortestInjuredPath(robots.get(i).getLocation()); if (p != null) {
			 * p.setColor(Color.RED); displayPaths.add(p); }
			 */
		}
		// long end = System.currentTimeMillis();
		// RescueFramework.log("Robot decision time: "+(end-start)+" ms");

		if (!movingRobot) {
			RescueFramework.finish();
			RescueFramework.log("No moving robot. Simulation is finished.");
			RescueFramework.log(MainFrame.map.getTotalScore());
			if (RescueFramework.autoClose) {
				System.exit(0);
			}
		}

		RescueFramework.refresh();
	}

	private Robot getRobotAtThisCell(List<Robot> robots, Cell cell) {
		if(!cell.hasRobot())
			return null;
		
		for(int i = 0; i < robots.size(); i++) {
			final Robot robot = robots.get(i);
			if(robot.getLocation().equals(cell))
				if(Type.ROBOT == robot.getType())
					return robot;
		}
		return null;
	}
	
	/**
	 * Disable the step thread
	 */
	public void disable() {
		enabled = false;
	}

	/**
	 * Returns true if the thread is enabled
	 * 
	 * @return True if the thread is enabled
	 */
	public boolean isEnabled() {
		return enabled;
	}

	/**
	 * Change the time of a single step
	 * 
	 * @param stepTime
	 *            The new duration of a single step
	 */
	public void setStepTime(int stepTime) {
		this.timeStep = stepTime;
		if (timeLeft > timeStep)
			timeLeft = 1000-timeStep;
	}
}