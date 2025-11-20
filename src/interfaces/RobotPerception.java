package interfaces;

import java.util.List;

import world.Path;

/**
 * Interface defining the access function of the robot towards the map
 */
public interface RobotPerception {

	/** Return all known exit cells */
	public List<CellInfo> getExitCells();

	/** Return all discovered injured */
	public List<InjuredInfo> getDiscoveredInjureds();

	/** Return all discovered injured in the health range specified */
	public List<InjuredInfo> getDiscoveredInjureds(int maxHealth, int minHealth);

	/** Returns the shortest path to a known exit cell */
	public Path getShortestExitPath(CellInfo start);

	/** Returns the shortest path to an unknown cell */
	public Path getShortestUnknownPath(CellInfo start);

	/** Returns the shortest path to an injured */
	public Path getShortestInjuredPath(CellInfo start);

	/** Returns the shortest path to an injured below a given health limit */
	public Path getShortestInjuredPath(CellInfo start, int maxHealth, int minHealth);
	
}
