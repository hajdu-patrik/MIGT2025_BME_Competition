package rescueframework;

import java.awt.Frame;
import java.awt.KeyboardFocusManager;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.File;
import java.util.ArrayList;
import java.util.Random;

import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import rescueagents.AMSService;
import rescueagents.StaticSensorControl;
import world.Cell;
import world.FlyingDrone;
import world.Injured;
import world.Map;
import world.MedicalRobot;
import world.PaintPanel;
import world.RescueRobot;
import world.Robot;
import world.StaticSensor;

import static java.lang.Integer.parseInt;

/**
 * Main frame of the simulator
 */
public class MainFrame extends javax.swing.JFrame {

	enum SimulationStatus {
		PAUSED, RUNNING, FINISHED
	};

	// string constants
	private static final String SPEED = "speed";
	private static final String STATIC_SENSOR_COUNT = "static_sensor_count";
	private static final String DRONE_AGENT_COUNT = "drone_agent_count";
	private static final String MED_AGENT_COUNT = "med_agent_count";
	public static final String RESC_AGENT_COUNT = "resc_agent_count";

	private boolean updateGUI = true;

	public static SimulationStatus status = SimulationStatus.PAUSED;

	/** Auto step thread of the frame */
	private StepThread stepThread = new StepThread();

	// Variables declaration - do not modify//GEN-BEGIN:variables
	private javax.swing.JButton loadBtn;
	private javax.swing.JButton stepBtn;
	private javax.swing.JButton runBtn;
	private javax.swing.JCheckBox agentPerspectiveCB;
	private javax.swing.JComboBox<String> mapFileNamesCB;
	private javax.swing.JLabel jLabel2;
	private javax.swing.JLabel rescueLabel;
	// private javax.swing.JLabel runSpeedLabel;
	private javax.swing.JLabel medicLabel;
	private javax.swing.JLabel droneLabel;
	private javax.swing.JLabel sensorLabel;
	private javax.swing.JPanel jPanel1;
	private javax.swing.JPanel jPanel3;
	private javax.swing.JSlider simulationSpeedSlider;
	private javax.swing.JSpinner rescueAgentCountSpinner;
	private javax.swing.JSpinner medicalAgentCountSpinner;
	private javax.swing.JSpinner droneAgentCountSpinner;
	private javax.swing.JSpinner staticSensorCountSpinner;
	private world.PaintPanel paintPanel;
	/** Robots operating on the map */
	public static ArrayList<Robot> robots = new ArrayList<>();
	/** Injureds already transported outside by the robots */
	public static ArrayList<Injured> savedInjureds = new ArrayList<>();
	private final int MEDICAL_ROBOT_START_MEDICINE_PER_INJURED = 100;
	public static Cell startCell = null;
	private static int randomSeed = 0;
	private static Random generator = new Random(randomSeed);

	/** The map the simulator uses */
	public static Map map = null;
	/** The map the robots use */ // (internalWorld)
	public static Map discovered = null;
	/** Simulation time */
	static int time = 0;
	/** found injured people on the map */
	public static ArrayList<Injured> foundInjureds = new ArrayList<>();
	private int btnWidth = 80;
	private int btnHeight = 23;

	// End of variables declaration//GEN-END:variables

	/**
	 * Creates new form MainFrame
	 */
	public MainFrame(String[] args) {

		// Init auto generated components
		initComponents();
		 paintPanel.setMaps(map,discovered,agentPerspectiveCB.isSelected());

		// Load all files from the "maps" subfolder
		mapFileNamesCB.removeAllItems();
		File folder = new File("maps");
		File[] listOfFiles = folder.listFiles();

		// Add files as options to the JComboBox

		String lastMap = "";
		if (args.length == 6) {
			lastMap = args[0];
			rescueAgentCountSpinner.setValue(Integer.valueOf(args[1]));
			medicalAgentCountSpinner.setValue(Integer.valueOf(args[2]));
			droneAgentCountSpinner.setValue(Integer.valueOf(args[3]));
			staticSensorCountSpinner.setValue(Integer.valueOf(args[4]));

			updateGUI = args[5].equals("1");
			simulationSpeedSlider.setValue(500);
			randomSeed = parseInt(args[6]);

		} else {
			lastMap = Settings.getString("map", "");
			// Load agent count and simulation speed
			rescueAgentCountSpinner.setValue(Settings.getInt(RESC_AGENT_COUNT, 1));
			medicalAgentCountSpinner.setValue(Settings.getInt(MED_AGENT_COUNT, 1));
			droneAgentCountSpinner.setValue(Settings.getInt(DRONE_AGENT_COUNT, 0));
			staticSensorCountSpinner.setValue(Settings.getInt(STATIC_SENSOR_COUNT, 0));
			simulationSpeedSlider.setValue(Settings.getInt(SPEED, 500));
		}
		int selectedIndex = -1;
		for (int i = 0; i < listOfFiles.length; i++) {
			if (listOfFiles[i].isFile()) {
				String fileName = listOfFiles[i].getName();
				mapFileNamesCB.addItem(fileName);
				if (fileName.equals(lastMap))
					selectedIndex = i;
			}
		}

		// Select the last used map based on the saved settings
		if (selectedIndex >= 0)
			mapFileNamesCB.setSelectedIndex(selectedIndex);

		// Load frame position, size and state from the saved settings
		setBounds(Settings.getInt("left", 0), Settings.getInt("top", 0), Settings.getInt("width", 1200),
				Settings.getInt("height", 800));
		if (Settings.getInt("maximized", 0) == 1) {
			setExtendedState(getExtendedState() | Frame.MAXIMIZED_BOTH);
		}

		// Move focus to the first button to avoid keyboard control from chaning the
		// jSpinner1 value
		loadBtnActionPerformed(null);
		loadBtn.requestFocus();

		// Key listener for keyboard robot control
		KeyboardFocusManager manager = KeyboardFocusManager.getCurrentKeyboardFocusManager();
		// manager.addKeyEventDispatcher(new MyDispatcher());

		// Window listener to detect window close event
		addWindowListener(new WindowAdapter() {
			public void windowClosing(WindowEvent e) {
				// Determine if the window is maximized
				if ((getExtendedState() & Frame.MAXIMIZED_BOTH) == 0) {
					Settings.setInt("maximized", 0);
				} else {
					Settings.setInt("maximized", 1);
				}

				// Save window position and size after unmaximizing it
				setExtendedState(0);
				Settings.setInt("top", getY());
				Settings.setInt("left", getX());
				Settings.setInt("width", getWidth());
				Settings.setInt("height", getHeight());
			}
		});

		// Start autostep thread
		stepThread.start();
		/*
		 * if (args.length == 6) { int rescAgentCount = (Integer)
		 * rescueAgentCountSpinner.getValue(); int medAgentCount = (Integer)
		 * medicalAgentCountSpinner.getValue(); int droneAgentCount = (Integer)
		 * droneAgentCountSpinner.getValue(); int staticSensorCount = (Integer)
		 * staticSensorCountSpinner.getValue(); map = new Map(lastMap, rescAgentCount,
		 * medAgentCount, droneAgentCount, staticSensorCount, true); //
		 * paintPanel.setMaps(map,discovered);
		 * 
		 * resume(); RescueFramework.autoClose = true; }
		 */
	}

	public static double random() {
		return generator.nextDouble();
	}

	public static int nextRandInt(int bound) {
		return generator.nextInt(bound);
	}

	public static int randomBetween(int minVal, int maxVal) { return (nextRandInt(maxVal + 1)) + minVal; }


	static void removeRandomWall() {
		if (nextRandInt(100) < 5) {
			int x = nextRandInt(map.cells.length);
			int y = nextRandInt(map.cells[x].length);

			ArrayList<Integer> walldirs = new ArrayList<>();
			for (int i = 0; i < 4; ++i) {
				if (map.cells[x][y].hasWallEx(i)) {
					walldirs.add(i);
				}
			}
			if (!walldirs.isEmpty()) {
				map.cells[x][y].removeWall(walldirs.get(nextRandInt(walldirs.size())));
			}
		}
	}

	/**
	 * Update the GUI to the latest state of the world
	 */
	public void refresh() {

		// Repaint cells and world objects
		paintPanel.repaint();

		// Calculate and display score
		if (map != null) {
			String label = map.getTotalScore();
			jLabel2.setText(label);
		} else {
			jLabel2.setText("No simulation.");
		}
	}

	/**
	 * This method is called from within the constructor to initialize the form.
	 * WARNING: Do NOT modify this code. The content of this method is always
	 * regenerated by the Form Editor.
	 */
	@SuppressWarnings("unchecked")
	// <editor-fold defaultstate="collapsed" desc="Generated
	// Code">//GEN-BEGIN:initComponents
	private void initComponents() {

		jPanel1 = new javax.swing.JPanel();
		mapFileNamesCB = new javax.swing.JComboBox<>();
		loadBtn = new javax.swing.JButton();
		agentPerspectiveCB = new javax.swing.JCheckBox();
		stepBtn = new javax.swing.JButton();
		runBtn = new javax.swing.JButton();
		simulationSpeedSlider = new javax.swing.JSlider();
		rescueAgentCountSpinner = new javax.swing.JSpinner();
		medicalAgentCountSpinner = new javax.swing.JSpinner();
		droneAgentCountSpinner = new javax.swing.JSpinner();
		staticSensorCountSpinner = new javax.swing.JSpinner();
		rescueLabel = new javax.swing.JLabel();
		// runSpeedLabel = new javax.swing.JLabel();
		medicLabel = new javax.swing.JLabel();
		droneLabel = new javax.swing.JLabel();
		sensorLabel = new javax.swing.JLabel();
		paintPanel = new world.PaintPanel();
		jPanel3 = new javax.swing.JPanel();
		jLabel2 = new javax.swing.JLabel();

		setDefaultCloseOperation(javax.swing.WindowConstants.EXIT_ON_CLOSE);
		setTitle("RescueFramework");
		setMinimumSize(new java.awt.Dimension(600, 400));
		setSize(new java.awt.Dimension(1200, 800));

		jPanel1.setPreferredSize(new java.awt.Dimension(924, 33));

		mapFileNamesCB.setModel(
				new javax.swing.DefaultComboBoxModel<>(new String[] { "Item 1", "Item 2", "Item 3", "Item 4" }));
		mapFileNamesCB.setFocusable(false);

		loadBtn.setText("Load");
		loadBtn.setFocusCycleRoot(true);

		loadBtn.setPreferredSize(new java.awt.Dimension(btnWidth, btnHeight));
		loadBtn.addActionListener((ActionListener) event -> {
			loadBtnActionPerformed(event);
		});

		agentPerspectiveCB.setSelected(true);
		agentPerspectiveCB.setText("Agent perspective");
		agentPerspectiveCB.addActionListener((ActionListener) event -> {
			agentPerspectiveCBStateChanged(event);
		});

		stepBtn.setText("Step");
		stepBtn.setPreferredSize(new java.awt.Dimension(btnWidth, btnHeight));
		stepBtn.addActionListener((ActionListener) event -> {
			stepBtnActionPerformed(event);
		});

		runBtn.setText("Run");
		runBtn.setPreferredSize(new java.awt.Dimension(btnWidth, btnHeight));
		runBtn.addActionListener((ActionListener) event -> {
			runBtnActionPerformed(event);
		});

		simulationSpeedSlider.setToolTipText("");
		simulationSpeedSlider.setMinimum(200);
		simulationSpeedSlider.setMaximum(999);
		simulationSpeedSlider.setValue(500);
		simulationSpeedSlider.addChangeListener((ChangeListener) event -> {
			simulationSpeedSliderStateChanged(event);
		});

		rescueAgentCountSpinner.setModel(new javax.swing.SpinnerNumberModel(1, 0, null, 1));
		rescueAgentCountSpinner.setFocusable(false);

		medicalAgentCountSpinner.setModel(new javax.swing.SpinnerNumberModel(1, 0, null, 1));
		medicalAgentCountSpinner.setFocusable(false);

		droneAgentCountSpinner.setModel(new javax.swing.SpinnerNumberModel(1, 0, null, 1));
		droneAgentCountSpinner.setFocusable(false);

		staticSensorCountSpinner.setModel(new javax.swing.SpinnerNumberModel(1, 0, null, 1));
		staticSensorCountSpinner.setFocusable(false);

		rescueLabel.setText("Rescue");

		medicLabel.setText("Medic");

		droneLabel.setText("Drone");

		sensorLabel.setText("Sensor");

		updateGUI = true;

		javax.swing.GroupLayout jPanel1Layout = new javax.swing.GroupLayout(jPanel1);
		jPanel1.setLayout(jPanel1Layout);
		jPanel1Layout.setHorizontalGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
				.addGroup(jPanel1Layout.createSequentialGroup().addContainerGap()
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
						.addComponent(mapFileNamesCB, javax.swing.GroupLayout.PREFERRED_SIZE, 100,
								javax.swing.GroupLayout.PREFERRED_SIZE)

						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED).addComponent(rescueLabel)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(rescueAgentCountSpinner, javax.swing.GroupLayout.PREFERRED_SIZE,
								javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)

						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED).addComponent(medicLabel)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(medicalAgentCountSpinner, javax.swing.GroupLayout.PREFERRED_SIZE,
								javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)

						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED).addComponent(droneLabel)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(droneAgentCountSpinner, javax.swing.GroupLayout.PREFERRED_SIZE,
								javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)

						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED).addComponent(sensorLabel)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(staticSensorCountSpinner, javax.swing.GroupLayout.PREFERRED_SIZE,
								javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(loadBtn, javax.swing.GroupLayout.PREFERRED_SIZE, btnWidth,
								javax.swing.GroupLayout.PREFERRED_SIZE)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(stepBtn, javax.swing.GroupLayout.PREFERRED_SIZE, btnWidth,
								javax.swing.GroupLayout.PREFERRED_SIZE)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(runBtn, javax.swing.GroupLayout.PREFERRED_SIZE, btnWidth,
								javax.swing.GroupLayout.PREFERRED_SIZE)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(simulationSpeedSlider, javax.swing.GroupLayout.PREFERRED_SIZE, 100,
								javax.swing.GroupLayout.PREFERRED_SIZE)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.RELATED)
						.addComponent(agentPerspectiveCB)
						.addPreferredGap(javax.swing.LayoutStyle.ComponentPlacement.UNRELATED)
						.addContainerGap(284, Short.MAX_VALUE)));
		jPanel1Layout.setVerticalGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
				.addGroup(jPanel1Layout.createSequentialGroup().addGap(4, 4, 4).addGroup(jPanel1Layout
						.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
						.addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
								.addComponent(agentPerspectiveCB))
						.addComponent(simulationSpeedSlider, javax.swing.GroupLayout.PREFERRED_SIZE,
								javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
						.addGroup(jPanel1Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.BASELINE)
								.addComponent(runBtn, javax.swing.GroupLayout.PREFERRED_SIZE,
										javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
								.addComponent(loadBtn, javax.swing.GroupLayout.PREFERRED_SIZE,
										javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
								.addComponent(mapFileNamesCB, javax.swing.GroupLayout.PREFERRED_SIZE,
										javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
								.addComponent(rescueAgentCountSpinner, javax.swing.GroupLayout.PREFERRED_SIZE,
										javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
								.addComponent(rescueLabel)
								.addComponent(stepBtn, javax.swing.GroupLayout.PREFERRED_SIZE,
										javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
								.addComponent(medicalAgentCountSpinner, javax.swing.GroupLayout.PREFERRED_SIZE,
										javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)

								.addComponent(droneLabel))
						.addComponent(droneAgentCountSpinner, javax.swing.GroupLayout.PREFERRED_SIZE,
								javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
						.addComponent(rescueLabel)
						.addComponent(stepBtn, javax.swing.GroupLayout.PREFERRED_SIZE,
								javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
						.addComponent(sensorLabel)
						.addComponent(staticSensorCountSpinner, javax.swing.GroupLayout.PREFERRED_SIZE,
								javax.swing.GroupLayout.DEFAULT_SIZE, javax.swing.GroupLayout.PREFERRED_SIZE)
						.addComponent(medicLabel))

						.addGap(13, 13, 13)));

		getContentPane().add(jPanel1, java.awt.BorderLayout.PAGE_START);

		paintPanel.addMouseListener(new MouseAdapter() {
			public void mouseClicked(MouseEvent evt) {
				paintPanelMouseClicked(evt);
			}
		});
		getContentPane().add(paintPanel, java.awt.BorderLayout.CENTER);

		jPanel3.setPreferredSize(new java.awt.Dimension(1241, 20));

		jLabel2.setText("No score yet.");

		javax.swing.GroupLayout jPanel3Layout = new javax.swing.GroupLayout(jPanel3);
		jPanel3.setLayout(jPanel3Layout);
		jPanel3Layout.setHorizontalGroup(
				jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING).addGroup(
						jPanel3Layout.createSequentialGroup().addComponent(jLabel2).addGap(0, 1176, Short.MAX_VALUE)));
		jPanel3Layout.setVerticalGroup(jPanel3Layout.createParallelGroup(javax.swing.GroupLayout.Alignment.LEADING)
				.addGroup(jPanel3Layout.createSequentialGroup().addComponent(jLabel2,
						javax.swing.GroupLayout.PREFERRED_SIZE, 14, javax.swing.GroupLayout.PREFERRED_SIZE)
						.addGap(0, 6, Short.MAX_VALUE)));

		getContentPane().add(jPanel3, java.awt.BorderLayout.SOUTH);

		pack();
	}// </editor-fold>//GEN-END:initComponents

	/**
	 * The user clicks the Load map button
	 * 
	 * @param evt
	 *            The click event
	 */
	private void loadBtnActionPerformed(ActionEvent evt) {// GEN-FIRST:event_jButton1ActionPerformed
		status = SimulationStatus.PAUSED;
		paintPanel.finished = false;
		// Save selected map and agent count
		String mapString = mapFileNamesCB.getSelectedItem() + "";
		int rescAgentCount = (Integer) rescueAgentCountSpinner.getValue();
		int medAgentCount = (Integer) medicalAgentCountSpinner.getValue();
		int droneAgentCount = (Integer) droneAgentCountSpinner.getValue();
		int staticSensorCount = (Integer) staticSensorCountSpinner.getValue();

		Settings.setString("map", mapString);
		Settings.setInt(RESC_AGENT_COUNT, rescAgentCount);
		Settings.setInt(MED_AGENT_COUNT, medAgentCount);
		Settings.setInt(DRONE_AGENT_COUNT, droneAgentCount);
		Settings.setInt(STATIC_SENSOR_COUNT, staticSensorCount);
		Settings.save();

		// Load the map from file
		generator = new Random(randomSeed);
		discovered = new Map(mapString, rescAgentCount, medAgentCount, droneAgentCount, staticSensorCount);
		time = 0;
		if (discovered.injureds != null)
			discovered.deleteInjureds();
		if (robots != null)
			robots.removeAll(robots);
		if (map != null && !map.injureds.isEmpty())
			map.injureds.removeAll(map.injureds);

		if (discovered.injureds != null && !discovered.injureds.isEmpty())
			discovered.injureds.removeAll(discovered.injureds);

		savedInjureds.removeAll(savedInjureds);

		// set the initial version of the robots internal world model
		AMSService.setInternalWorldModel(discovered);

		map = new Map(mapString, rescAgentCount, medAgentCount, droneAgentCount, staticSensorCount, true);

		 paintPanel.setMaps(map,discovered,agentPerspectiveCB.isSelected());
		// Init agents
		if (startCell == null)
			startCell = map.getCell(0, 0);
		Cell nextStartCell = startCell;

		for (int i = 0; i < rescAgentCount; i++) {
			Robot newRobot = new RescueRobot(nextStartCell, discovered);
			robots.add(newRobot);
			if (nextStartCell != null) {
				map.updateRobotPresence(nextStartCell);
				discovered.updateRobotPresence(nextStartCell);
				nextStartCell = nextStartCell.getAccessibleNeigbourEx(1);
				if(nextStartCell ==null)
					nextStartCell = startCell.getAccessibleNeigbourEx(2);
			}
		}

		for (int i = 0; i < medAgentCount; i++) {
			Robot newRobot = new MedicalRobot(nextStartCell, discovered,
					MEDICAL_ROBOT_START_MEDICINE_PER_INJURED * map.injureds.size());
			robots.add(newRobot);
			if (nextStartCell != null) {
				map.updateRobotPresence(nextStartCell);
				discovered.updateRobotPresence(nextStartCell);
				nextStartCell = nextStartCell.getAccessibleNeigbourEx(1);
				if(nextStartCell ==null)
					nextStartCell = startCell.getAccessibleNeigbourEx(2);						
			}
		}

		for (int i = 0; i < droneAgentCount; i++) {
			Robot newRobot = new FlyingDrone(nextStartCell, discovered);
			robots.add(newRobot);
			if (nextStartCell != null) {
				map.updateRobotPresence(nextStartCell);
				discovered.updateRobotPresence(nextStartCell);
				nextStartCell = nextStartCell.getAccessibleNeigbourEx(1);
				if(nextStartCell ==null)
					nextStartCell = startCell.getAccessibleNeigbourEx(2);
			}
		}
		
		for (int i = 0; i < staticSensorCount; i++) {
			Cell staticStartCell = null;

			while (staticStartCell == null) {
				int xCoord = StaticSensorControl.generateXCoord(map.getWidth(), map.getHeight());
				int yCoord = StaticSensorControl.generateYCoord(map.getWidth(), map.getHeight());

				xCoord += generator.nextInt(8) - 4;
				yCoord += generator.nextInt(8) - 4;

				staticStartCell = map.getCell(xCoord, yCoord);

				if (staticStartCell != null && staticStartCell.hasInjuredEx() && staticStartCell.hasRobot()) {
					staticStartCell = null;
				}
			}

			Robot sensor = new StaticSensor(staticStartCell, discovered);

			robots.add(sensor);
			map.updateRobotPresence(staticStartCell);
			discovered.updateRobotPresence(staticStartCell);
		}


		map.maxEnergy = map.getRobotsEnergyLevel();

		// Update the GUI and disable autostep
		runBtn.setText("Run");
		if (stepThread != null)
			stepThread.disable();
		map.updateAllRobotVisibleCells(true);
		discovered.updateAllRobotVisibleCells(false);

		RescueFramework.log("Map " + mapString + " loaded, ready to start.");

		refresh();
	}// GEN-LAST:event_jButton1ActionPerformed

	/**
	 * The user toggles the Agent perspective checkbox
	 * 
	 * @param e
	 *            The state change event
	 */
	private void agentPerspectiveCBStateChanged(ActionEvent e) {// GEN-FIRST:event_jCheckBox1StateChanged
		// Repaint the map with the updated robot visibility settings
		paintPanel.changeMaps();
		refresh();
	}// GEN-LAST:event_jCheckBox1StateChanged

	/**
	 * The user clicks the single step button
	 * 
	 * @param evt
	 *            The click event
	 */
	private void stepBtnActionPerformed(ActionEvent evt) {// GEN-FIRST:event_jButton3ActionPerformed
		// Make one time step
		stepThread.stepTime();
	}// GEN-LAST:event_jButton3ActionPerformed

	/**
	 * The pause / resume button is clicked
	 * 
	 * @param evt
	 *            The click event
	 */
	private void runBtnActionPerformed(ActionEvent evt) {// GEN-FIRST:event_jButton4ActionPerformed
		if (stepThread.isEnabled()) {
			pause();
		} else {
			resume();
			RescueFramework.autoClose = false;
		}

		refresh();
	}// GEN-LAST:event_jButton4ActionPerformed

	/**
	 * The speed bar is being moved
	 * 
	 * @param evt
	 *            The change event
	 */
	private void simulationSpeedSliderStateChanged(ChangeEvent evt) {// GEN-FIRST:event_jSlider1StateChanged
		if (stepThread != null) {
			// Update the simulation speed of the stepThread
			stepThread.setStepTime(getSimulationSpeed());
			// Save speed settings
			Settings.setInt(SPEED, getSimulationSpeed());
		}
	}// GEN-LAST:event_jSlider1StateChanged

	/**
	 * Mouse clicked on a cell
	 * 
	 * @param evt
	 *            The click event
	 */
	private void paintPanelMouseClicked(MouseEvent evt) {// GEN-FIRST:event_paintPanelMouseClicked
		// Make the paintPanel handle the click event
		paintPanel.mouseClicked(evt.getX(), evt.getY());
	}// GEN-LAST:event_paintPanelMouseClicked

	private void updateGuiCBStateChanged(ActionEvent e) {// GEN-FIRST:event_jCheckBox2StateChanged
		paintPanel.setActive(updateGUI);
	}// GEN-LAST:event_jCheckBox2StateChanged

	/**
	 * Pause the autostep thread
	 */
	public void pause() {
		stepThread.disable();
		runBtn.setText("Run");
		status = SimulationStatus.PAUSED;
	}

	/**
	 * Finishes the painting
	 */
	public void finish() {
		stepThread.disable();
		paintPanel.finished = true;
		paintPanel.repaint();
		runBtn.setText("Reset");
		status = SimulationStatus.FINISHED;
	}

	/**
	 * Resume the paused autostep thread
	 */
	public void resume() {
		if (status == SimulationStatus.FINISHED) {
			loadBtnActionPerformed(null);
			runBtn.setText("Run");
			status = SimulationStatus.PAUSED;
		} else {
			runBtn.setText("Pause");
			status = SimulationStatus.RUNNING;
			stepThread.enable();
		}
	}

	/**
	 * Return true if agent perspective is enabled
	 * 
	 * @return True if agent perspective is enabled
	 */
	public boolean isFogEnabled() {
		return agentPerspectiveCB.isSelected();
	}

	public static int getTime() {
		return time;
	}

	/**
	 * Custom KeyEventDispatcher to cach keyboard event globally in the application
	 */

	/*
	 * private class MyDispatcher implements KeyEventDispatcher { /** Custom
	 * dispatchKeyEvent function to process key events
	 * 
	 * @param e The KeyEvent to process
	 * 
	 * @return Always returns false
	 */
	/*
	 * @Override public boolean dispatchKeyEvent(KeyEvent e) { if (e.getID() ==
	 * KeyEvent.KEY_PRESSED) { // Only process up-down-left-right keys if
	 * (e.getKeyCode() >= 37 && e.getKeyCode() <= 40) { int dir = (e.getKeyCode() -
	 * 34) % 4; // Move the first robot if there is one Robot r =
	 * map.getRobots().get(0); if (r != null) { map.moveRobot(r, dir);
	 * map.stepTime(false); } } } return false; } }
	 */
	/**
	 * Determine simulation speed based on the jSlider1 settings
	 * 
	 * @return The 3-103 sleep time for the StepThread
	 */
	public int getSimulationSpeed() {
		return simulationSpeedSlider.getValue();
	}

	public PaintPanel getPaintPanel() {
		return paintPanel;
	}

}
