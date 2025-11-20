package world;

import interfaces.InjuredInfo;

/**
 * Class representing injured people on the map
 */
public class Injured implements InjuredInfo{
    
    /**The maximum health an injured can have */
    final public static int MAXHEALTH = 1000;
    /** Healt level */
    private int health = MAXHEALTH;
    /** True if the injured have already been transported to an exit cell */
    private boolean saved = false;
    /** True if the injured is already discovered by a robot */
    private boolean discovered = false;
    /** Cell location of the injured */
    private Cell location = null;
    /** Static ID value of the next injured */
    private static int nextID = 1;
    /** ID of the injured object */
    public int id;
    

    /**
     * Returns the location cell of the injured
     * @return      The location cell of the injured
     */
    public Cell getLocation() {
        return location;
    }

    /**
     * Set the location cell of the injured
     * @param location      The location cell of the injured
     */
    public void setLocation(Cell location) {
        this.location = location;
    }
    
    
    /**
     * Default constructor
     * 
     * @param health        Health level of the new injured
     */
    public Injured(int health) {
        // Save health
        this.health = health;
        
        // Generate unique ID
        id = nextID;
        nextID++;
    }
    
    /**
     * Return the health value of the injured
     * @return      Health value of the injured
     */
    public int getHealth() {
        return health;
    }
    
    /**
     * Set the health value of the injured
     * @param health        Health value of the injured
     */
    public void setHealth(int health) {
        this.health = health;
    }
    
    /**
     * Mark the injured as saved
     */
    public void setSaved() {
        saved = true;
    }
    
    /**
     * Returns true if the injured is already saved
     * 
     * @return      True if the injured is already saved
     */
    public boolean isSaved() {
        return saved;
    }
    
    /**
     * Returns the health level in percents
     * @return      The health level in percents
     */
    public float getHealthRatio() {
        return (float)health/1000F;
    }
    
    /**
     * Returns true if the injured is alredy discovered
     * @return      True if the injured is alredy discovered
     */
    public boolean isDiscovered() {
        return discovered;
    }
    
    /**
     * Returns true if the injured is still alive
     * @return      True if the injured is still alive
     */
    public boolean isAlive() {
        return health>0;
    }
    
    /**
     * Return the injured ID and health level in a string
     * @return      The injured ID and health level in a string 
     */
    public String toString() {
        return "ID="+id+" with health "+health;
    }
    
    /**
     * Mark the injured as discovered
     * @param discovered        Set true to mark the injured as discovered
     */
    public void setDiscovered(boolean discovered) {
        this.discovered = discovered;
    }
    
    
    protected void addHealth(int amount) {
        if (health>0) {
            // Only alive patients can be healed
            health += amount;
        }
        if (health>MAXHEALTH) {
            health = MAXHEALTH;
        }
    }
 }
