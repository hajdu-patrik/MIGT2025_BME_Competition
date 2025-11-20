package rescueframework;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

/**
 * Static class for reading and writing settings to the settings.txt file
 */
public class Settings {
    /** The Properties object storing the settings */
    private static Properties properties = new Properties();
    
    /**
     * Load settings from file
     */
    public static void load(String fileName) {
        InputStream configFile = null;
		
        // Open file
        try {
            configFile = new FileInputStream(fileName);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        // Read configuration
        try {
            properties.load(configFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    /**
     * Return string value from the settings
     * @param key               The key to read value from
     * @param defaultValue      Default value if the key does not exists
     * @return                  String value belonging the the key
     */
    public static String getString(String key, String defaultValue) {
        return properties.getProperty(key,defaultValue);
    }
    
    /**
     * Return int value from the settings
     * @param key               The key to read value from
     * @param defaultValue      Default value if the key does not exists
     * @return                  Integer value belonging the the key
     */
    public static int getInt(String key, int defaultValue) {
        return Integer.parseInt(properties.getProperty(key, String.valueOf(defaultValue)));
    }
    
    /**
     * Save the settings to file
     */
    public static void save() {
        try {
            properties.store(new FileOutputStream("settings.txt"), null);
        } catch (IOException e) {
            e.printStackTrace();
        }    
    }
    
    /**
     * Update the settings and save to file
     * @param key           The key to update
     * @param value         The string value to update with
     */
    public static void setString(String key, String value) {
        properties.setProperty(key, value);
        save();
    }
    
    /**
     * Update the settings and save to file
     * @param key           The key to update
     * @param value         The int value to update with
     */
    public static void setInt(String key, int value) {
        properties.setProperty(key, value+"");
        save();
    }
}
