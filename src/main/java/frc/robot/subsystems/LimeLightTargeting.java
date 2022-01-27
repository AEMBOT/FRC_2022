package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Class created to manage limelight data
 * 
 * @author Will Richards, Goirick Saha
 */

public class LimeLightTargeting {

    // Creates uninitialized variables to hold limelight table, x offset, y offset
    // and object area
    private NetworkTable limelightTable;
    private NetworkTableEntry tx;
    private static NetworkTableEntry ty;
    private NetworkTableEntry ta;
    private NetworkTableEntry tv;

    // Variables for calculating distance
    private static double targetHeight;
    private static double camHeight;
    private static double camAngle;
    private static double distance;

    /**
     * Create and collect tables and entries
     */
    public LimeLightTargeting() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // Initializes entries for limelight values
        tx = limelightTable.getEntry("tx"); // X degrees
        ty = limelightTable.getEntry("ty"); // Y degrees
        ta = limelightTable.getEntry("ta"); // Target area
        tv = limelightTable.getEntry("tv"); // Target valid
    }

    /**
     * Will get the current X offset value if no object is detected it will default
     * to 0
     * 
     * @return the X offset in degrees to the target
     */
    public double getX() {
        return tx.getDouble(0.0);
    }

    /**
     * Gets whether or not a target can be seen by the camera
     * 
     * @return the visibility of the target
     */
    public boolean hasValidTarget() {
        if(tv.getDouble(0.0) > 0)
            return true;
        else
            return false;
    }

    /**
     * Will get the current Y offset value if no object is detected it will default
     * to 0
     * 
     * @return Y offset to target in degrees
     */
    public static double getY() {
        return ty.getDouble(0.0);
    }

    /**
     * Will get the current area value
     * 
     * @return returns area, if no objects are detected it returns 0
     */
    public double getArea() {
        return ta.getDouble(0.0);
    }

    /**
     * @return returns distance in inches to target
     */
    public static double getDistance() {
        camAngle = 9.4623;
        camHeight = 12;
        targetHeight = 103.625;
        distance = (targetHeight - camHeight) / (Math.tan(camAngle + getY()));
        return distance;
    }

    /**
     * Will turn on the LED so the camera can track the reflective tape
     */
    public void turnOnLED() {
        limelightTable.getEntry("ledMode").setNumber(3);
    }

    /**
     * Will turn off the LED so we dont blind everyone as the LED is very bright
     */
    public void turnOffLED() {
        limelightTable.getEntry("ledMode").setNumber(1);
    }

    /**
     * Allows for getting the current camera mode
     * 
     * @return an int (1 or 0) that specifies what mode it is in
     */
    public double getCamMode() {
        return limelightTable.getEntry("camMode").getDouble(0.0);
    }

    /**
     * Allows for manual camera mode setting
     * 
     * @param mode pass in 1 or 0 to switch
     */
    public void setCamMode(double mode) {
        limelightTable.getEntry("camMode").setNumber(mode);
    }
}