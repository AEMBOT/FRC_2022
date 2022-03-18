package frc.robot.hardware;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** A class for interfacing with a Limelight camera. */
public class Limelight {
  /** An enum to represent the LED mode of the Limelight. */
  public static enum LEDMode {
    Pipeline(0),
    Off(1),
    Blink(2),
    On(3);

    public int value;

    LEDMode(int value) {
      this.value = value;
    }
  }

  /** An enum to represent the camera mode of the limelight. */
  public static enum CameraMode {
    Vision(0),
    Driver(1);

    public int value;

    CameraMode(int value) {
      this.value = value;
    }
  }

  // Creates uninitialized variables to hold limelight table, x offset, y offset
  // and object area
  private NetworkTableEntry m_tx;
  private NetworkTableEntry m_ty;
  private NetworkTableEntry m_tv;
  private NetworkTableEntry m_ledMode;
  private NetworkTableEntry m_camMode;

  /** Create and collect tables and entries */
  public Limelight() {
    NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    // Initializes entries for limelight values
    m_tx = limelightTable.getEntry("tx"); // X degrees
    m_ty = limelightTable.getEntry("ty"); // Y degrees
    m_tv = limelightTable.getEntry("tv"); // Target valid
    m_ledMode = limelightTable.getEntry("ledMode");
    m_camMode = limelightTable.getEntry("camMode");
  }

  /**
   * Will get the current X offset value if no object is detected it will default to 0
   *
   * @return the X offset in degrees to the target
   */
  public double getX() {
    return m_tx.getNumber(Double.NaN).doubleValue();
  }

  /**
   * Will get the current Y offset value if no object is detected it will default to 0
   *
   * @return Y offset to target in degrees
   */
  public double getY() {
    return m_ty.getNumber(Double.NaN).doubleValue();
  }

  /**
   * Gets whether or not a target can be seen by the camera
   *
   * @return the visibility of the target
   */
  public boolean hasValidTarget() {
    return m_tv.getNumber(0).intValue() == 1;
  }

  /** Sets the Limelight's LED mode. */
  public void setLEDMode(LEDMode mode) {
    m_ledMode.setNumber(mode.value);
  }

  /** Sets the Limelight's camera mode. */
  public void setCameraMode(CameraMode mode) {
    m_camMode.setNumber(mode.value);
  }
}
