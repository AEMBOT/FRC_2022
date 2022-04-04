// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Gets values from a color sensor, and converts them to HSV. Accessors included to use for indexing
// purposes.

package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;

/**
 * A subsystem encompassing the robot's indexer belts as well as the color sensor located in the
 * indexer.
 */
public class IndexerSubsystem extends SubsystemBase {
  // Belt motors
  private final CANSparkMax m_upperTopBelt =
      new CANSparkMax(kCANIndexerUpperTopBeltID, MotorType.kBrushless);
  private final CANSparkMax m_lowerTopBelt =
      new CANSparkMax(kCANIndexerLowerTopBeltID, MotorType.kBrushless);
  private final CANSparkMax m_upperBottomBelt =
      new CANSparkMax(kCANIndexerUpperBottomBeltID, MotorType.kBrushless);
  private final CANSparkMax m_lowerBottomBelt =
      new CANSparkMax(kCANIndexerLowerBottomBeltID, MotorType.kBrushless);

  // Color sensor for detecting cargo
  // TODO: Move this to the MXP port (i.e. to the NavX) to avoid lockups
  // https://docs.wpilib.org/en/stable/docs/yearly-overview/known-issues.html#onboard-i2c-causing-system-lockups
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  // Get the alliance from the FMS for dashboard display purposes
  private final Alliance m_alliance = DriverStation.getAlliance();

  // Create a specific Network Table for indexer information
  private final NetworkTable m_indexerTable =
      NetworkTableInstance.getDefault().getTable("Robot").getSubTable("Indexer");
  private final NetworkTableEntry m_cargoCorrect = m_indexerTable.getEntry("cargoCorrect");
  private final NetworkTableEntry m_cargoInIndexer = m_indexerTable.getEntry("cargoInIndexer");

  /** Constructs a new IndexerSubsystem, configuring the belt motors. */
  public IndexerSubsystem() {
    // Restore motors to factory defaults for consistent settings
    m_upperTopBelt.restoreFactoryDefaults();
    m_upperBottomBelt.restoreFactoryDefaults();
    m_lowerBottomBelt.restoreFactoryDefaults();
    m_lowerTopBelt.restoreFactoryDefaults();

    // Indexer belts shouldn't continue moving after stopping
    m_upperTopBelt.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_upperBottomBelt.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_lowerBottomBelt.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Lower intake belt should follow the upper one
    m_upperBottomBelt.follow(m_upperTopBelt, true);

    // Bottom intake belt should follow the inner roller belt
    m_lowerBottomBelt.follow(m_lowerTopBelt);
  }

  @Override
  public void periodic() {
    // Write diagnostic info to network tables periodically
    m_cargoCorrect.setBoolean(correctCargoColor());
    m_cargoInIndexer.setBoolean(cargoInIndexer());
  }

  // INDEXER BELT CONTROL

  /** Moves cargo towards the shooter. */
  public void moveCargoUp() {
    m_upperTopBelt.set(-0.7);
    m_lowerTopBelt.set(0.7);
  }

  /** Moves cargo down towards the intake. */
  public void moveCargoDown() {
    m_upperTopBelt.set(0.7);
    m_lowerTopBelt.set(-0.7);
  }

  /**
   * Runs only the lower indexer belt. Meant to be used when intaking cargo to avoid accidentally
   * ejecting any cargo currently in the indexer.
   */
  public void intakeCargo() {
    m_lowerTopBelt.set(0.7);
  }

  /** Stops all intake belts. */
  public void stopBelts() {
    m_upperTopBelt.set(0);
    m_lowerTopBelt.set(0);
  }

  // COLOR SENSOR READING

  /** Returns true if there is a piece of cargo in the indexer. */
  public boolean cargoInIndexer() {
    return m_colorSensor.getProximity() >= kCargoMinProximity;
  }

  /**
   * Returns ture if the cargo in the indexer is the correct color (i.e. matches our alliance
   * color).
   */
  public boolean correctCargoColor() {
    return getCargoAllianceColor() == m_alliance;
  }

  /** Return the alliance that the loaded cargo belongs to, based on calibrated color tolerances. */
  public Alliance getCargoAllianceColor() {
    double[] hsv = readSensorHSV();

    double hue = hsv[0];
    double saturation = hsv[1];
    double value = hsv[2];

    Alliance alliance;

    // Red cargo
    if (hue >= kMinRedHue
        && hue <= kMaxRedHue
        && saturation >= kMinRedSaturation
        && saturation <= kMaxRedSaturation
        && value >= kMinRedValue
        && value <= kMaxRedValue) {
      alliance = Alliance.Red;
    }

    // Blue cargo
    else if (hue >= kMinBlueHue
        && hue <= kMaxBlueHue
        && saturation >= kMinBlueSaturation
        && saturation <= kMaxBlueSaturation
        && value >= kMinBlueValue
        && value <= kMaxBlueValue) {
      alliance = Alliance.Blue;
    }

    // Something other than cargo
    else {
      alliance = Alliance.Invalid;
    }

    return alliance;
  }

  /** Returns an array of HSV values as read from the color sensor. */
  private double[] readSensorHSV() {
    int red = m_colorSensor.getRed();
    int green = m_colorSensor.getGreen();
    int blue = m_colorSensor.getBlue();

    // HSV and HSB are the same thing, so ignore this difference
    float[] output = Color.RGBtoHSB(red, green, blue, null);
    return new double[] {output[0] * 360, output[1] * 100, output[2] * 100};
  }
}
