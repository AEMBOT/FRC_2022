// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Gets values from a color sensor, and converts them to HSV. Accessors included to use for indexing
// purposes.

package frc.robot.subsystems;

import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;

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

public class IndexerSubsystem extends SubsystemBase {
  // Belt motors
  private final CANSparkMax m_lowerBelt =
      new CANSparkMax(kIndexerUpperBottomBeltPort, MotorType.kBrushless);
  private final CANSparkMax m_upperBelt =
      new CANSparkMax(kIndexerTopBeltPort, MotorType.kBrushless);

  // Color sensor for detecting cargo
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  // Get the alliance from the FMS for dashboard display purposes
  private final Alliance m_alliance = DriverStation.getAlliance();

  // Create a specific Network Table for indexer information
  private final NetworkTable m_indexerTable =
      NetworkTableInstance.getDefault().getTable("Robot").getSubTable("Indexer");
  private final NetworkTableEntry m_cargoCorrect = m_indexerTable.getEntry("cargoCorrect");
  private final NetworkTableEntry m_cargoInIndexer = m_indexerTable.getEntry("cargoInIndexer");

  public IndexerSubsystem() {
    // Indexer shouldn't continue moving after stopping
    m_lowerBelt.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_upperBelt.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Lower intake belt should follow the upper one
    m_lowerBelt.follow(m_upperBelt, true);
  }

  @Override
  public void periodic() {
    // Write diagnostic info to network tables
    m_cargoCorrect.setBoolean(correctCargoColor());
    m_cargoInIndexer.setBoolean(cargoInIndexer());
  }

  // INDEXER BELT CONTROL

  /** Moves cargo towards the shooter. */
  public void moveCargoUp() {
    m_upperBelt.set(-0.7);
  }

  /** Moves cargo down towards the intake. */
  public void moveCargoDown() {
    m_upperBelt.set(0.7);
  }

  /** Stops the upper intake belt. */
  public void stopBelts() {
    m_upperBelt.set(0);
  }

  // COLOR SENSOR READING

  /** Returns true if there is a piece of cargo in the indexer. */
  public boolean cargoInIndexer() {
    return m_colorSensor.getProximity() >= kCargoMinProximity;
    // && getCargoAllianceColor() != Alliance.Invalid;
  }

  public boolean correctCargoColor() {
    return getCargoAllianceColor() == m_alliance;
  }

  /** Return the alliance that the loaded cargo belongs to */
  public Alliance getCargoAllianceColor() {
    double[] hsv = readSensorHSV();

    double hue = hsv[0];
    double saturation = hsv[1];
    double value = hsv[2];

    Alliance color = Alliance.Invalid;

    // Red cargo
    if (hue >= kMinRedHue
        && hue <= kMaxRedHue
        && saturation >= kMinRedSaturation
        && saturation <= kMaxRedSaturation
        && value >= kMinRedValue
        && value <= kMaxRedValue) {
      color = Alliance.Red;
    }

    // Blue cargo
    else if (hue >= kMinBlueHue
        && hue <= kMaxBlueHue
        && saturation >= kMinBlueSaturation
        && saturation <= kMaxBlueSaturation
        && value >= kMinBlueValue
        && value <= kMaxBlueValue) {
      color = Alliance.Blue;
    } else {
      color = Alliance.Invalid;
    }

    return color;
  }

  // TODO: Move this to the utilities folder?
  private double[] readSensorHSV() {
    double red, blue, green;

    red = m_colorSensor.getRed();
    blue = m_colorSensor.getBlue();
    green = m_colorSensor.getGreen();

    // does math to make hsv
    red /= 255;
    blue /= 255;
    green /= 255;

    double cmax = Math.max(red, Math.max(green, blue));
    double cmin = Math.min(red, Math.min(green, blue));
    double diff = cmax - cmin;
    double h = -1;
    double s = -1;

    // if cmax and cmin are equal then h = 0
    if (cmax == cmin) h = 0;

    // if cmax equal r then compute h
    else if (cmax == red) h = (60 * ((green - blue) / diff) + 360) % 360;

    // if cmax equal g then compute h
    else if (cmax == green) h = (60 * ((blue - red) / diff) + 120) % 360;

    // if cmax equal b then compute h
    else if (cmax == blue) h = (60 * ((red - green) / diff) + 240) % 360;

    // if cmax equal zero
    if (cmax == 0) s = 0;
    else s = (diff / cmax) * 100;

    // compute v
    double v = cmax * 100;

    // updates dashboard with latest RGB and HSV readings
    // SmartDashboard.putNumber("Red", red);
    // SmartDashboard.putNumber("Blue", blue);
    // SmartDashboard.putNumber("Green", green);

    // SmartDashboard.putNumber("Hue", h);
    // SmartDashboard.putNumber("Saturation", s);
    // SmartDashboard.putNumber("Value", v);

    return new double[] {h, s, v};
  }
}
