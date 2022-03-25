// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.*;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.ClimberArm;
import frc.robot.hardware.LIDAR07_100W_Module;
import java.time.Clock;

public class ClimberSubsystem extends SubsystemBase {

  public enum ClimberState {
    kExtending,
    kRetracting,
    kCoasting
  }

  private ClimberState climberState;

  private ClimberArm leftArm;
  private ClimberArm rightArm;

  private Solenoid anglePistonsControl;

  private LIDAR07_100W_Module m_sensor;

  // Roughly the distance from no extension (0) to full extension (~600mm)
  private double m_previousDistance = 0;
  private double m_currentDistance = 0;
  private double m_currentVelocity = 0;
  private long m_previousTimestamp = 0;
  private long m_currentTimestamp = 0;
  Clock clock = Clock.systemDefaultZone();

  Accelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    leftArm =
        new ClimberArm(ClimbSolenoidLeftExtend, ClimbSolenoidLeftRetract, ClimbSolenoidLeftChoke);
    rightArm =
        new ClimberArm(
            ClimbSolenoidRightExtend, ClimbSolenoidRightRetract, ClimbSolenoidRightChoke);

    anglePistonsControl = new Solenoid(PneumaticsModuleType.CTREPCM, AngleSolenoid);
    setRetracting();

    m_sensor = new LIDAR07_100W_Module(I2C.Port.kMXP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_currentDistance = m_sensor.getDistanceMM();
    if (m_currentDistance == m_previousDistance) {
      // Nothing to do!
      return;
    }
    m_currentTimestamp = clock.millis();
    m_currentVelocity =
        (1000 * (m_currentDistance - m_previousDistance))
            / ((double) m_currentTimestamp - m_previousTimestamp);

    // SmartDashboard.putNumber("Distance diff (mm)", m_currentDistance - m_previousDistance);
    // SmartDashboard.putNumber("Timestamp diff (ms)", m_currentTimestamp - m_previousTimestamp);
    // SmartDashboard.putNumber("LIDAR distance (mm)", m_currentDistance);
    // SmartDashboard.putNumber("LIDAR velocity (mm/s)", m_currentVelocity);

    m_previousDistance = m_currentDistance;
    m_previousTimestamp = m_currentTimestamp;

    //   SmartDashboard.putString("Climber state", climberState.toString());

    //   SmartDashboard.putNumber("Angle X", m_accelerometer.getX());
    //   SmartDashboard.putNumber("Angle Y", m_accelerometer.getY());
    //   SmartDashboard.putNumber("Angle Z", m_accelerometer.getZ());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setExtending() {
    leftArm.extendArm();
    rightArm.extendArm();

    climberState = ClimberState.kExtending;
  }

  public void setRetracting() {
    leftArm.retractArm();
    rightArm.retractArm();

    climberState = ClimberState.kRetracting;
  }

  public void setCoasting() {
    leftArm.coastArm();
    rightArm.coastArm();

    climberState = ClimberState.kCoasting;
  }

  /** Set the solenoid to extend the "angle" pistons */
  public void verticalMainCylinders() {
    anglePistonsControl.set(false);
  }

  /** Set the solenoid to retract the "angle" pistons */
  public void angleMainCylinders() {
    anglePistonsControl.set(true);
  }

  public ClimberState getCurrentState() {
    return climberState;
  }

  public double getCurrentDistance() {
    return m_currentDistance;
  }

  public double getCurrentVelocity() {
    return m_currentVelocity;
  }
}
