// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ShooterConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.utilities.ShooterMath;

/**
 * A subsystem that's responsible for control of the shooter, either manually or using the
 * Limelight.
 */
public class ShooterSubsystem extends SubsystemBase {
  // Flywheel motors
  private static CANSparkMax m_leftFlywheelMotor =
      new CANSparkMax(kLeftMotorCANId, MotorType.kBrushless);
  private static CANSparkMax m_rightFlywheelMotor =
      new CANSparkMax(kRightMotorCANId, MotorType.kBrushless);

  // The encoder & PID controller only have to be fetched from one motor, since the other follows it
  private RelativeEncoder m_flywheelEncoder = m_leftFlywheelMotor.getEncoder();
  private SparkMaxPIDController m_mainPIDController = m_leftFlywheelMotor.getPIDController();

  // Limelight for vision processing
  private Limelight m_limelight;

  // An RPM offset that can be changed on the fly in case the shooter is consistently
  // over/undershooting
  private double m_rpmOffset = 0;

  /**
   * Creates a new ShooterSubsystem, configuring the flywheel motors & storing a reference to the
   * Limelight.
   *
   * @param limelight The robot's {@link Limelight} instance
   */
  public ShooterSubsystem(Limelight limelight) {
    // Restore motor factory defaults for consistent settings
    m_leftFlywheelMotor.restoreFactoryDefaults();
    m_rightFlywheelMotor.restoreFactoryDefaults();

    // Set smart motion constants for main flywheel
    setSmartMotionConstants(m_mainPIDController);

    // Have the secondary shooter motor be inverted relative to the primary
    m_rightFlywheelMotor.follow(m_leftFlywheelMotor, true);

    // Flywheel motors should be in coast mode so as to not destroy themselves
    m_leftFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    m_rightFlywheelMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    m_limelight = limelight;
  }

  @Override
  public void periodic() {
    updateDashboard();
  }

  /** Updates the dashboard with values of interest. */
  private void updateDashboard() {
    // SmartDashboard.putNumber("Shooter Velocity", getFlywheelRPM());
  }

  /**
   * Sets the PIDF constants for the given {@link SparkMaxPIDController}.
   *
   * @param controller The {@link SparkMaxPIDController} controller to configure
   */
  private void setSmartMotionConstants(SparkMaxPIDController controller) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setIZone(kIZone);
    controller.setD(kD);
    controller.setFF(kFF);
    controller.setOutputRange(kMinOutput, kMaxOutput);
  }

  /** Get the current flywheel RPM. */
  public double getFlywheelRPM() {
    return m_flywheelEncoder.getVelocity();
  }

  /** Gets the target RPM for the shooter based on what the Limelight sees. */
  public double getTargetRPM() {
    double yAngle;

    if (!m_limelight.hasValidTarget()) {
      yAngle = kDefaultYAngle;
    } else {
      yAngle = m_limelight.getY();
    }

    return ShooterMath.calculateRPM(yAngle);
  }

  /** Runs the shooter at a given RPM using the Limelight */
  public void runAtCalibratedRPM() {
    double rpm = getTargetRPM();
    runAtRPM(rpm);
  }

  /**
   * Runs the shooter flywheels at the given RPM.
   *
   * @param rpm The speed in RPM to run the shooter flywheels at
   */
  public void runAtRPM(double rpm) {
    m_mainPIDController.setReference(rpm + m_rpmOffset, ControlType.kVelocity);
  }

  /** Stops the shooter flywheels. */
  public void stopShooter() {
    m_leftFlywheelMotor.set(0);
  }

  /** Returns true if the flywheel is within some tolerance of the target RPM. */
  public boolean atTargetRPM() {
    return m_flywheelEncoder.getVelocity() >= getTargetRPM() - kRPMTolerance;
  }

  // MANUAL RPM COMPENSATION

  /** Increments the manual RPM offset by 50rpm. */
  public void incrementRPMOffset() {
    m_rpmOffset += 50;
  }

  /** Decrements the manual RPM offset by 50rpm. */
  public void decrementRPMOffset() {
    m_rpmOffset -= 50;
  }
}
