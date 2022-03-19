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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.Limelight;
import frc.robot.utilities.ShooterMath;

public class ShooterSubsystem extends SubsystemBase {

  // Flywheel motors & related devices
  // TODO: Figure out which one is left vs. right and name accordingly
  private static CANSparkMax m_leftFlywheelMotor =
      new CANSparkMax(kLeftMotorCANId, MotorType.kBrushless);
  private static CANSparkMax m_rightFlywheelMotor =
      new CANSparkMax(kRightMotorCANId, MotorType.kBrushless);

  private RelativeEncoder m_flywheelEncoder = m_leftFlywheelMotor.getEncoder();
  private SparkMaxPIDController m_mainPIDController = m_leftFlywheelMotor.getPIDController();

  // Limelight for vision processing
  private Limelight m_limelight;

  private double targetPower = 0;

  /** Creates a new ArcShooter. */
  public ShooterSubsystem(Limelight limelight) {
    // Restore motor factory defaults without persisting
    m_leftFlywheelMotor.restoreFactoryDefaults();
    m_rightFlywheelMotor.restoreFactoryDefaults();

    // Set smart motion constants for main flywheel
    // TODO: Should this be done for both flywheels?
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
    // FIXME: This might not play well with teleop control of the shooter
    // targetPower = SmartDashboard.getNumber("Shooter Target Power", targetPower);
    // runAtRPM(targetPower);

    updateDashboard();
  }

  private void updateDashboard() {
    // The current flywheel speed
    SmartDashboard.putNumber("Shooter Current Power", getFlywheelRPM());
    SmartDashboard.putNumber("Shooter Target Power", targetPower);
  }

  private void setSmartMotionConstants(SparkMaxPIDController controller) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
    controller.setFF(kFF);
    controller.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    controller.setSmartMotionMaxVelocity(kMaxVel, smartMotionSlot);
    controller.setSmartMotionMinOutputVelocity(kMinVel, smartMotionSlot);
    controller.setSmartMotionMaxAccel(kMaxAcc, smartMotionSlot);
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
  // TODO: Find a better name for this method
  public void runAtCalibratedRPM() {
    double rpm = getTargetRPM();
    runAtRPM(rpm);
  }

  /** Runs the shooter flywheels at the given RPM. */
  public void runAtRPM(double rpm) {
    m_mainPIDController.setReference(rpm, ControlType.kVelocity);
  }

  /** Stops the shooter flywheels. */
  public void stopShooter() {
    m_leftFlywheelMotor.set(0);
  }

  /** Returns true if the flywheel is within some tolerance of the target RPM. */
  public boolean atTargetRPM() {
    return m_flywheelEncoder.getVelocity() >= getTargetRPM() - kRPMTolerance;
  }
}
