package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  // Lift motors
  private final CANSparkMax m_liftLeft = new CANSparkMax(kLiftLeftPort, MotorType.kBrushless);
  private final CANSparkMax m_liftRight = new CANSparkMax(kLiftRightPort, MotorType.kBrushless);

  // Intake motors
  private final CANSparkMax m_intakeRoller = new CANSparkMax(kRollerPort, MotorType.kBrushless);
  private final CANSparkMax m_lowerIndexBelt =
      new CANSparkMax(kIndexerLowerBottomBeltPort, MotorType.kBrushless);

  // Lift encoder
  private final RelativeEncoder m_liftLeftEncoder = m_liftLeft.getEncoder();

  // whether the subsystem is successfully homed to its max point
  private boolean m_homingComplete = false;

  // The encoder positions for the lowest and highest allowed
  private double m_lowestAllowedPosition;
  private double m_highestAllowedPosition;

  // motor geared 125:1 -> 24:72 gearing
  public IntakeSubsystem() {
    // Lower indexer belt follows the intake roller
    m_lowerIndexBelt.follow(m_intakeRoller);

    // Lift motors should be in brake mode so the lift doesn't drift
    m_liftLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_liftRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Lift motors should follow each other, albeit mirrored
    m_liftLeft.follow(m_liftRight, true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lift Current", m_liftLeft.getOutputCurrent());
    SmartDashboard.putNumber("Lift Position", m_liftLeftEncoder.getPosition());
  }

  // INTAKE ROLLER CONTROL

  /** Runs the intake roller inward to intake cargo. */
  public void runRollerInwards() {
    m_intakeRoller.set(0.75);
  }

  /** Runs the intake roller outward to eject cargo. */
  public void runRollerOutwards() {
    m_intakeRoller.set(-0.75);
  }

  /** Stops the intake roller. */
  public void stopRoller() {
    m_intakeRoller.set(0);
  }

  // INTAKE LIFT CONTROL

  // TODO: raise/lower might be reversed
  /** Raises the intake lift at a constant speed. */
  public void raiseIntake() {
    m_liftLeft.set(0.5);
  }

  /** Lowers the intake lift at a constant speed. */
  public void lowerIntake() {
    m_liftLeft.set(-0.5);
  }

  /** Stops moving the intake lift. */
  public void stopLift() {
    m_liftLeft.set(0);
  }

  // HOMING (not quite functional)

  /** Returns true if the intake lift is drawing too much current. */
  public boolean isAtHardLimit() {
    return (m_liftLeft.getOutputCurrent() > Constants.IntakeConstants.kMaxExpectedCurrent
        || m_liftRight.getOutputCurrent() > Constants.IntakeConstants.kMaxExpectedCurrent);
  }

  /** sets the range of motion for the intake lift (UNTESTED). */
  public void setHome(double min, double max) {
    if (m_homingComplete) {
      System.out.println("Intake being re-homed!");
    }

    // Set the soft limits for the lift motors
    // setLiftSoftLimits((float) max, (float) min);

    // Enable forward and reverse soft limits for the lift motors
    // enableLiftSoftLimits();

    m_lowestAllowedPosition = min;
    m_highestAllowedPosition = max;

    m_homingComplete = true;
  }

  /** Enable soft limits for the lift motors. */
  private void enableLiftSoftLimits() {
    m_liftLeft.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_liftRight.enableSoftLimit(SoftLimitDirection.kForward, true);

    m_liftLeft.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_liftRight.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  /** Disable soft limits for the lift motors. */
  private void disableLiftSoftLimits() {
    m_liftLeft.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_liftRight.enableSoftLimit(SoftLimitDirection.kForward, false);

    m_liftLeft.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_liftRight.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  /** Sets soft limits for the lift motors. */
  public void setLiftSoftLimits(float max, float min) {
    m_liftLeft.setSoftLimit(SoftLimitDirection.kForward, max);
    m_liftRight.setSoftLimit(SoftLimitDirection.kForward, max);

    m_liftLeft.setSoftLimit(SoftLimitDirection.kReverse, min);
    m_liftRight.setSoftLimit(SoftLimitDirection.kReverse, min);
  }

  /** gets the encoder position of the left lift */
  public double getLiftPosition() {
    return m_liftRight.getEncoder().getPosition();
  }
}
