package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // Intake motors
  private final CANSparkMax m_intakeRoller = new CANSparkMax(kCANRollerID, MotorType.kBrushless);
  private final CANSparkMax m_intakeWinch = new CANSparkMax(kCANWinchID, MotorType.kBrushless);

  // Lowest indexer belt
  private final CANSparkMax m_lowerIndexBelt =
      new CANSparkMax(kIndexerLowerBottomBeltPort, MotorType.kBrushless);

  // Winch encoder
  private final RelativeEncoder m_winchEncoder = m_intakeWinch.getEncoder();

  // whether the subsystem is successfully homed to its max point
  private boolean m_homingComplete = false;

  // The encoder positions for the lowest and highest allowed
  private double m_lowestAllowedPosition;
  private double m_highestAllowedPosition;

  // motor geared 125:1 -> 24:72 gearing
  public IntakeSubsystem() {
    // Restore motors to factory defaults for settings to be consistent
    m_intakeRoller.restoreFactoryDefaults();
    m_intakeWinch.restoreFactoryDefaults();

    // Lower indexer belt follows the intake roller
    m_lowerIndexBelt.follow(m_intakeRoller);

    // Winch shouldn't drift, so set it to brake mode
    m_intakeWinch.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Roller can be in coast mode, since preciseness isn't important (I think)
    m_intakeRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Winch Current", m_intakeWinch.getOutputCurrent());
    SmartDashboard.putNumber("Winch Position", getWinchPosition());
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

  // FIXME: raise/lower might be reversed
  /** Raises the intake lift at a constant speed. */
  public void raiseIntake() {
    m_intakeWinch.set(0.5);
  }

  /** Lowers the intake lift at a constant speed. */
  public void lowerIntake() {
    m_intakeWinch.set(-0.5);
  }

  /** Stops moving the intake lift. */
  public void stopWinch() {
    m_intakeWinch.set(0);
  }

  // HOMING (not quite functional)

  // TODO: This may not work with the intake winch design
  /** Returns true if the intake lift is drawing too much current. */
  public boolean isAtHardLimit() {
    return m_intakeWinch.getOutputCurrent() > kMaxExpectedCurrent;
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
  private void enableWinchSoftLimits() {
    m_intakeWinch.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_intakeWinch.enableSoftLimit(SoftLimitDirection.kReverse, true);
  }

  /** Disable soft limits for the lift motors. */
  private void disableLiftSoftLimits() {
    m_intakeWinch.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_intakeWinch.enableSoftLimit(SoftLimitDirection.kReverse, false);
  }

  /** Sets soft limits for the lift motors. */
  public void setLiftSoftLimits(float max, float min) {
    m_intakeWinch.setSoftLimit(SoftLimitDirection.kForward, max);
    m_intakeWinch.setSoftLimit(SoftLimitDirection.kReverse, min);
  }

  /** gets the encoder position of the left lift */
  public double getWinchPosition() {
    return m_winchEncoder.getPosition();
  }
}
