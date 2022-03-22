package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // Intake motors
  private final CANSparkMax m_intakeRoller = new CANSparkMax(kCANRollerID, MotorType.kBrushless);
  private final CANSparkMax m_intakeWinch = new CANSparkMax(kCANWinchID, MotorType.kBrushless);
  private final CANSparkMax m_innerRoller =
      new CANSparkMax(kCANInnerRollerID, MotorType.kBrushless);

  // Lowest indexer belt
  private final CANSparkMax m_lowerIndexBelt =
      new CANSparkMax(kIndexerLowerBottomBeltPort, MotorType.kBrushless);

  // Winch encoder
  private final RelativeEncoder m_winchEncoder = m_intakeWinch.getEncoder();

  // whether the subsystem is successfully homed to its max point
  private boolean m_homingComplete = false;

  // motor geared 125:1 -> 24:72 gearing
  public IntakeSubsystem() {
    // Restore motors to factory defaults for settings to be consistent
    m_intakeRoller.restoreFactoryDefaults();
    m_intakeWinch.restoreFactoryDefaults();
    m_innerRoller.restoreFactoryDefaults();

    // Lower indexer belt & inner roller follow the intake roller
    m_lowerIndexBelt.follow(m_intakeRoller);
    m_innerRoller.follow(m_intakeRoller);

    // Winch shouldn't drift, so set it to brake mode
    m_intakeWinch.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // Set max current the winch can draw
    m_intakeWinch.setSmartCurrentLimit(kWinchMaxExpectedCurrent, 0, 20000);

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
    m_intakeRoller.set(kRollerPower);
  }

  /** Runs the intake roller outward to eject cargo. */
  public void runRollerOutwards() {
    m_intakeRoller.set(-kRollerPower);
  }

  /** Stops the intake roller. */
  public void stopRoller() {
    m_intakeRoller.set(0);
  }

  // INTAKE LIFT CONTROL

  /** Raises the intake lift at a constant speed. */
  public void raiseIntake() {
    m_intakeWinch.set(kWinchRaisingPower);
  }

  /** Lowers the intake lift at a constant speed. */
  public void lowerIntake() {
    m_intakeWinch.set(kWinchLoweringPower);
  }

  /** Stops moving the intake lift. */
  public void stopWinch() {
    m_intakeWinch.set(0);
  }

  // HOMING (not quite functional)

  // TODO: This may not work with the intake winch design
  /** Returns true if the intake lift is drawing too much current. */
  public boolean isAtHardLimit() {
    return m_intakeWinch.getOutputCurrent() > kWinchMaxExpectedCurrent;
  }

  /** sets the range of motion for the intake lift */
  public void setHome() {
    m_homingComplete = true;
    m_intakeWinch.getEncoder().setPosition(0);
  }

  public boolean getHomingComplete() {
    return m_homingComplete;
  }

  public double getWinchPosition() {
    return m_winchEncoder.getPosition();
  }
}
