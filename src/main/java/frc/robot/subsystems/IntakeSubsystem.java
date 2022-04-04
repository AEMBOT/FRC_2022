package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // Intake motors
  private final CANSparkMax m_intakeRoller = new CANSparkMax(kCANRollerID, MotorType.kBrushless);
  private final CANSparkMax m_intakeLift = new CANSparkMax(kCANLiftID, MotorType.kBrushless);

  // Lift encoder
  private final RelativeEncoder m_liftEncoder = m_intakeLift.getEncoder();

  // motor geared 125:1 -> 24:72 gearing
  public IntakeSubsystem() {
    // Restore motors to factory defaults for settings to be consistent
    m_intakeRoller.restoreFactoryDefaults();
    m_intakeLift.restoreFactoryDefaults();

    // Lift shouldn't drift, so set it to brake mode
    m_intakeLift.setIdleMode(CANSparkMax.IdleMode.kBrake);
    // Set max current the winch can draw
    m_intakeLift.setSmartCurrentLimit(kLiftMaxExpectedCurrent, 0, 20000);

    // Roller can be in coast mode, since preciseness isn't important
    m_intakeRoller.setIdleMode(CANSparkMax.IdleMode.kCoast);
  }

  // INTAKE ROLLER CONTROL

  /** Runs the intake roller inward to intake cargo. */
  public void runRollerInwards() {
    m_intakeRoller.set(-kRollerPower);
  }

  /** Runs the intake roller outward to eject cargo. */
  public void runRollerOutwards() {
    m_intakeRoller.set(kRollerPower);
  }

  /** Stops the intake roller. */
  public void stopRoller() {
    m_intakeRoller.set(0);
  }

  // INTAKE LIFT CONTROL

  /** Raises the intake lift at a constant speed. */
  public void raiseIntake() {
    m_intakeLift.set(kLiftRaisingPower);
  }

  /** Lowers the intake lift at a constant speed. */
  public void lowerIntake() {
    m_intakeLift.set(kLiftLoweringPower);
  }

  /** Stops moving the intake lift. */
  public void stopLift() {
    m_intakeLift.set(0);
  }

  // HOMING

  /** Returns true if the intake lift is drawing too much current. */
  public boolean isAtHardLimit() {
    return m_intakeLift.getOutputCurrent() > kLiftMaxExpectedCurrent;
  }

  /** Returns the current position of the intake lift motor, in rotations. */
  public double getLiftPosition() {
    return m_liftEncoder.getPosition();
  }

  /** Resets the encoder on the intake lift motor. */
  public void resetLiftEncoder() {
    m_liftEncoder.setPosition(0);
  }
}
