package frc.robot.hardware;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * A convenience class for controlling a climber arm, which consists of a double solenoid (for
 * controlling the actual pistion) and a regular solenoid (for cutting off pressure).
 */
public class ClimberArm {
  // the solenoid that controls the piston
  private DoubleSolenoid m_extensionSolenoid;

  // the solenoid that controls airflow to the control solenoid
  private Solenoid m_flowSolenoid;

  /**
   * Constructs a ClimberArm with the given ports on the PCM.
   *
   * @param extendPort The port for extending the arm piston
   * @param retractPort The port for retracting the arm piston
   * @param flowControlPort The port that the cutoff solenoid is attached to
   */
  public ClimberArm(int extendPort, int retractPort, int flowControlPort) {
    m_extensionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, extendPort, retractPort);
    m_flowSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, flowControlPort);
  }

  /** Extends the climber arm piston. */
  public void extendArm() {
    m_extensionSolenoid.set(Value.kForward);
    m_flowSolenoid.set(false);
  }

  /** Retracts the climber arm piston. */
  public void retractArm() {
    m_extensionSolenoid.set(Value.kReverse);
    m_flowSolenoid.set(false);
  }

  /** Cuts off pressure to the arm piston. */
  public void cutOffPressure() {
    m_flowSolenoid.set(true);
  }
}
