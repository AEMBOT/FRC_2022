package frc.robot.hardware;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberArm {

  // the solenoid that controls the piston
  private DoubleSolenoid controlSolenoid;

  // the solenoid that controls airflow to the control solenoid
  private Solenoid flowSolenoid;

  public ClimberArm(int extendPort, int retractPort, int flowControlPort) {
    controlSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, extendPort, retractPort);
    flowSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, flowControlPort);
  }

  public void extendArm() {
    controlSolenoid.set(Value.kForward);
    flowSolenoid.set(false);
  }

  public void retractArm() {
    controlSolenoid.set(Value.kReverse);
    flowSolenoid.set(false);
  }

  public void coastArm() {
    flowSolenoid.set(true);
  }
}
