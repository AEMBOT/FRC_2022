package frc.robot.hardware;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class ClimberArm {

  // the solenoid that controls the piston
  private Solenoid controlSolenoid;

  // the solenoid that controls airflow to the control solenoid
  private Solenoid flowSolenoid;

  public ClimberArm(int port, int flowControlPort) {
    controlSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, port);
    flowSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, flowControlPort);
  }

  public void extendArm() {
    controlSolenoid.set(true);
    flowSolenoid.set(false);
  }

  public void retractArm() {
    controlSolenoid.set(false);
    flowSolenoid.set(false);
  }

  public void coastArm() {
    flowSolenoid.set(true);
  }
}
