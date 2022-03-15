package frc.robot.commands.utilities;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.LEDMode;

public class TurnOnLimelightLEDs extends CommandBase {
  private Limelight m_limelight;

  public TurnOnLimelightLEDs(Limelight limelight) {
    m_limelight = limelight;
  }

  @Override
  public void execute() {
    m_limelight.setLEDMode(LEDMode.On);
  }
}
