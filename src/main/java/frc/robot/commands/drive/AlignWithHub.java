package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithHub extends SequentialCommandGroup {
  private Limelight m_limelight;
  private double m_goalAngle;

  public AlignWithHub(Limelight limelight, DriveSubsystem drive) {
    m_limelight = limelight;
    addCommands(new TurnDegrees(() -> this.m_goalAngle, drive));
  }

  @Override
  public void initialize() {
    super.initialize();
    m_goalAngle = -m_limelight.getX();
  }

  // @Override
  // public void end(boolean _interrupted) {
  //   m_limelight.setLEDMode(LEDMode.Off);
  // }
}
