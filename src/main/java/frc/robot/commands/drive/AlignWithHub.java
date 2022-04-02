package frc.robot.commands.drive;

import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DriveSubsystem;

/** A command that aligns the robot (specifically the shooter) with the central hub. */
public class AlignWithHub extends TurnDegrees {
  private Limelight m_limelight;

  /**
   * Constructs an AlignWithHub command, which uses the Limelight to align the robot with the hub in
   * preparation for shooting.
   *
   * @param limelight The robot's Limelight
   * @param drive The robot's drive subsystem
   */
  public AlignWithHub(Limelight limelight, DriveSubsystem drive) {
    super(() -> -limelight.getX(), drive);

    m_limelight = limelight;
  }

  @Override
  public void initialize() {
    super.initialize();
    // m_limelight.setLEDMode(LEDMode.On);
  }

  // @Override
  // public void end(boolean _interrupted) {
  //   m_limelight.setLEDMode(LEDMode.Off);
  // }
}
