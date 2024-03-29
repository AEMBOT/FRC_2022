package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;

/** A command that aligns the robot (specifically the shooter) with the central hub. */
public class AlignWithHub extends SequentialCommandGroup {
  private Limelight m_limelight;
  private double m_goalAngle;

  /**
   * Constructs an AlignWithHub command, which uses the Limelight to align the robot with the hub in
   * preparation for shooting.
   *
   * @param limelight The robot's Limelight
   * @param drive The robot's drive subsystem
   */
  public AlignWithHub(Limelight limelight, DrivetrainSubsystem drive) {
    m_limelight = limelight;
    addCommands(
            // Only fetch the hub angle when the command is first scheduled
            new InstantCommand(() -> m_goalAngle = -m_limelight.getX()),
            new TurnDegrees(() -> this.m_goalAngle, drive)
    );
  }
}
