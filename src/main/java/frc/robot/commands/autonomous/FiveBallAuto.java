package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** A theoreticall five ball autonomous, but not everything has been implemented. DO NOT RUN YET. */
public class FiveBallAuto extends SequentialCommandGroup {
  /**
   * Construcsts a FiveBallAuto command, which is theoretically capable of making 5 cargo into the
   * hub. NOT FUNCTIONAL YET.
   *
   * @param drive The robot's drive subsystem
   * @param shooter The robot's shooter subsystem
   * @param indexer The robot's indexer subsystem
   * @param intake The robot's intake subsystem
   * @param limelight The robot's {@link Limelight} instance
   */
  public FiveBallAuto(
      DrivetrainSubsystem drive,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      Limelight limelight) {
    addCommands(
        // Starting position is determined by jig - see discord for specifics
        // 1
        // back up into ball nearest hub
        drive.driveMetersCommand(Units.inchesToMeters(-55.5)),
        new WaitCommand(2),

        // --intake here
        // --shoot here

        // 2
        // drive forwards to line up shot
        drive.driveMetersCommand(Units.inchesToMeters(84)),
        new WaitCommand(2),

        // turn 1
        // rotate to align with 2 far balls
        drive.turnDegreesCommand(-75),
        new WaitCommand(2),

        // 3
        // drive back to first ball
        drive.driveMetersCommand(Units.inchesToMeters(-87)),
        new WaitCommand(2),

        // --intake here

        // 4
        // drive back to second ball
        drive.driveMetersCommand(Units.inchesToMeters(-157)),
        new WaitCommand(2),

        // --intake here

        // 5
        // return to hub
        drive.driveMetersCommand(Units.inchesToMeters(244)),
        new WaitCommand(2),

        // turn 2
        // line up shot, shoot
        drive.turnDegreesCommand(-285),
        new WaitCommand(2),
        drive.driveMetersCommand(-1),
        drive.alignWithHubCommand(limelight)
        // --shoot here

        ); // end command list
  }
}
