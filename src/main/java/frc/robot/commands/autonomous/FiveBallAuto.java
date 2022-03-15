package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AlignWithHubSmart;
import frc.robot.commands.drive.DriveStraightSmart;
import frc.robot.commands.drive.TurnToAngleSmart;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// command list for autopathing
public class FiveBallAuto extends SequentialCommandGroup {
  public FiveBallAuto(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      Limelight limelight) {
    drive.setBrakeMode();
    addCommands(
        // Starting position is determined by jig - see discord for specifics
        // 1
        // back up into ball nearest hub
        new DriveStraightSmart(Units.inchesToMeters(-55.5), drive),
        new WaitCommand(2),

        // --intake here
        // --shoot here

        // 2
        // drive forwards to line up shot
        new DriveStraightSmart(Units.inchesToMeters(84), drive),
        new WaitCommand(2),

        // turn 1
        // rotate to align with 2 far balls
        new TurnToAngleSmart(-75, drive),
        new WaitCommand(2),

        // 3
        // drive back to first ball
        new DriveStraightSmart(Units.inchesToMeters(-87), drive),
        new WaitCommand(2),

        // --intake here

        // 4
        // drive back to second ball
        new DriveStraightSmart(Units.inchesToMeters(-157), drive),
        new WaitCommand(2),

        // --intake here

        // 5
        // return to hub
        new DriveStraightSmart(Units.inchesToMeters(244), drive),
        new WaitCommand(2),

        // turn 2
        // line up shot, shoot
        new TurnToAngleSmart(-285, drive),
        new WaitCommand(2),
        new DriveStraightSmart(-1, drive),
        new AlignWithHubSmart(limelight, drive)
        // --shoot here

        ); // end command list
  }
}
