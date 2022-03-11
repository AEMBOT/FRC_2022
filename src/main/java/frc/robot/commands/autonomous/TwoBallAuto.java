package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.HomeIntakeCommand;
import frc.robot.commands.RunShooterForTime;
import frc.robot.commands.drive.AlignWithHubSmart;
import frc.robot.commands.drive.DriveStraightProfiled;
import frc.robot.commands.drive.DriveStraightSmart;
import frc.robot.commands.drive.HomeOnHub;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightTargeting;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
    public TwoBallAuto(DriveSubsystem drive, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake, LimeLightTargeting limelight) {
        drive.setBrakeMode();
        addCommands(
            new DriveStraightSmart(Units.inchesToMeters(-55.5), drive),
            new WaitCommand(2),
            // lower & run intake/lower indexer

            // Align with the hub
            new AlignWithHubSmart(limelight, drive),

            // Run indexer into shooter
            new ParallelCommandGroup(
                new InstantCommand(() -> indexer.toggleExitSide(), indexer),
                new RunShooterForTime(3, shooter)
            ),

            // Stop the indexer
            new InstantCommand(() -> indexer.toggleExitSide(), indexer)
        );
    }
}
