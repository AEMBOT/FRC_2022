package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.RunUpperIndexer;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RampThenShoot extends SequentialCommandGroup {
    public RampThenShoot(IndexerSubsystem indexer, ShooterSubsystem shooter) {
        addCommands(
            new RampShooter(shooter),
            new RunUpperIndexer(indexer)
        );
    }
}
