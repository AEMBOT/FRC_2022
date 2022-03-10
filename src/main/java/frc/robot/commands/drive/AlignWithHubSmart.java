package frc.robot.commands.drive;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimeLightTargeting;

public class AlignWithHubSmart extends SequentialCommandGroup {
    public AlignWithHubSmart(LimeLightTargeting limelight, DriveSubsystem drive) {
        addCommands(
            new InstantCommand(limelight::turnOnLED),
            new WaitCommand(0.2),
            new TurnToAngleSmart(limelight::getX, drive),
            new InstantCommand(limelight::turnOffLED)
        );
    }
}
