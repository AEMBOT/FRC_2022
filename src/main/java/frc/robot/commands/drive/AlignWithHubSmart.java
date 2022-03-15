package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.utilities.TurnOnLimelightLEDs;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DriveSubsystem;

public class AlignWithHubSmart extends SequentialCommandGroup {
  public AlignWithHubSmart(Limelight limelight, DriveSubsystem drive) {
    addCommands(
        new TurnOnLimelightLEDs(limelight),
        new WaitCommand(0.2),
        new TurnToAngleSmart(() -> -limelight.getX(), drive).withTimeout(0.5)
        // new TurnOffLimelight(limelight)
        );
  }
}
