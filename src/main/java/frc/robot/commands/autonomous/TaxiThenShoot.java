package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AlignWithHubSmart;
import frc.robot.commands.drive.DriveStraightSmart;
import frc.robot.commands.shooter.RampThenShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimeLightTargeting;
import frc.robot.subsystems.ShooterSubsystem;

public class TaxiThenShoot extends SequentialCommandGroup {
    public TaxiThenShoot(DriveSubsystem drive, IndexerSubsystem indexer, ShooterSubsystem shooter, LimeLightTargeting limelight, XboxController driverController) {
        addCommands(
            // TODO: Measure the length of the robot
            new DriveStraightSmart(Units.feetToMeters(4), drive),
            new AlignWithHubSmart(limelight, drive),

            // A driver controller has to be passed in order for this command to work (it includes rumble)
            new RampThenShoot(indexer, shooter, limelight, driverController).withTimeout(1)
        );
    }
}
