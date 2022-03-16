package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AlignWithHubSmart;
import frc.robot.commands.drive.DriveStraightSmart;
import frc.robot.commands.shooter.RampThenShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimeLightTargeting;
import frc.robot.subsystems.ShooterSubsystem;

public class TaxiThenShoot extends SequentialCommandGroup {
    public TaxiThenShoot(DriveSubsystem drive, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, LimeLightTargeting limelight) {
        addCommands(
            // Turn on the intake
            new InstantCommand(() -> intake.runRollerAtMaxPower(false), intake),

            // Run the intake while driving away from the hub
            // TODO: Tune this distance
            new DriveStraightSmart(Units.feetToMeters(-6), drive),

            // Align with the hub using the limelight
            new AlignWithHubSmart(limelight, drive).withTimeout(1),

            // A driver controller has to be passed in order for this command to work (it includes rumble)
            new RampThenShoot(indexer, shooter, limelight).withTimeout(5),

            // Stop running the intake
            new InstantCommand(() -> intake.setRPM(0), intake)
        );
    }
}
