package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AlignWithHubSmart;
import frc.robot.commands.drive.DriveStraightSmart;
import frc.robot.commands.intake.RunIntakeLift;
import frc.robot.commands.shooter.RampShooter;
import frc.robot.commands.utilities.enums.LiftDirection;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {
  public TwoBallAuto(
      DriveSubsystem drive,
      ShooterSubsystem shooter,
      IndexerSubsystem indexer,
      IntakeSubsystem intake,
      Limelight limelight) {
    drive.setBrakeMode();
    addCommands(
        // TODO: This is untested
        new RunIntakeLift(intake, LiftDirection.Down).withTimeout(2),
        new DriveStraightSmart(Units.inchesToMeters(-55.5), drive),
        new WaitCommand(2),
        // lower & run intake/lower indexer

        // Align with the hub
        new AlignWithHubSmart(limelight, drive),

        // Run indexer into shooter
        new RampShooter(shooter).withTimeout(3),

        // Stop the indexer
        new InstantCommand(indexer::stopBelts, indexer));
  }
}
