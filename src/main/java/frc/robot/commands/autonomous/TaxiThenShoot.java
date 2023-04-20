package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AlignWithHub;
import frc.robot.commands.intake.LiftIntake;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.shooter.RampThenShoot;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/** An autonomous command that taxis, intakes a cargo, and shoots 2 cargo. */
public class TaxiThenShoot extends SequentialCommandGroup {
  private IntakeSubsystem m_intake;
  private DrivetrainSubsystem m_drive;

  /**
   * Constructs a TaxiThenShoot command, which is essentially a 2 ball auto.
   *
   * @param drive The robot's drive subsystem
   * @param intake The robot's intake subsystem
   * @param indexer The robot's indexer subsystem
   * @param shooter The robot's shooter subsystem
   * @param limelight The robot's Limelight instance
   */
  public TaxiThenShoot(
      DrivetrainSubsystem drive,
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      Limelight limelight) {
    addCommands(
        // Home the intake at the beginning of auto
        new LiftIntake(intake),

        // Lower and turn on the intake
        new LowerIntake(intake),
        new InstantCommand(intake::runRollerInwards, intake),

        // Run the intake while driving away from the hub
        // TODO: Tune this distance
        drive.driveMetersCommand(Units.feetToMeters(-6)).withTimeout(4),

        // Stop running the intake
        new InstantCommand(intake::stopRoller, intake),

        // Drive forward a bit to make alignment easier if we hit the wall
        drive.driveMetersCommand(Units.feetToMeters(1)),
        new WaitCommand(1),

        // Align with the hub using the limelight
        new AlignWithHub(limelight, drive).withTimeout(1),
        new WaitCommand(0.2),
        new AlignWithHub(limelight, drive).withTimeout(1),

        // A driver controller has to be passed in order for this command to work (it includes
        // rumble)
        new RampThenShoot(intake, indexer, shooter, limelight).withTimeout(5));
    finallyDo(
            (interrupted) -> {
              // Stop all mechanisms
              m_intake.stopRoller();
              m_intake.stopLift();
              m_drive.stopMotors();
            }
    );

    m_intake = intake;
    m_drive = drive;
  }
}
