package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AlignWithHub;
import frc.robot.commands.drive.DriveStraightSmart;
import frc.robot.commands.intake.LiftIntake;
import frc.robot.commands.intake.LowerIntake;
import frc.robot.commands.intake.StartIntakeRoller;
import frc.robot.commands.intake.StopIntakeRoller;
import frc.robot.commands.shooter.RampThenShoot;
import frc.robot.commands.utilities.enums.CargoDirection;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TaxiThenShoot extends SequentialCommandGroup {
  private IntakeSubsystem m_intake;
  private DriveSubsystem m_drive;

  public TaxiThenShoot(
      DriveSubsystem drive,
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      Limelight limelight) {

    m_intake = intake;
    m_drive = drive;
    addCommands(
        // Home the intake at the beginning of auto
        new LiftIntake(intake),

        // Lower and turn on the intake
        new LowerIntake(intake),
        new StartIntakeRoller(intake, CargoDirection.Intake),

        // Run the intake while driving away from the hub
        // TODO: Tune this distance
        new DriveStraightSmart(Units.feetToMeters(-6), drive).withTimeout(4),

        // Stop running the intake
        new StopIntakeRoller(intake),

        // Drive forward a bit to make alignment easier if we hit the wall
        new DriveStraightSmart(Units.feetToMeters(1), drive),
        new WaitCommand(1),

        // Align with the hub using the limelight
        new AlignWithHub(limelight, drive).withTimeout(1),
        new WaitCommand(0.2),
        new AlignWithHub(limelight, drive).withTimeout(1),

        // A driver controller has to be passed in order for this command to work (it includes
        // rumble)
        new RampThenShoot(intake, indexer, shooter, limelight).withTimeout(5));
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    m_intake.stopRoller();
    m_drive.stopMotors();
  }
}
