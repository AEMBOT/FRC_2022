package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.indexer.RunUpperIndexer;
import frc.robot.commands.intake.RunIntakeRoller;
import frc.robot.commands.utilities.Noop;
import frc.robot.commands.utilities.TimedRumble;
import frc.robot.commands.utilities.enums.CargoDirection;
import frc.robot.hardware.Limelight;
import frc.robot.hardware.Limelight.LEDMode;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Ramps up the shooter to the proper RPM using the Limelight, then runs the indexer & intake into
 * the shooter.
 */
public class RampThenShoot extends SequentialCommandGroup {
  private Limelight m_limelight;

  /**
   * Constructs a RampThenShoot command for use in autonomous (doesn't rumble the controllers).
   *
   * @param intake The robot's intake subsystem
   * @param indexer The robot's indexer subsystem
   * @param shooter The robot's shooter subsystem
   * @param limelight The robot's Limelight instance
   */
  public RampThenShoot(
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      Limelight limelight) {
    this(intake, indexer, shooter, limelight, null, null);
  }

  /**
   * Constructs a RampThenShoot command that rumbles the controllers if the Limelight doesn't hub
   * isn't visible.
   *
   * @param intake The robot's intake subsystem
   * @param indexer The robot's indexer subsystem
   * @param shooter The robot's shooter subsystem
   * @param limelight The robot's Limelight instance
   * @param driverController The primary controller
   * @param secondaryController The secondary controller
   */
  public RampThenShoot(
      IntakeSubsystem intake,
      IndexerSubsystem indexer,
      ShooterSubsystem shooter,
      Limelight limelight,
      XboxController driverController,
      XboxController secondaryController) {
    addCommands(
        // Ramp up the shooter to the desired power, rumbling the driver controller if there's no
        // detected target
        new ConditionalCommand(
            new Noop(),
            parallel(
                new TimedRumble(driverController, 0.25, 0.5),
                new TimedRumble(secondaryController, 0.25, 0.5)),
            () ->
                limelight.hasValidTarget()
                    || driverController == null
                    || secondaryController == null),

        // Ramp up the shooter then run the indexer once that's finished
        parallel(
            new RunShooterWithLimelight(shooter),
            sequence(
                new WaitUntilCommand(shooter::atTargetRPM).withTimeout(1),
                parallel(
                    new RunIntakeRoller(intake, CargoDirection.Intake),
                    new RunUpperIndexer(indexer, CargoDirection.Intake)))));

    m_limelight = limelight;
  }

  @Override
  public void initialize() {
    super.initialize();
    m_limelight.setLEDMode(LEDMode.On);
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);

    // Turn off the limelight LED after finishing
    // m_limelight.setLEDMode(LEDMode.Off);
  }
}
