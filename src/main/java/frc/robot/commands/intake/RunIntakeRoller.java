package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utilities.enums.CargoDirection;
import frc.robot.subsystems.IntakeSubsystem;

/** Runs the intake roller indefinitely, then stops it. (Meant for use in teleop) */
public class RunIntakeRoller extends CommandBase {
  private IntakeSubsystem m_intake;
  private Runnable m_runIntake;

  /**
   * Constructs a RunInakeRoller command to run the intake roller during teleop.
   *
   * @param intake The robot's intake subsystem
   * @param direction The direction to move cargo
   */
  public RunIntakeRoller(IntakeSubsystem intake, CargoDirection direction) {
    m_intake = intake;
    addRequirements(m_intake);

    // Capture a different method depending on the cargo direction
    if (direction == CargoDirection.Intake) {
      m_runIntake = m_intake::runRollerInwards;
    } else if (direction == CargoDirection.Eject) {
      m_runIntake = m_intake::runRollerOutwards;
    }
  }

  @Override
  public void execute() {
    m_runIntake.run();
  }

  @Override
  public void end(boolean _interrupted) {
    m_intake.stopRoller();
  }
}
