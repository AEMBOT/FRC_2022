package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** Homes the intake to its "max" point. */
public class IntakeControl extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_subsystem;

  private boolean shouldInvert = false;

  /**
   * Creates a new HomeIntakeCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeControl(IntakeSubsystem subsystem, boolean invert) {
    m_subsystem = subsystem;
    shouldInvert = invert;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    int direction = (shouldInvert ? -1 : 1);
    double power = 0.3 * direction;
    m_subsystem.setLiftPower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_subsystem.setLiftPower(0);
  }
}
