package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class StopIntakeRoller extends CommandBase {
  private IntakeSubsystem m_intake;

  public StopIntakeRoller(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    m_intake.stopRoller();
  }

  // Just stop it and that's it
  @Override
  public boolean isFinished() {
    // This command should only end when interrupted
    return true;
  }
}
