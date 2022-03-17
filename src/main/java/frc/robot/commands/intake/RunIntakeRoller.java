package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utilities.enums.CargoDirection;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntakeRoller extends CommandBase {
  private IntakeSubsystem m_intake;
  private CargoDirection m_direction;

  public RunIntakeRoller(IntakeSubsystem intake, CargoDirection direction) {
    m_intake = intake;
    m_direction = direction;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    if (m_direction == CargoDirection.Intake) {
      m_intake.runRollerInwards();
    } else if (m_direction == CargoDirection.Eject) {
      m_intake.runRollerOutwards();
    }
  }

  @Override
  public void end(boolean _interrupted) {
    m_intake.stopRoller();
  }

  @Override
  public boolean isFinished() {
    // This command should only end when interrupted
    return false;
  }
}
