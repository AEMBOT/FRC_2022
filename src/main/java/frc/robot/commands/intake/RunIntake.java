package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends CommandBase {
  private IntakeSubsystem m_intake;
  private boolean m_reverse;

  public RunIntake(IntakeSubsystem intake) {
    this(intake, false);
  }

  public RunIntake(IntakeSubsystem intake, boolean reverse) {
    m_intake = intake;
    m_reverse = reverse;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    m_intake.runRollerAtMaxPower(m_reverse);
  }

  @Override
  public void end(boolean _interrupted) {
    m_intake.setRPM(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
