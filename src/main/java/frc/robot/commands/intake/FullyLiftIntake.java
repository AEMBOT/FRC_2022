package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class FullyLiftIntake extends CommandBase {
  private IntakeSubsystem m_intake;

  public FullyLiftIntake(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    m_intake.raiseIntake();
  }

  @Override
  public void end(boolean _interrupted) {
    m_intake.stopWinch();
    m_intake.resetLiftEncoder();
  }

  @Override
  public boolean isFinished() {
    // return false;
    return m_intake.isAtHardLimit();
  }
}
