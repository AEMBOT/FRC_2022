package frc.robot.commands.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class LowerIntake extends CommandBase {
  private IntakeSubsystem m_intake;
  private final Timer m_rollerTimer = new Timer();

  public LowerIntake(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    if (m_intake.getWinchPosition() >= -kLiftRangeOfMotion) {
      m_intake.lowerIntake();
    } else {
      m_intake.stopWinch();
      m_intake.runRollerInwards();
      m_rollerTimer.start();
    }
  }

  @Override
  public void end(boolean _interrupted) {
    m_intake.stopWinch();
    m_intake.stopRoller();
    m_rollerTimer.stop();
    m_rollerTimer.reset();
  }

  @Override
  public boolean isFinished() {
    return m_rollerTimer.get() > 0.1;
  }
}
