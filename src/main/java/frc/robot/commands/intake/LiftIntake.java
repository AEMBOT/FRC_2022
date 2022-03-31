package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** Fully lifts the intake and resets the lift encoders if not interrupted. */
public class LiftIntake extends CommandBase {
  private IntakeSubsystem m_intake;

  public LiftIntake(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    m_intake.raiseIntake();
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopLift();

    // Only re-home the intake lift (reset the encoder) if this wasn't interrupted
    if (!interrupted) {
      m_intake.resetLiftEncoder();
    }
  }

  @Override
  public boolean isFinished() {
    // Stop running the intake when it starts drawing too much current (i.e. when it runs into the
    // bot)
    return m_intake.isAtHardLimit();
  }
}
