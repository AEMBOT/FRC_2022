package frc.robot.commands.intake;

import static frc.robot.Constants.IntakeConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** A command that lowers the intake, running it in the process. */
public class LowerIntake extends CommandBase {
  private IntakeSubsystem m_intake;
  private final Timer m_rollerTimer = new Timer();

  /**
   * Constructs a LowerIntake command that lowers the intake.
   *
   * @param intake The robot's intake subsystem
   */
  public LowerIntake(IntakeSubsystem intake) {
    m_intake = intake;
    addRequirements(m_intake);
  }

  @Override
  public void execute() {
    // Lower the intake if it's not already all the way down
    if (m_intake.getLiftPosition() >= -kLiftRangeOfMotion) {
      m_intake.lowerIntake();
    }

    // Stop the intake lift and run the roller to make it fall
    else {
      m_intake.stopLift();
      m_intake.runRollerInwards();

      // The roller has to run for a bit to get the intake down
      m_rollerTimer.start();
    }
  }

  @Override
  public void end(boolean _interrupted) {
    // Stop the intake lift and roller if they are running
    m_intake.stopLift();
    m_intake.stopRoller();

    // Stop & reset the timer for running the roller
    m_rollerTimer.stop();
    m_rollerTimer.reset();
  }

  @Override
  public boolean isFinished() {
    // Finish after the roller runs for a tenth of a second
    return m_rollerTimer.get() > 0.1;
  }
}
