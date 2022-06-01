package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** A command that intakes cargo, moving it into the lower section of the indexer. */
public class IntakeCargo extends CommandBase {
  private IndexerSubsystem m_indexer;
  private IntakeSubsystem m_intake;

  /**
   * Constructs an IntakeCargo command, which intakes cargo and moves it to the lower section of the
   * indexer.
   *
   * @param indexer The robot's indexer subsystem
   * @param intake The robot's intake subsystem
   */
  public IntakeCargo(IndexerSubsystem indexer, IntakeSubsystem intake) {
    m_indexer = indexer;
    m_intake = intake;

    addRequirements(indexer, intake);
  }

  @Override
  public void execute() {
    m_intake.runRollerInwards();
    m_indexer.intakeCargo();
  }

  @Override
  public void end(boolean _interrupted) {
    m_indexer.stopBelts();
    m_intake.stopRoller();
  }
}
