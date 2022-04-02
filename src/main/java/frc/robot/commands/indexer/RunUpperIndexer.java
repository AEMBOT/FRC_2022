package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.utilities.enums.CargoDirection;
import frc.robot.subsystems.IndexerSubsystem;

/**
 * A command that runs the upper indexer (which is basically the indexer proper) in the given
 * direction during teleop.
 */
public class RunUpperIndexer extends CommandBase {
  private IndexerSubsystem m_indexer;
  private CargoDirection m_direction;

  /**
   * Constructs a RunUpperIndexer command, which runs the indexer in the given direction.
   *
   * @param indexer The robot's indexer subsystem
   * @param direction The direction to move cargo in
   */
  public RunUpperIndexer(IndexerSubsystem indexer, CargoDirection direction) {
    m_indexer = indexer;
    m_direction = direction;
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    if (m_direction == CargoDirection.Intake) {
      m_indexer.moveCargoUp();
    } else if (m_direction == CargoDirection.Eject) {
      m_indexer.moveCargoDown();
    }
  }

  @Override
  public void end(boolean _interrupted) {
    m_indexer.stopBelts();
  }
}
