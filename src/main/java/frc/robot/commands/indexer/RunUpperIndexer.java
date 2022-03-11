package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class RunUpperIndexer extends CommandBase {
    private IndexerSubsystem m_indexer;

    public RunUpperIndexer(IndexerSubsystem indexer) {
        m_indexer = indexer;
        addRequirements(indexer);
    }

    @Override
    public void execute() {
        m_indexer.powerExitSide(0.7);
    }

    @Override
    public void end(boolean _interrupted) {
        m_indexer.stopExitSide();
    }
}
