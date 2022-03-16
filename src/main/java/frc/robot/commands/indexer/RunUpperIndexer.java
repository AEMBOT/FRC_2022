package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class RunUpperIndexer extends CommandBase {
    private IndexerSubsystem m_indexer;
    private int powerMultiplier;

    public RunUpperIndexer(IndexerSubsystem indexer, boolean invert) {
        m_indexer = indexer;
        powerMultiplier = invert ? -1 : 1;
        addRequirements(indexer);
    }

    public RunUpperIndexer(IndexerSubsystem indexer) {
        this(indexer, false);
    }

    @Override
    public void execute() {
        m_indexer.powerExitSide(powerMultiplier * 0.7);
    }

    @Override
    public void end(boolean _interrupted) {
        m_indexer.stopExitSide();
    }
}
