package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class StopIndexer extends CommandBase {
    
    private IndexerSubsystem indexer;

    public StopIndexer (IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        indexer.disableIndexing();
        indexer.disableShooting();
    }

}
