package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexer extends CommandBase {

    private IndexerSubsystem indexer;
    
    public RunIndexer (IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        this.indexer.start_run();
    }

}
