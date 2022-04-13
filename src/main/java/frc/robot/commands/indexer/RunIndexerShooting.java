package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class RunIndexerShooting extends CommandBase {
    
    private IndexerSubsystem indexer;

    public RunIndexerShooting (IndexerSubsystem indexer) {
        this.indexer = indexer;
        addRequirements(this.indexer);
    }

    @Override
    public void initialize() {
        indexer.enableIndexing();
        indexer.enableShooting();
    }

    @Override
    public void execute() {
        indexer.highIndexShooting();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.disableIndexing();
        indexer.disableShooting();
    }

}
