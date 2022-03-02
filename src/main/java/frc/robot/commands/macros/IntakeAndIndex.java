package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAndIndex extends CommandBase {

    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;
    
    public IntakeAndIndex (IntakeSubsystem intake, IndexerSubsystem indexer) {
        this.intake = intake;
        this.indexer = indexer;
    }

    @Override
    public void initialize() {
        indexer.enableIndexing();
        indexer.disableShooting();
    }

    @Override
    public void execute() {
        intake.extend();
        intake.intake();
    }

    @Override
    public void end(boolean interrupted) {
        indexer.disableIndexing();
    }

}
