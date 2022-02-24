package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAndIndex extends ParallelCommandGroup {
    
    public IntakeAndIndex (IntakeSubsystem intake, IndexerSubsystem indexer) {
        super(
            new ExtendAndIntake(intake),
            new RunIndexer(indexer)
        );
    }

}
