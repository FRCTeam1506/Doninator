package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAndIndex extends ParallelCommandGroup {
    
    public IntakeAndIndex (IntakeSubsystem intake, IndexerSubsystem indexer) {
        super(
            new ExtendAndIntake(intake),
            new SequentialCommandGroup(
                new InstantCommand(() -> indexer.disableShooting(), indexer),
                new RunIndexer(indexer)
            )
        );
    }

}
