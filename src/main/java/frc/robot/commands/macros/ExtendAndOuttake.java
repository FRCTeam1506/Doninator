package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ExtendAndOuttake extends CommandBase {
    
    private IntakeSubsystem intake;
    private IndexerSubsystem indexer;

    public ExtendAndOuttake (IntakeSubsystem intake, IndexerSubsystem indexer) {
        this.intake = intake;
        this.indexer = indexer;
        addRequirements(this.intake, this.indexer);
    }

    @Override
    public void execute() {
        intake.extend();
        intake.outtake();
        indexer.reverse();
    }

}
