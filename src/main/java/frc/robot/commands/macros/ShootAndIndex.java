package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndIndex extends SequentialCommandGroup {

    private IndexerSubsystem indexer;
    
    public ShootAndIndex (ShooterSubsystem shooter, IndexerSubsystem indexer, double velocity_rpm) {
        super(
            new InstantCommand(() -> indexer.enableShooting(), indexer),
            new ParallelCommandGroup(
                new RunIndexer(indexer),
                new RunShooter(shooter, velocity_rpm)
            )
        );

        this.indexer = indexer;
        addRequirements(this.indexer);
    }

    @Override
    public void end(boolean interrupted) {
        indexer.disableShooting();
    }

}
