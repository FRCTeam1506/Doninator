package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAndIndex extends ParallelCommandGroup {
    
    public ShootAndIndex (ShooterSubsystem shooter, IndexerSubsystem indexer, double velocity_rpm) {
        super(
            new RunIndexer(indexer),
            new RunShooter(shooter, velocity_rpm)
        );
    }

}
