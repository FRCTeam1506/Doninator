package frc.robot.commands.autons;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.commands.intake.StopAndRetract;
import frc.robot.commands.macros.ShootAndIndex;
import frc.robot.commands.turret.AimTurret;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class FourBall1 extends SequentialCommandGroup {

    public FourBall1 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, TurretSubsystem turret, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3) {
        super(
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory1),
                new ExtendAndIntake(intake)
            ).withTimeout(4.0),
            new StopAndRetract(intake).withTimeout(0.1),
            new ParallelCommandGroup(
                new RunIndexer(indexer),
                new AimTurret(turret, () -> 0.0)
            ).withTimeout(0.8),
            new ShootAndIndex(shooter, indexer, turret, 0.0).perpetually().withTimeout(4.0),
            
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory2),
                new ExtendAndIntake(intake)
            ).withTimeout(6.0),
            new StopAndRetract(intake).withTimeout(0.1),
            
            new RunPathPlannerTrajectory(drivetrain, trajectory3).withTimeout(4.0),
            new ShootAndIndex(shooter, indexer, turret, 0.0).perpetually().withTimeout(7)
        );
    }
    
}
