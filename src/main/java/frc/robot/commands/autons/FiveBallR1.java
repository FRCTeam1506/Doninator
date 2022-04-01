package frc.robot.commands.autons;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.indexer.RunIndexer;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.macros.ShootAndIndex;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;

public class FiveBallR1 extends SequentialCommandGroup {

    public FiveBallR1 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3, PathPlannerTrajectory trajectory4) {
        
        addCommands(
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory1),
                new ExtendAndIntake(intake)
            ).withTimeout(3.0),
            new Retract(intake)
        );

        addCommands(
            new ShootAndIndex(shooter, indexer, 1200.0).withTimeout(2.0)
        );

        addCommands(
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory2),
                new ExtendAndIntake(intake)
            ).withTimeout(4.0),
            new Retract(intake)
        );

        addCommands(
            new ParallelCommandGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory3),
                new ExtendAndIntake(intake)
            ).withTimeout(5.3),
            new Retract(intake)
        );

        addCommands(
            new RunPathPlannerTrajectory(drivetrain, trajectory4).withTimeout(2.0),
            new ShootAndIndex(shooter, indexer, 2100.0)
        );
    }
    
}
