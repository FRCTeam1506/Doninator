package frc.robot.commands.autons;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.commands.intake.Retract;
import frc.robot.commands.macros.ShootAndIndex;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class FiveBallR1 extends SequentialCommandGroup {

    public FiveBallR1 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3, PathPlannerTrajectory trajectory4) {
        
        addCommands(
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory1),
                new ExtendAndIntake(intake)
            ).withTimeout(3.0),
            new InstantCommand(() -> intake.retract()),
            new PrintCommand("step 1")
        );

        addCommands(
            new ShootAndIndex(shooter, indexer, 1740.0).withTimeout(1.3),
            new PrintCommand("step 2")
        );

        addCommands(
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory2),
                new ExtendAndIntake(intake)
            ).withTimeout(4.0),
            new ExtendAndIntake(intake).withTimeout(0.3),
            new InstantCommand(() -> intake.retract()),
            new PrintCommand("step 3")
        );

        addCommands(
            new ShootAndIndex(shooter, indexer, 1770.0).withTimeout(1.3),
            new PrintCommand("step 4")
        );

        addCommands(
            new InstantCommand(() -> indexer.enableIndexing(), indexer),
            new ParallelCommandGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory3),
                new ExtendAndIntake(intake)
            ).withTimeout(4.87),
            new ExtendAndIntake(intake).withTimeout(0.3),
            new InstantCommand(() -> intake.retract()),
            new PrintCommand("step 5")
        );

        addCommands(
            new RunPathPlannerTrajectory(drivetrain, trajectory4),
            new InstantCommand(() -> drivetrain.zeroGyro()),
            new ShootAndIndex(shooter, indexer, 1930.0),
            new PrintCommand("step 6")
        );
    }
    
}
