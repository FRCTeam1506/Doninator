package frc.robot.commands.autons;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.intake.ExtendAndIntake;
import frc.robot.commands.macros.ShootAndIndex;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;

public class FourBallR1 extends SequentialCommandGroup {

    public FourBallR1 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3) {
        super(
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory1),
                new ExtendAndIntake(intake)
            ),
            new ShootAndIndex(shooter, indexer, 1720.0).perpetually().withTimeout(4),

            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory2),
                new ExtendAndIntake(intake)
            ),

            new RunPathPlannerTrajectory(drivetrain, trajectory3),
            new ShootAndIndex(shooter, indexer, 1720.0).perpetually().withTimeout(4)
        );
    }
    
}
