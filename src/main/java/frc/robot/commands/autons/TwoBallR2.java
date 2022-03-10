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

public class TwoBallR2 extends SequentialCommandGroup {

    public TwoBallR2 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, PathPlannerTrajectory trajectory) {
        super(
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory),
                new ExtendAndIntake(intake)
            ),

            new ShootAndIndex(shooter, indexer, 1740.0).perpetually().withTimeout(4) // 1720.0
        );
    }
    
}
