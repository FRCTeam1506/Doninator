package frc.robot.commands.autons;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.DriveDistance;
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

public class TwoBall1 extends SequentialCommandGroup {

    public TwoBall1 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, TurretSubsystem turret, PathPlannerTrajectory trajectory) {
        super(
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory(drivetrain, trajectory),
                new ExtendAndIntake(intake)
            ).withTimeout(6.5),

            new StopAndRetract(intake).withTimeout(0.1),

            new ParallelCommandGroup(
                new RunIndexer(indexer),
                new AimTurret(turret, () -> 0.0)
            ).withTimeout(1.4),

            new ShootAndIndex(shooter, indexer, turret, 0.0).perpetually().withTimeout(7)
        );
    }
    
}
