package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.intake.*;
import frc.robot.commands.telescoping.SetHigh;
import frc.robot.commands.telescoping.SetLow;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingSubsystem;


public class RA1 extends SequentialCommandGroup {

    public RA1 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2) {
        
        addCommands(
            new PrintCommand("pick up 1, drive to 2, shoot"),
            // new ParallelDeadlineGroup(
                new SetHigh(telescope).withTimeout(1.5),
                new JustOuttakeHigh(intake, arm).withTimeout(1.5),
                new SetLow(telescope).withTimeout(1.5),
                new JustOuttakeHigh(intake, arm).withTimeout(1.5),

            // ).withTimeout(4.0)
            new RunPathPlannerTrajectory2(drivetrain, trajectory1)
            );

        addCommands(
            new PrintCommand("pick up 1, drive to 2, shoot"),
            // new ParallelDeadlineGroup(
                new SetHigh(telescope).withTimeout(1.5),
                new JustOuttakeHigh(intake, arm).withTimeout(1.5),
            // ).withTimeout(4.0)
            new RunPathPlannerTrajectory2(drivetrain, trajectory1)
            );
    
        // addCommands(
        //     new PrintCommand("pick up 3, shoot"),
        //     new ParallelDeadlineGroup(
        //         new RunPathPlannerTrajectory2(drivetrain, trajectory2),
        //         new ExtendAndIntake(intake)
        //     ).withTimeout(1.0),
        //     new ShootAndIndex(shooter, indexer, 1810.0).withTimeout(1.0)
        // );

    }
    
}
