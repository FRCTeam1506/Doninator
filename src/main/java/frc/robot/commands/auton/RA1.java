package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.commands.arm.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.intake.*;
import frc.robot.commands.macros.ground;
import frc.robot.commands.macros.high;
import frc.robot.commands.macros.mid;
import frc.robot.commands.telescoping.SetHigh;
import frc.robot.commands.telescoping.SetLow;
import frc.robot.commands.telescoping.SetZeroTest;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingSubsystem;


public class RA1 extends SequentialCommandGroup {

    public RA1 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3) {
        
        addCommands(
            new ParallelCommandGroup(
                new SetHigh(telescope),
                new armMid(arm)
            ).withTimeout(3),
            new JustIntake(intake).withTimeout(1),
            new JustStopIntake(intake).withTimeout(0.1),
            new ParallelCommandGroup(
                new SetZeroTest(telescope),
                new armLow(arm),
                new RunPathPlannerTrajectory2(drivetrain, trajectory1)
            )
            );

        System.out.println("part two auton");

        // addCommands(
        //     new JustIntake(intake).withTimeout(1.5),
        //     new RunPathPlannerTrajectory2(drivetrain, trajectory2)
            // new high(telescope, arm).withTimeout(2.5),
            // new JustOuttake(intake),
            // new RunPathPlannerTrajectory2(drivetrain, trajectory3)
            // );
    
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
