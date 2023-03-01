package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import frc.robot.commands.arm.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
import frc.robot.commands.drivetrain.*;


public class RA1 extends SequentialCommandGroup {

    public RA1 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3) {
        
        addCommands(

        // new ZeroGyro(drivetrain).withTimeout(0.1),
        new ZeroGyro(drivetrain).withTimeout(0.1),
        new ParallelCommandGroup(
            new SetHigh(telescope),
            new armMid(arm)
        ).withTimeout(2),
        new JustOuttake(intake).withTimeout(0.25),
        new SetLow(telescope).withTimeout(1.5),
        new armLow(arm).withTimeout(1),
        new JustStopIntake(intake).withTimeout(0.1),
        new ParallelCommandGroup(
            new JustOuttake(intake).withTimeout(5.2),
            new RunPathPlannerTrajectory2(drivetrain, trajectory1)
        )
        );
    }
    
}
