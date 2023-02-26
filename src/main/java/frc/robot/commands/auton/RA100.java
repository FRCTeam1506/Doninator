package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.intake.*;
import frc.robot.commands.macros.ground;
import frc.robot.commands.macros.high;
import frc.robot.commands.telescoping.SetHigh;
import frc.robot.commands.telescoping.SetLow;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingSubsystem;


public class RA100 extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    public RA100 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, PathPlannerTrajectory trajectory1) {
        
        addCommands(
            new ParallelCommandGroup(
                new SetHigh(telescope),
                new armMid(arm)
            ).withTimeout(3),
            new JustIntake(intake).withTimeout(1),
            new ParallelCommandGroup(
                new ground(telescope, arm).withTimeout(3.5),
                new RunPathPlannerTrajectory2(drivetrain, trajectory1)
            )
        );
    }
    
}
