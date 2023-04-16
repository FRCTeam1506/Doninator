package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.BackwardsSlow;
import frc.robot.commands.drivetrain.ForwardSlow2;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.Stop;
import frc.robot.commands.drivetrain.ZeroGyro;
import frc.robot.commands.drivetrain.turn180;
import frc.robot.commands.intake.*;
import frc.robot.commands.macros.ground;
import frc.robot.commands.macros.high;
import frc.robot.commands.telescoping.SetHigh;
import frc.robot.commands.telescoping.SetHighAuto;
import frc.robot.commands.telescoping.SetLow;
import frc.robot.commands.telescoping.SetMid;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OurBeautifulGlowingCANdleSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingSubsystem;


public class bumpWorlds extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public bumpWorlds (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, OurBeautifulGlowingCANdleSubsystem candle, PathPlannerTrajectory trajectory01, PathPlannerTrajectory trajectory02, PathPlannerTrajectory trajectory03, PathPlannerTrajectory trajectory11, PathPlannerTrajectory trajectory12, PathPlannerTrajectory trajectory13, PathPlannerTrajectory trajectory21, PathPlannerTrajectory trajectory22, PathPlannerTrajectory trajectory23){
        
        addCommands(
            //drop cone
            new DropCone(drivetrain, intake, telescope, arm, candle),
            new armHigh(arm).withTimeout(0.3),

            //to cube 1
            new RunPathPlannerTrajectory2(drivetrain, trajectory01, true),
            new RunPathPlannerTrajectory2(drivetrain, trajectory02, true),
            new ParallelCommandGroup(
                new RunPathPlannerTrajectory2(drivetrain, trajectory03, true),
                new JustOuttakeSpeed(intake, 0.4).withTimeout(1),
                new armLow(arm).withTimeout(1)
            ),

            //score cube 1
            new RunPathPlannerTrajectory2(drivetrain, trajectory11, true),
            new RunPathPlannerTrajectory2(drivetrain, trajectory12, true),
            new ParallelCommandGroup(
                new RunPathPlannerTrajectory2(drivetrain, trajectory13, true),
                new armMid(arm).withTimeout(1),
                new SetHighAuto(telescope).withTimeout(1.4)
            ),
            new JustOuttakeSpeed(intake, -0.7).withTimeout(0.1),
            new SetLow(telescope).withTimeout(0.5),

            //get cube 2
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory2(drivetrain, trajectory21, true),
                new SetLow(telescope),
                new armLow(arm)
            ),
            new RunPathPlannerTrajectory2(drivetrain, trajectory22, true),
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory2(drivetrain, trajectory23, true),
                new JustOuttakeSpeed(intake, -0.7).withTimeout(0.1)
            ),

            //stop and hold
            new ParallelCommandGroup(
                new JustStopIntake(intake),
                new armHigh(arm)
            )

            //score cube 2***********
            // new RunPathPlannerTrajectory2(drivetrain, trajectory31, true),
            // new RunPathPlannerTrajectory2(drivetrain, trajectory32, true),
            // new ParallelCommandGroup(
            //     new RunPathPlannerTrajectory2(drivetrain, trajectory33, true),
            //     new armMid(arm).withTimeout(1),
            //     new SetMid(telescope).withTimeout(1.4)
            // ),
            // new JustOuttakeSpeed(intake, -0.7).withTimeout(0.1),
            // new SetLow(telescope).withTimeout(0.5),






            
            //new InstantCommand(() -> drivetrain.
        );
    }
    
}
