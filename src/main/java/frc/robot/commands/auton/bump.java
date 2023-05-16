package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.ZeroGyro;
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


public class bump extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public bump (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, 
                  ArmSubsystem arm, OurBeautifulGlowingCANdleSubsystem candle, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory15, PathPlannerTrajectory trajectory2) {
        
        addCommands(
            new DropCone(drivetrain, intake, telescope, arm, candle),
            new JustStopIntake(intake).withTimeout(0.01),
            new armLow(arm).withTimeout(.05),
            // new RunPathPlannerTrajectory2(drivetrain, trajectory1, true),
            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory2(drivetrain, trajectory1, true),
                new JustOuttakeSpeed(intake, 0.4).withTimeout(1),//4.4
                new dropArmSpecific(arm,drivetrain)
            ),
            //    FollowPathWithEvents(
            //new RunPathPlannerTrajectory2(drivetrain, trajectory1)
            new JustStopIntake(intake).withTimeout(0.01),

            new ParallelDeadlineGroup(
                new RunPathPlannerTrajectory2(drivetrain, trajectory2, true),
                new armMid(arm).withTimeout(0.7),
                new TapAutoCube(intake)
                // new SetLow(telescope).withTimeout(4),
            ),
            new SetHighAuto(telescope).withTimeout(1.5),
            new JustIntake(intake).withTimeout(0.2),
            new armHigh(arm).withTimeout(0.01),
            // new SetLow(telescope).withTimeout(0.4),
            // new armLow(arm).withTimeout(0.1),
            new JustStopIntake(intake).withTimeout(0.1),

            new ParallelCommandGroup(
                new SetLow(telescope).withTimeout(0.4),
                new armHigh(arm)
            )
        );
    }
    
}