package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Timer;
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


public class CenterLeft extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public CenterLeft (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, 
                  ArmSubsystem arm, OurBeautifulGlowingCANdleSubsystem candle, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3) {
        
        addCommands(
            new DropCone(drivetrain, intake, telescope, arm, candle),
            new JustStopIntake(intake).withTimeout(0.01),
            // new armLow(arm).withTimeout(.5),
            new ParallelCommandGroup(
                new JustOuttakeSpeed(intake, 0.4).withTimeout(3.0),
                new RunPathPlannerTrajectory2(drivetrain, trajectory1, true)
            ),
            //    FollowPathWithEvents(
            //new RunPathPlannerTrajectory2(drivetrain, trajectory1)
            new JustStopIntake(intake).withTimeout(0.01),
            new armMid(arm).withTimeout(.2),
            new ParallelCommandGroup(
                new RunPathPlannerTrajectory2(drivetrain, trajectory2,true),
                // new armMid(arm).withTimeout(0.7),
                // new SetLow(telescope).withTimeout(4),
                new SetMid(telescope).withTimeout(.8)
            ),
            new JustIntake(intake).withTimeout(0.2),
            // new SetLow(telescope).withTimeout(.01),
            // new armLow(arm).withTimeout(0.01),
            new JustStopIntake(intake).withTimeout(0.1),

            new ParallelCommandGroup(
                new RunPathPlannerTrajectory2(drivetrain, trajectory3 ,true),
                // new armMid(arm).withTimeout(0.7),
                new SetLow(telescope).withTimeout(1)
            ),

            new ForwardSlow2(drivetrain).until(() -> drivetrain.getGyroRoll() <11),
            new BackwardsSlow(drivetrain).withTimeout(1.25), //1.75
            new Stop(drivetrain)

        );
    }
    
}
