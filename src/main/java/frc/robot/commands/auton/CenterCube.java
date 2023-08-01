package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.BackwardsFast;
import frc.robot.commands.drivetrain.BackwardsPark;
import frc.robot.commands.drivetrain.BackwardsSlow;
import frc.robot.commands.drivetrain.ForwardFast;
import frc.robot.commands.drivetrain.ForwardSlow2;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.Stop;
import frc.robot.commands.drivetrain.ZeroGyro;
import frc.robot.commands.intake.*;
import frc.robot.commands.macros.ground;
import frc.robot.commands.macros.high;
import frc.robot.commands.telescoping.SetHigh;
import frc.robot.commands.telescoping.SetLow;
import frc.robot.commands.telescoping.SetMid;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.OurBeautifulGlowingCANdleSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingSubsystem;


public class CenterCube extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public CenterCube (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, OurBeautifulGlowingCANdleSubsystem candle, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2, PathPlannerTrajectory trajectory3) {
        
        addCommands(
            new DropCone(drivetrain, intake, telescope, arm, candle),
            new armHigh(arm).withTimeout(0.3),
            new RunPathPlannerTrajectory2(drivetrain, trajectory1, true).withTimeout(4),
            new ParallelCommandGroup(
                new armLow(arm),
                new JustOuttake(intake)
            ).withTimeout(0.05),
            new RunPathPlannerTrajectory2(drivetrain, trajectory2, true),
            new ParallelCommandGroup(
                new armHigh(arm).withTimeout(0.1),
                new JustStopIntake(intake).withTimeout(0.1),
                new RunPathPlannerTrajectory2(drivetrain, trajectory3, true)
            ),
            new ForwardFast(drivetrain).until(() -> Math.abs(drivetrain.getGyroRoll()) <2),
            new JustIntake(intake).withTimeout(0.1),
            // new BackwardsSlow(drivetrain).withTimeout(1.25), //1.75
            new BackwardsFast(drivetrain).withTimeout(0.7),
            new Stop(drivetrain)
            // new InstantCommand(() -> drivetrain.
        );
    }
    
}
