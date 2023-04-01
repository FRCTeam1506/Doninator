package frc.robot.commands.auton;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
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
import frc.robot.utils.TrajectoryHelper;


public class Wings2 extends SequentialCommandGroup {

    private static final HashMap<String, Command> AUTO_EVENT_MAP = new HashMap<>();

    //intake and outtake work for cube, so inverse for cone
    public Wings2 (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, 
                  ArmSubsystem arm, OurBeautifulGlowingCANdleSubsystem candle, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2) {
        PathPlannerTrajectory trajectory3 = TrajectoryHelper.loadHolonomicPathPlannerTrajectory(
            "BB_LW3",4,2.0, false);

        AUTO_EVENT_MAP.put("event1", new DropCone(drivetrain, intake, telescope, arm, candle));
        AUTO_EVENT_MAP.put("intakecude", new JustOuttakeSpeed(intake, 0.4).withTimeout(2));
        AUTO_EVENT_MAP.put("event2", new JustStopIntake(intake).withTimeout(0.1));

        addCommands(
            new DropCone(drivetrain, intake, telescope, arm, candle),
            new FollowPathWithEvents(new RunPathPlannerTrajectory(drivetrain, trajectory1,false, true),
                trajectory1.getMarkers(),
                AUTO_EVENT_MAP),
            new JustStopIntake(intake).withTimeout(0.1),
            new armHigh(arm).withTimeout(0.1),

            new FollowPathWithEvents(new RunPathPlannerTrajectory(drivetrain, trajectory2,false, false),
                trajectory2.getMarkers(),
                AUTO_EVENT_MAP),
                new armHigh(arm).withTimeout(0.1),
            // new ParallelCommandGroup(
            //     new RunPathPlannerTrajectory(drivetrain, trajectory2,false, false),
            //     new armHigh(arm).withTimeout(0.1)
                // new SetLow(telescope).withTimeout(4),
                //new SetHigh(telescope).withTimeout(1)
            // ),
            new JustIntake(intake).withTimeout(0.2),

            new FollowPathWithEvents(new RunPathPlannerTrajectory(drivetrain, trajectory3,false, false),
                trajectory2.getMarkers(),
                AUTO_EVENT_MAP),
                new armHigh(arm).withTimeout(0.1),

            // new ParallelCommandGroup(
            //     new RunPathPlannerTrajectory(drivetrain, trajectory3,false, false),
            //     new armHigh(arm).withTimeout(0.1)
            //     // new SetLow(telescope).withTimeout(4),
            //     //new SetHigh(telescope).withTimeout(1)
            // ),
            // new SetLow(telescope).withTimeout(2),
            new armLow(arm).withTimeout(0.2),
            new JustStopIntake(intake).withTimeout(0.1)

        );
    }
    
}
