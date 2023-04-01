package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CandleSubsystem;
import frc.robot.commands.skittles;
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


public class DropCone extends SequentialCommandGroup {


    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public DropCone (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, OurBeautifulGlowingCANdleSubsystem candle) {
    

        addCommands(
            new ParallelCommandGroup(
                new ZeroGyro(drivetrain).withTimeout(0.05),
                new skittles(candle)
            ).withTimeout(0.01),
            new ParallelCommandGroup(
                new ExtractCone(intake),
                // new SetHighAuto(telescope),
                new armMid(arm)
            ).withTimeout(0.2),
            new SetHighAuto(telescope).withTimeout(1.4),
            // new JustOuttake(intake).withTimeout(0.25),
            new JustOuttakeSpeed(intake, 0.3).withTimeout(0.2), //justouttakeslow 0.25
            new RetractCone(intake).withTimeout(.05),   //.andThen(new SetLow(telescope).withTimeout(1)),
            new SetLow(telescope).withTimeout(1),
            new armLow(arm).withTimeout(0.05),
            new JustStopIntake(intake).withTimeout(0.05)
        );
    }
    
}
