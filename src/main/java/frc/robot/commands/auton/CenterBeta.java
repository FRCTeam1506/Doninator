package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.*;
import frc.robot.commands.drivetrain.ForwardSlow2;
import frc.robot.commands.drivetrain.RunPathPlannerTrajectory2;
import frc.robot.commands.drivetrain.ZeroGyro;
import frc.robot.commands.intake.*;
import frc.robot.commands.macros.ground;
import frc.robot.commands.macros.high;
import frc.robot.commands.telescoping.SetHigh;
import frc.robot.commands.telescoping.SetLow;
import frc.robot.commands.telescoping.SetMid;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.TelescopingSubsystem;
import java.lang.Math;


public class CenterBeta extends SequentialCommandGroup {

    private Translation2d translation;
    //intake and outtake work for cube, so inverse for cone
    //RA100 only for one PathPlannerTrajectory --- simple auton
    public CenterBeta (SwerveDrivetrain drivetrain, IntakeSubsystem intake, TelescopingSubsystem telescope, ArmSubsystem arm, PathPlannerTrajectory trajectory1, PathPlannerTrajectory trajectory2) {
        
        translation = new Translation2d(.2, 0);
        addCommands(
            // new DropCone(drivetrain, intake, telescope, arm),

            // new RunPathPlannerTrajectory2(drivetrain, trajectory1),
            //.until stop command when condition is true keep running while false
            //  new RunPathPlannerTrajectory2(drivetrain, trajectory2).until(() -> (Math.abs(drivetrain.getGyroPitchDegrees()) >15)), // ? true : false
            // new ForwardSlow2(drivetrain).until(() -> (Math.abs(drivetrain.getGyroPitchDegrees()) <2) ? true : false),
            new ForwardSlow2(drivetrain),
        //    translation = new Translation2d(yAxis, xAxis).times(Constants.SwerveDrivetrain.MAX_SPEED);
        //rotation = rAxis * Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY;
            //drivetrain.drive(translation, 0, true, true)
            //drivetrain.drive(translation, 0, isFinished(), isFinished()),
            //new ForwardSlow2(drivetrain),
            new PrintCommand("Running" + drivetrain.getGyroAngleDegrees())

            //new InstantCommand(() -> drivetrain.
        );

        // for(int i = 0; i<1000; i++){
        //     System.out.println("BBC");
        // }
    }
    
}
