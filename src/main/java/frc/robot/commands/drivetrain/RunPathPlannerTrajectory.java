package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class RunPathPlannerTrajectory extends SequentialCommandGroup {

    public RunPathPlannerTrajectory (SwerveDrivetrain drivetrain, PathPlannerTrajectory trajectory) {
        addRequirements(drivetrain);
        ProfiledPIDController thetaController = Constants.Auton.THETA_CONTROLLER;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addCommands(
            new InstantCommand(() -> {
                Pose2d partialInitalPose = trajectory.getInitialPose();
                PathPlannerState initialState = (PathPlannerState) trajectory.getStates().get(0);
                Pose2d actualInitialPose = new Pose2d(partialInitalPose.getTranslation(), initialState.holonomicRotation);
                drivetrain.resetOdometry(actualInitialPose);
            }),
            new PPSwerveControllerCommand(
                trajectory, 
                drivetrain::getPose, 
                Constants.SwerveDrivetrain.SWERVE_KINEMATICS, 
                Constants.Auton.PX_CONTROLLER, 
                Constants.Auton.PY_CONTROLLER,
                thetaController,
                drivetrain::setModuleStates,
                drivetrain
            ),
            new InstantCommand(() -> drivetrain.stop())
        );
    }
    
}
