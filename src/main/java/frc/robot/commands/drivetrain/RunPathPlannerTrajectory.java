package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class RunPathPlannerTrajectory extends SequentialCommandGroup {

    public RunPathPlannerTrajectory (SwerveDrivetrain drivetrain, PathPlannerTrajectory trajectory) {

        //? create controllers
        PIDController xController = new PIDController(Constants.Auton.PX_CONTROLLER, 0.0, 0.0);
        PIDController yController = new PIDController(Constants.Auton.PY_CONTROLLER, 0.0, 0.0);

        ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0.0, 0.0, Constants.Auton.THETA_CONTROLLER_CONTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        //? create trajectory preparation command
        Command c_trajectoryPrep = new InstantCommand(() -> {
            Pose2d partialInitalPose = trajectory.getInitialPose();
            PathPlannerState initialState = (PathPlannerState) trajectory.getStates().get(0);
            Pose2d actualInitialPose = new Pose2d(partialInitalPose.getTranslation(), initialState.holonomicRotation);
            drivetrain.resetOdometry(actualInitialPose);
        });

        //? create trajectory command
        PPSwerveControllerCommand c_trajectory = new PPSwerveControllerCommand(
            trajectory, 
            drivetrain::getPose, 
            Constants.SwerveDrivetrain.SWERVE_KINEMATICS, 
            xController, 
            yController, 
            thetaController, 
            drivetrain::setModuleStates, 
            drivetrain
        );

        //? create drivetrain stop command
        Command c_stop = new InstantCommand(() -> drivetrain.stop());

        addCommands(
            c_trajectoryPrep,
            c_trajectory,
            c_stop
        );
    }
    
}
