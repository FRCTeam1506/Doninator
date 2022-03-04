package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class RunPathPlannerTrajectory extends SequentialCommandGroup {

    public RunPathPlannerTrajectory (SwerveDrivetrain drivetrain, PathPlannerTrajectory trajectory) {
        addRequirements(drivetrain);
        addCommands(
            new InstantCommand(() -> {
                drivetrain.setPose(trajectory.getInitialPose());
                drivetrain.setGyro(trajectory.getInitialPose().getRotation().getDegrees());
            }),
            new PPSwerveControllerCommand(
                trajectory, 
                drivetrain::getPose, 
                Constants.SwerveDrivetrain.SWERVE_KINEMATICS, 
                Constants.Auton.PX_CONTROLLER, 
                Constants.Auton.PY_CONTROLLER,
                Constants.Auton.THETA_CONTROLLER,
                drivetrain::setModuleStates,
                drivetrain
            ),
            new InstantCommand(() -> drivetrain.stop())
        );
    }
    
}
