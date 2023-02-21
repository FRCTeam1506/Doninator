package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class RunPathPlannerTrajectory2 extends SequentialCommandGroup {
    
    public RunPathPlannerTrajectory2 (SwerveDrivetrain drivetrain, PathPlannerTrajectory trajectory) {
        addRequirements(drivetrain);
        addCommands(
            new InstantCommand(() -> drivetrain.setPose(trajectory.getInitialPose())),
            new PPSwerveControllerCommand(
                trajectory, 
                drivetrain::getPose, 
                Constants.SwerveDrivetrain.SWERVE_KINEMATICS, 
                Constants.Auton.PX_CONTROLLER, 
                Constants.Auton.PY_CONTROLLER,
                Constants.Auton.THETA_CONTROLLER,
                drivetrain::setModuleStates,
                drivetrain
            )
        );
    }
    
}
