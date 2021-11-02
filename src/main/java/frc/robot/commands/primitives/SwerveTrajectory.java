package frc.robot.commands.primitives;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveTrajectory extends SequentialCommandGroup {

    public SwerveTrajectory (SwerveDrivetrain drivetrain, Trajectory trajectory) {
        var thetaController = new ProfiledPIDController(
            Constants.Auton.PTHETA_CONTROLLER, 0, 0, Constants.Auton.THETA_CONTROLLER_CONTRAINTS
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand command = new SwerveControllerCommand(
            trajectory, 
            drivetrain::getPose,
            Constants.SwerveDrivetrain.SWERVE_KINEMATICS,
            new PIDController(Constants.Auton.PX_CONTROLLER, 0, 0),
            new PIDController(Constants.Auton.PY_CONTROLLER, 0, 0),
            thetaController,
            drivetrain::setModuleStates,
            drivetrain
        );

        addCommands(
            new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
            command
        );
    }
    
}
