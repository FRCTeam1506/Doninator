package frc.robot.commands.drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class RunPathPlannerTrajectory extends CommandBase {
    
    private SwerveDrivetrain drivetrain;
    private ProfiledPIDController rotationalPID;
    private HolonomicDriveController holoController;

    private Pose2d currentPosition;
    private PathPlannerTrajectory trajectory;
    private ChassisSpeeds chassisSpeeds;
    private PathPlannerState currentTrajectoryState;

    private final Timer timer = new Timer();


    public RunPathPlannerTrajectory (SwerveDrivetrain drivetrain, PathPlannerTrajectory trajectory) {
        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        this.chassisSpeeds = new ChassisSpeeds();
        this.rotationalPID = Constants.Auton.ROT_PID_CONTROLLER;
    }


    @Override
    public void initialize() {
        this.rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
        this.holoController = new HolonomicDriveController(Constants.Auton.PX_CONTROLLER, Constants.Auton.PY_CONTROLLER, Constants.Auton.ROT_PID_CONTROLLER);
        this.timer.reset();
        this.timer.start();
    }

    @Override
    public void execute() {
        var currentTime = timer.get();
        this.currentTrajectoryState = (PathPlannerState) this.trajectory.sample(currentTime);
        this.currentPosition = this.drivetrain.getPose();
        this.chassisSpeeds = this.holoController.calculate(this.currentPosition, this.currentTrajectoryState, this.currentTrajectoryState.holonomicRotation);
        this.drivetrain.setModuleStates(Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(this.chassisSpeeds));
    }

    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();
        // this.drivetrain.executeDefense();
    }


}
