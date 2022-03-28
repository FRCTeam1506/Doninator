package frc.robot.commands.drivetrain;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.utils.TrajectoryHelper;

public class DriveDistance extends CommandBase {

    private SwerveDrivetrain drivetrain;
    private double distance_m;

    private Trajectory trajectory;
    private Trajectory.State desiredState;
    private Pose2d initialPose, targetPose, currentPose;
    private double x,y;
    private ChassisSpeeds speeds;

    private boolean isForward;
    private HolonomicDriveController controller;
    private Timer timer;

    public DriveDistance (SwerveDrivetrain drivetrain, double distance_m, boolean isForward) {
        this.drivetrain = drivetrain;
        this.distance_m = distance_m;
        this.timer = new Timer();
        this.isForward = isForward;

        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {
        
        //? create controllers
        PIDController xController = new PIDController(1.0, 0.0, 0.0);
        PIDController yController = new PIDController(1.0, 0.0, 0.0);

        ProfiledPIDController thetaController = new ProfiledPIDController(Constants.Auton.PTHETA_CONTROLLER, 0.0, 0.0, Constants.Auton.THETA_CONTROLLER_CONTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        initialPose = drivetrain.getPose();

        if (isForward) {
            x = initialPose.getX() + (distance_m * initialPose.getRotation().getSin());
            y = initialPose.getY() + (distance_m * initialPose.getRotation().getCos());
        } else {
            x = -((distance_m * initialPose.getRotation().getSin()) - initialPose.getX());
            y = -((distance_m * initialPose.getRotation().getCos()) - initialPose.getY());
        }

        targetPose = new Pose2d(x, y, initialPose.getRotation());
        trajectory = TrajectoryHelper.createTrajectory(
            initialPose, 
            List.of(
                new Translation2d(x * 0.2, y * 0.2),
                new Translation2d(x * 0.4, y * 0.4),
                new Translation2d(x * 0.6, y * 0.6),
                new Translation2d(x * 0.8, y * 0.8)
            ),
            targetPose, 
            Constants.Auton.MAX_SPEED_MPS, 
            Constants.Auton.MAX_ACCELERATION_MPSS,
            !isForward
        );
        controller = new HolonomicDriveController(
            xController,
            yController,
            thetaController
        );
        controller.setEnabled(true);

        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        desiredState = trajectory.sample(timer.get());
        currentPose = drivetrain.getPose();
        speeds = controller.calculate(currentPose, desiredState, initialPose.getRotation());
        drivetrain.setModuleStates(Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(speeds));
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
    
}
