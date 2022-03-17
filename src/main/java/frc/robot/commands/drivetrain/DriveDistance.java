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
        PIDController xController = new PIDController(Constants.Auton.PX_CONTROLLER, 0.0, 0.0);
        PIDController yController = new PIDController(Constants.Auton.PY_CONTROLLER, 0.0, 0.0);

        ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0.0, 0.0, Constants.Auton.THETA_CONTROLLER_CONTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        initialPose = drivetrain.getPose();
        x = initialPose.getX() + (distance_m * initialPose.getRotation().getCos());
        y = initialPose.getY() + (distance_m * initialPose.getRotation().getSin());

        if (!isForward) {
            x = -x;
            y = -y;
        }

        targetPose = new Pose2d(x, y, initialPose.getRotation());
        trajectory = TrajectoryHelper.createTrajectory(
            initialPose, 
            List.of(
                new Translation2d(x * 0.25, y * 0.25),
                new Translation2d(x * 0.50, y * 0.50),
                new Translation2d(x * 0.75, y * 0.75)
            ),
            targetPose, 
            Constants.Auton.MAX_SPEED_MPS, 
            Constants.Auton.MAX_ACCELERATION_MPSS
        );
        controller = new HolonomicDriveController(
            xController,
            yController,
            thetaController
        );
        controller.setEnabled(true);
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
