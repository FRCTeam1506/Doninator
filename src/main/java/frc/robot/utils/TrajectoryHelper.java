package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import frc.robot.Constants;


public class TrajectoryHelper {
    
    public static Trajectory loadWPILibTrajectoryFromFile (String filename) {
        Trajectory trajectory = null;

        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + filename + ".wpilib.json");
        try {
            System.out.println("Loading trajectory at " + trajectoryPath.toString());
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory at " + trajectoryPath.toString(), e.getStackTrace());
        }

        return trajectory;
    }

    public static PathPlannerTrajectory loadHolonomicPathPlannerTrajectory (String filename, double maxVelocity, double maxAcceleration) {
        return PathPlanner.loadPath(filename, maxVelocity, maxAcceleration, false);
    }

    public static PathPlannerTrajectory loadHolonomicPathPlannerTrajectory (String filename) {
        return PathPlanner.loadPath(filename, Constants.Auton.MAX_SPEED_MPS, Constants.Auton.MAX_ACCELERATION_MPSS, false);
    }

    public static Trajectory createTrajectory (Pose2d initial,
                                                List<Translation2d> interiorWaypoints,
                                                Pose2d end,
                                                double maxSpeed,
                                                double maxAcceleration,
                                                boolean reversed) {

        TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAcceleration)
            .setKinematics(Constants.SwerveDrivetrain.SWERVE_KINEMATICS)
            .setReversed(reversed);

        Trajectory trajectory =
            TrajectoryGenerator.generateTrajectory(
                initial,
                interiorWaypoints,
                end,
                config
            );

        return trajectory;
    }

}
