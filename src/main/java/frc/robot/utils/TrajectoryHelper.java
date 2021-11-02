package frc.robot.utils;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.spline.Spline;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;


public class TrajectoryHelper {
    
    public static Trajectory loadTrajectoryFromFile(String filename) {
        Trajectory trajectory = null;

        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/" + filename + ".wpilib.json");
        try {
            System.out.println("Loading trajectory at " + trajectoryPath.toString());
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e) {
            DriverStation.reportError("Unable to open trajectory at " + filename, e.getStackTrace());
        }

        return trajectory;
    }

    public static Trajectory createTrajectory (SwerveDrivetrain drivetrain,
                                                Spline.ControlVector initial,
                                                List<Translation2d> interiorWaypoints,
                                                Spline.ControlVector end,
                                                double maxSpeed,
                                                double maxAcceleration) {

        TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAcceleration)
            .setKinematics(Constants.SwerveDrivetrain.SWERVE_KINEMATICS);

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
