package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.utils.swerve.SwerveModule;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrain extends SubsystemBase {

    private static final Pose2d HUB_CENTER = new Pose2d(8.24, 4.16, new Rotation2d(0.0));
    private static final Pose2d PROTECTED_ZONE = new Pose2d(3.77, 5.23, new Rotation2d(0.0));
    
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] swerveModules;
    private Pigeon2 gyro;
    private Field2d field;

    public SwerveDrivetrain() {
        this.gyro = new Pigeon2(Constants.SwerveDrivetrain.GYRO_ID, "rio");
        this.gyro.configFactoryDefault();
        this.zeroGyro();

        this.swerveOdometry = new SwerveDriveOdometry(Constants.SwerveDrivetrain.SWERVE_KINEMATICS, this.getYaw());

        this.swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveDrivetrain.Mod0.constants),
            new SwerveModule(1, Constants.SwerveDrivetrain.Mod1.constants),
            new SwerveModule(2, Constants.SwerveDrivetrain.Mod2.constants),
            new SwerveModule(3, Constants.SwerveDrivetrain.Mod3.constants)
        };

        this.field = new Field2d();

        dashboard();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative ?
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    this.getYaw()
                )
            :
                new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation
                )
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrivetrain.MAX_SPEED);

        for (SwerveModule mod : this.swerveModules) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void openLoopDrive (ChassisSpeeds speeds) {
        if (speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0) {
            stop();
            return;
        }
      
        SwerveModuleState[] swerveModuleStates = Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveDrivetrain.MAX_SPEED);
        setModuleStates(swerveModuleStates);
    }

    /* Gyro */
    public void zeroGyro() {
        this.gyro.setYaw(0);
    }

    private double optimizeGyro (double degrees) {
        // -180 < degrees < 180
        if ((degrees > 0.0) && (degrees < 360.0)) {
            return degrees - 180.0;
        } else {
            int m = (int) Math.floor( degrees / 360.0 );
            double optimizedDegrees = degrees - (m * 360.0);
            return Math.abs(optimizedDegrees) - 180.0;
        }
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        this.gyro.getYawPitchRoll(ypr);
        double yaw = ypr[0];
        // double yaw = optimizeGyro(ypr[0]);
        return Constants.SwerveDrivetrain.INVERT_GYRO ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }

    public void setGyro (double degrees) {
        gyro.setYaw(degrees);
    }

    public double getGyroAngleDegrees() {
        return this.getYaw().getDegrees();
    }

    public double getGyroAngleRadians() {
        return this.getYaw().getRadians();
    }

    /* Odometry */
    public Pose2d getPose () {
        return this.swerveOdometry.getPoseMeters();
    }

    public void resetOdometry (Pose2d pose) {
        this.swerveOdometry.resetPosition(pose, this.getYaw());
    }

    public void resetOdometryToProtectedZone () {
        resetOdometry(PROTECTED_ZONE);
    }

    private double getRadiusFromHub () {
        Pose2d currentPose = getPose();
        double x = currentPose.getX();
        double y = currentPose.getY();

        double a = x - HUB_CENTER.getX();
        double b = y - HUB_CENTER.getY();

        double radius = Math.sqrt( Math.pow(a, 2) + Math.pow(b, 2) );
        return radius;
    }

    public int getZone () {
        double radius = getRadiusFromHub();

        int zone;
        if (radius >= 10.0) {
            zone = 20;
        } else if (radius < 10.0 && radius >= 9.5) {
            zone = 19;
        } else if (radius < 9.5 && radius >= 9.0) {
            zone = 18;
        } else if (radius < 9.0 && radius >= 8.5) {
            zone = 17;
        } else if (radius < 8.5 && radius >= 8.0) {
            zone = 16;
        } else if (radius < 8.0 && radius >= 7.5) {
            zone = 15;
        } else if (radius < 7.5 && radius >= 7.0) {
            zone = 14;
        } else if (radius < 7.0 && radius >= 6.5) {
            zone = 13;
        } else if (radius < 6.5 && radius >= 6.0) {
            zone = 12;
        } else if (radius < 6.0 && radius >= 5.5) {
            zone = 11;
        } else if (radius < 5.5 && radius >= 5.0) {
            zone = 10;
        } else if (radius < 5.0 && radius >= 4.5) {
            zone = 9;
        } else if (radius < 4.5 && radius >= 4.0) {
            zone = 8;
        } else if (radius < 4.0 && radius >= 3.5) {
            zone = 7;
        } else if (radius < 3.5 && radius >= 3.0) {
            zone = 6;
        } else if (radius < 3.0 && radius >= 2.5) {
            zone = 5;
        } else if (radius < 2.5 && radius >= 2.0) {
            zone = 4;
        } else if (radius < 2.0 && radius >= 1.5) {
            zone = 3;
        } else if (radius < 1.5 && radius >= 1.0) {
            zone = 2;
        } else if (radius < 1.0 && radius >= 0.5) {
            zone = 1;
        } else {
            zone = 0;
        }

        return zone;
    }

    /* Module States */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : this.swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void setModuleStates (SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDrivetrain.MAX_SPEED);
        for (SwerveModule mod : this.swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void stop () {
        setModuleStates(Constants.SwerveDrivetrain.SWERVE_KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0)));
    }

    private double getSwerveModule0Degrees () {
        return this.swerveModules[0].getCanCoder().getDegrees();
    }

    private double getSwerveModule1Degrees () {
        return this.swerveModules[1].getCanCoder().getDegrees();
    }

    private double getSwerveModule2Degrees () {
        return this.swerveModules[2].getCanCoder().getDegrees();
    }

    private double getSwerveModule3Degrees () {
        return this.swerveModules[3].getCanCoder().getDegrees();
    }

    public void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(this);
        // tab.addNumber("Gyro Angle ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGyro);
        tab.addNumber("Gyro Angle (GRAPH) ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Mod 0 Encoder", this::getSwerveModule0Degrees);
        tab.addNumber("Mod 1 Encoder", this::getSwerveModule1Degrees);
        tab.addNumber("Mod 2 Encoder", this::getSwerveModule2Degrees);
        tab.addNumber("Mod 3 Encoder", this::getSwerveModule3Degrees);
        tab.addNumber("Pose X", () -> getPose().getX());
        tab.addNumber("Pose Y", () -> getPose().getY());
        tab.addNumber("Radius", () -> getRadiusFromHub());
        tab.addNumber("Zone", () -> getZone());
        SmartDashboard.putData(this.field);
    }

    @Override
    public void periodic() {
        this.swerveOdometry.update(this.getYaw(), this.getStates());
        this.field.setRobotPose(this.swerveOdometry.getPoseMeters());
    }
}
