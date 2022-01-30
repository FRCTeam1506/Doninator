package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.utils.swerve.SwerveModule;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveDrivetrain extends SubsystemBase {
    
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] swerveModules;
    private PigeonIMU gyro;

    public SwerveDrivetrain() {
        this.gyro = new PigeonIMU(Constants.SwerveDrivetrain.GYRO_ID);
        this.gyro.configFactoryDefault();
        this.zeroGyro();

        this.swerveOdometry = new SwerveDriveOdometry(Constants.SwerveDrivetrain.SWERVE_KINEMATICS, this.getYaw());

        this.swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveDrivetrain.Mod0.constants),
            new SwerveModule(1, Constants.SwerveDrivetrain.Mod1.constants),
            new SwerveModule(2, Constants.SwerveDrivetrain.Mod2.constants),
            new SwerveModule(3, Constants.SwerveDrivetrain.Mod3.constants)
        };
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

    /* Gyro */
    public void zeroGyro() {
        this.gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        this.gyro.getYawPitchRoll(ypr);
        return Constants.SwerveDrivetrain.INVERT_GYRO ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }

    public double getGyroAngleDegrees() {
        return this.getYaw().getDegrees();
    }

    public double getGyroAngleRadians() {
        return this.getYaw().getRadians();
    }

    /* Odometry */
    public Pose2d getPose() {
        return this.swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        this.swerveOdometry.resetPosition(pose, this.getYaw());
    }

    /* Module States */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : this.swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDrivetrain.MAX_SPEED);
        for (SwerveModule mod : this.swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void dashboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(this);
        tab.addNumber("Gyro Angle ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGyro);
        tab.addNumber("Gyro Angle (GRAPH) ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGraph);
        
        // SmartDashboard.putData("ANGLE PID", data);
        // SmartDashboard.putData("DRIVE PID", data);
    }

    @Override
    public void periodic() {
        this.swerveOdometry.update(this.getYaw(), this.getStates());
    }
}
