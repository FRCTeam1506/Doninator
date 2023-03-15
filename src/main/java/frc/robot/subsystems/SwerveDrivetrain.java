package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
//import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants;
import frc.robot.utils.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveDrivetrain extends SubsystemBase {
    
    private SwerveDriveOdometry swerveOdometry;
    private SwerveModule[] swerveModules;
    private Pigeon2 gyro;
    private Field2d field;

    public SwerveDrivetrain() {
        this.gyro = new Pigeon2(Constants.SwerveDrivetrain.GYRO_ID);
        this.gyro.configFactoryDefault();
        this.zeroGyro();

        swerveModules = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveDrivetrain.Mod0.constants),
            new SwerveModule(1, Constants.SwerveDrivetrain.Mod1.constants),
            new SwerveModule(2, Constants.SwerveDrivetrain.Mod2.constants),
            new SwerveModule(3, Constants.SwerveDrivetrain.Mod3.constants)
        };


        this.swerveOdometry = new SwerveDriveOdometry(Constants.SwerveDrivetrain.SWERVE_KINEMATICS, this.getYaw(),
                                                    getModulePositions());

        for(SwerveModule mod : swerveModules){
            System.out.println("CANcoder on Module " + mod.moduleNumber + " took " + mod.CANcoderInitTime + " ms to be ready.");
        }
        

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


    public void driveSlow(){
        Constants.SwerveDrivetrain.MAX_SPEED = 1;
        Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY = 5.5;
        for(int i = 0; i<77777; i++){
            System.out.println("slow");
        }
    }

    public void driveNormal(){
        Constants.SwerveDrivetrain.MAX_SPEED = 5;
        Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY = 11.5;
    }

    // public void driveFast(){
    //     Constants.SwerveDrivetrain.MAX_SPEED = 5.5;
    //     Constants.SwerveDrivetrain.MAX_ANGULAR_VELOCITY = 11.5;
    // }



    /* Gyro */
    public void zeroGyro() {
        this.gyro.setYaw(0);
        
    }

    private double optimizeGyro (double degrees) {
        // 0 < degrees < 360
        if ((degrees > 0.0) && (degrees < 360.0)) {
            return degrees;
        } else {
            int m = (int) Math.floor( degrees / 360.0 );
            double optimizedDegrees = degrees - (m * 360.0);
            return Math.abs(optimizedDegrees);
        }
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        this.gyro.getYawPitchRoll(ypr);
        double yaw = optimizeGyro(ypr[0]);
        return Constants.SwerveDrivetrain.INVERT_GYRO ? Rotation2d.fromDegrees(360 - yaw) : Rotation2d.fromDegrees(yaw);
    }

    public double getGyroAngleDegrees() {
        return this.getYaw().getDegrees();
    }

    public double getGyroAngleRadians() {
        return this.getYaw().getRadians();
    }

    public double getGyroPitchDegrees() {
        //change this next year pigeon 2 Pitch axis is Y
        return this.gyro.getRoll() ;
    }

    public double getGyroRoll() {
        return this.gyro.getRoll();
    }


    /* Odometry */
    public Pose2d getPose() {
        return this.swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        this.swerveOdometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose) {
        this.swerveOdometry.resetPosition(this.getYaw(), getModulePositions() , pose);
    }

    /* Module States */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : this.swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveModules){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveDrivetrain.MAX_SPEED);
        for (SwerveModule mod : this.swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /*
    * Can be mapped to button to reset mods to abs.
    * (or put on dash as button)
    */
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : swerveModules){
            mod.resetToAbsolute();
        }
    }

    public void dashboard() {
         ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        // tab.add(this);
        // tab.addNumber("Gyro Angle ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGyro);
        // tab.addNumber("Gyro Angle (GRAPH) ???", this::getGyroAngleDegrees).withWidget(BuiltInWidgets.kGraph);
        tab.addNumber("Gyro Pitch", this::getGyroPitchDegrees).withWidget(BuiltInWidgets.kGyro);
        tab.addNumber("Gyro Pitch (GRAPH)", this::getGyroPitchDegrees).withWidget(BuiltInWidgets.kGraph);
        // SmartDashboard.putData(this.field);
        // SmartDashboard.putData("ANGLE PID", data);
        // SmartDashboard.putData("DRIVE PID", data);


    }

    @Override
    public void periodic() {
        this.swerveOdometry.update(this.getYaw(), getModulePositions());
        this.field.setRobotPose(this.swerveOdometry.getPoseMeters());
        
    }
}
