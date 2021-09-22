package frc.lib.util;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

import frc.lib.math.Conversions;

import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;


public class SwerveModule {
    public int moduleNumber;

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private boolean driveMotorInverted;
    private boolean angleMotorInverted;
    private CANCoder angleEncoder;
    private double angleOffset;

    private double lastAngle;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveDrivetrain.FF_kS, Constants.SwerveDrivetrain.FF_kV, Constants.SwerveDrivetrain.FF_kA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;

        /* Drive Motor Config */
        this.driveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();
        this.driveMotorInverted = moduleConstants.driveMotorInverted;

        /* Angle Motor Config */
        this.angleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();
        this.angleMotorInverted = moduleConstants.angleMotorInverted;
        
        /* Angle Encoder Config */
        this.angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        this.angleOffset = moduleConstants.angleOffset;

        this.lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not

        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveDrivetrain.MAX_SPEED;
            this.driveMotor.set(ControlMode.PercentOutput, percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
            this.driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveDrivetrain.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle.getDegrees();   // Prevent rotating module if speed is less then 1%. Prevents Jittering.
        this.angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO)); 
        this.lastAngle = angle;
    }

    private void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset, Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO);
        this.angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCANCoderConfig);
    }

    private void configAngleMotor(){
        this.angleMotor.configFactoryDefault();
        this.angleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleTalonFXConfig);
        this.angleMotor.setInverted(this.angleMotorInverted);
        this.angleMotor.setNeutralMode(Constants.SwerveDrivetrain.ANGLE_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        this.driveMotor.configFactoryDefault();
        this.driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveTalonFXConfig);
        this.driveMotor.setInverted(this.driveMotorInverted);
        this.driveMotor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        this.driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState(){
        double velocity = Conversions.falconToMPS(this.driveMotor.getSelectedSensorVelocity(), Constants.SwerveDrivetrain.WHEEL_CIRCUMFERENCE, Constants.SwerveDrivetrain.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(this.angleMotor.getSelectedSensorPosition(), Constants.SwerveDrivetrain.ANGLE_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);
    }
    
}