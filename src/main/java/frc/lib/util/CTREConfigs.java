package frc.lib.util;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants;

public final class CTREConfigs {
    
    public TalonFXConfiguration swerveDriveTalonFXConfig;
    public TalonFXConfiguration swerveAngleTalonFXConfig;
    public CANCoderConfiguration swerveCANCoderConfig;

    public CTREConfigs () {
        this.swerveDriveTalonFXConfig   = new TalonFXConfiguration();
        this.swerveAngleTalonFXConfig   = new TalonFXConfiguration();
        this.swerveCANCoderConfig       = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.ANGLE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.ANGLE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.ANGLE_PEAK_CL, 
            Constants.SwerveDrivetrain.ANGLE_PEAK_CURRENT_DURATION);

        this.swerveAngleTalonFXConfig.slot0.kP = Constants.SwerveDrivetrain.ANGLE_kP;
        this.swerveAngleTalonFXConfig.slot0.kI = Constants.SwerveDrivetrain.ANGLE_kI;
        this.swerveAngleTalonFXConfig.slot0.kD = Constants.SwerveDrivetrain.ANGLE_kD;
        this.swerveAngleTalonFXConfig.slot0.kF = Constants.SwerveDrivetrain.ANGLE_kF;
        this.swerveAngleTalonFXConfig.supplyCurrLimit = angleSupplyLimit;
        this.swerveAngleTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        this.swerveDriveTalonFXConfig.slot0.kP = Constants.SwerveDrivetrain.DRIVE_kP;
        this.swerveDriveTalonFXConfig.slot0.kI = Constants.SwerveDrivetrain.DRIVE_kI;
        this.swerveDriveTalonFXConfig.slot0.kD = Constants.SwerveDrivetrain.DRIVE_kD;
        this.swerveDriveTalonFXConfig.slot0.kF = Constants.SwerveDrivetrain.DRIVE_kF;        
        this.swerveDriveTalonFXConfig.supplyCurrLimit = driveSupplyLimit;
        this.swerveDriveTalonFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        this.swerveDriveTalonFXConfig.openloopRamp = Constants.SwerveDrivetrain.OPEN_LOOP_RAMP;
        this.swerveDriveTalonFXConfig.closedloopRamp = Constants.SwerveDrivetrain.CLOSED_LOOP_RAMP;

        
        /* Swerve CANCoder Configuration */
        this.swerveCANCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        this.swerveCANCoderConfig.sensorDirection = Constants.SwerveDrivetrain.CAN_CODER_INVERTED;
        this.swerveCANCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        this.swerveCANCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

}
