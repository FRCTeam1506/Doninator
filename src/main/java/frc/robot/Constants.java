// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.swerve.SwerveModuleConstants;

public final class Constants {
    public static final class SwerveDrivetrain {

        /* Gyro */
        public static final int GYRO_ID = 10;
        public static final boolean INVERT_GYRO = false; // false

        /* Drivetrain */
        public static final double TRACK_WIDTH          = Units.inchesToMeters(21.25);
        public static final double WHEEL_BASE           = Units.inchesToMeters(21.25);
        public static final double WHEEL_DIAMETER       = Units.inchesToMeters(3.88); // 3.58
        public static final double WHEEL_CIRCUMFERENCE  = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP   = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = 6.538;
        public static final double ANGLE_GEAR_RATIO = 15.428;

        public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(  WHEEL_BASE / 2.0,   TRACK_WIDTH / 2.0),
            new Translation2d(  WHEEL_BASE / 2.0,  -TRACK_WIDTH / 2.0),
            new Translation2d( -WHEEL_BASE / 2.0,   TRACK_WIDTH / 2.0),
            new Translation2d( -WHEEL_BASE / 2.0,  -TRACK_WIDTH / 2.0)
        );

        /* Current Limiting */
        public static final int ANGLE_CONTINUOUS_CL = 25;
        public static final int ANGLE_PEAK_CL       = 40;
        public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;

        public static final int DRIVE_CONTINUOUS_CL = 35;
        public static final int DRIVE_PEAK_CL       = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        /* Angle Motor PID Values */
        public static final double ANGLE_kP = 0.6;   // 0.6
        public static final double ANGLE_kI = 0.0;   // 0.0
        public static final double ANGLE_kD = 12.0;  // 12.0
        public static final double ANGLE_kF = 0.0;   // 0.0

        /* Drive Motor PID Values */
        public static final double DRIVE_kP = 0.1;   // 0.10
        public static final double DRIVE_kI = 0.0;   // 0.0
        public static final double DRIVE_kD = 0.0;   // 0.0
        public static final double DRIVE_kF = 0.0;   // 0.0

        /* Drive Motor Characterization Values (FeedForward) */
        public static final double FF_kS    = (0.632 / 12);     // 0.667 --- divide by 12 to convert from volts to percent output for CTRE
        public static final double FF_kV    = (0.0514 / 12);    // 2.44
        public static final double FF_kA    = (0.00337 / 12);   // 0.27

        /* Swerve Profiling Values */
        public static final double MAX_SPEED            = 4.5;  // m/s
        public static final double MAX_ANGULAR_VELOCITY = 11.5; // m/s

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = true;
        public static final boolean ANGLE_MOTOR_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERTED = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int CAN_CODER_ID    = 10;
            public static final int ANGLE_MOTOR_ID  = 11;
            public static final int DRIVE_MOTOR_ID  = 12;
            public static final double ANGLE_OFFSET = 145.986;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int CAN_CODER_ID    = 20;
            public static final int ANGLE_MOTOR_ID  = 21;
            public static final int DRIVE_MOTOR_ID  = 22;
            public static final double ANGLE_OFFSET = 237.041;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int CAN_CODER_ID    = 30;
            public static final int ANGLE_MOTOR_ID  = 31;
            public static final int DRIVE_MOTOR_ID  = 32;
            public static final double ANGLE_OFFSET = 26.455;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int CAN_CODER_ID    = 40;
            public static final int ANGLE_MOTOR_ID  = 41;
            public static final int DRIVE_MOTOR_ID  = 42;
            public static final double ANGLE_OFFSET = 111.973;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class Shooter {
        public static final int LEFT_MOTOR_ID = 50;
        public static final int RIGHT_MOTOR_ID = 51;
    }

    public static final class Intake {
        public static final int MOTOR_ID = 52;
        public static final int XFACTOR_ID = 10; // 8
    }

    public static final class Indexer {
        public static final int TOP_MOTOR_ID = 53;
        public static final int BOTTOM_MOTOR_ID = 54;
    }

    public static final class Climber {
        public static final int LEFT_MOTOR_ID = 61;
        public static final int RIGHT_MOTOR_ID = 62;

        public static final int LEFT_LEANBOI_ID = 8;
        public static final int RIGHT_LEANBOI_ID = 9;
    }

    public static final class Turret {
        public static final int HOOD_ID = 11; // 15
        public static final int MOTOR_ID = 55;
        public static final double AUTO_kP = 0.05;  // 0.023
        public static final double AUTO_kI = 0.005; // 0.005
        public static final double AUTO_kD = 0.0;   // 0.0
    }

    public static final class Auton {
        public static final double MAX_SPEED_MPS            = 1.0;    // meters per second          7.0
        public static final double MAX_ACCELERATION_MPSS    = 1.0;    // meters per second squared  5.0

        public static final double MAX_ANGULAR_SPEED_RPS    = 2 * Math.PI;      // radians per second
        public static final double MAX_ANGULAR_SPEED_RPSS   = 2 * Math.PI;      // radians per second squared

        public static final double PX_CONTROLLER = 6.0;
        public static final double PY_CONTROLLER = 6.0;
        public static final double PTHETA_CONTROLLER = 7.0;
        
        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPSS);
    }

}
