// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Units;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double JOYSTICK_DEADBAND = 0.1;

    public static final class SwerveDrivetrain {

        /* Gyro */
        public static final int GYRO_ID = 13;
        public static final boolean INVERT_GYRO = false;

        /* Drivetrain */
        public static final double TRACK_WIDTH          = Units.inchesToMeters(15.125);
        public static final double WHEEL_BASE           = Units.inchesToMeters(15.125);
        public static final double WHEEL_DIAMETER       = Units.inchesToMeters(3.58);
        public static final double WHEEL_CIRCUMFERENCE  = WHEEL_DIAMETER * Math.PI;

        public static final double OPEN_LOOP_RAMP   = 0.25;
        public static final double CLOSED_LOOP_RAMP = 0.0;

        public static final double DRIVE_GEAR_RATIO = (6.0 / 1.0);  // 6.86:1
        public static final double ANGLE_GEAR_RATIO = (12.0 / 1.0); // 12.8:1

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
        public static final double DRIVE_kP = 0.10;  // 0.10
        public static final double DRIVE_kI = 0.0;   // 0.0
        public static final double DRIVE_kD = 0.0;   // 0.0
        public static final double DRIVE_kF = 0.0;   // 0.0

        /* Drive Motor Characterization Values (FeedForward) */
        public static final double FF_kS    = (0.632 / 12);     // 0.667 --- divide by 12 to convert from volts to percent output for CTRE
        public static final double FF_kV    = (0.0514 / 12);    // 2.44
        public static final double FF_kA    = (0.00337 / 12);   // 0.27

        /* Swerve Profiling Values */
        public static final double MAX_SPEED            = 4.5; //meters per second
        public static final double MAX_ANGULAR_VELOCITY = 11.5;

        /* Neutral Modes */
        public static final NeutralMode ANGLE_NEUTRAL_MODE = NeutralMode.Coast;
        public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean ANGLE_MOTOR_INVERTED = true;

        /* Angle Encoder Invert */
        public static final boolean CAN_CODER_INVERTED = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID  = 1;
            public static final int ANGLE_MOTOR_ID  = 2;
            public static final boolean DRIVE_MOTOR_INVERTED = false;
            public static final boolean ANGLE_MOTOR_INVERTED = true;
            public static final int CAN_CODER_ID    = 3;
            public static final double ANGLE_OFFSET = 60.0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, DRIVE_MOTOR_INVERTED, ANGLE_MOTOR_INVERTED, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID  = 5;
            public static final int ANGLE_MOTOR_ID  = 4;
            public static final boolean DRIVE_MOTOR_INVERTED = false;
            public static final boolean ANGLE_MOTOR_INVERTED = true;
            public static final int CAN_CODER_ID    = 6;
            public static final double ANGLE_OFFSET = 138.3;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, DRIVE_MOTOR_INVERTED, ANGLE_MOTOR_INVERTED, CAN_CODER_ID, ANGLE_OFFSET);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID  = 11;
            public static final int ANGLE_MOTOR_ID  = 10;
            public static final boolean DRIVE_MOTOR_INVERTED = false;
            public static final boolean ANGLE_MOTOR_INVERTED = true;
            public static final int CAN_CODER_ID    = 12;
            public static final double ANGLE_OFFSET = 214.5;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, DRIVE_MOTOR_INVERTED, ANGLE_MOTOR_INVERTED, CAN_CODER_ID, ANGLE_OFFSET);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID  = 7;
            public static final int ANGLE_MOTOR_ID  = 9;
            public static final boolean DRIVE_MOTOR_INVERTED = false;
            public static final boolean ANGLE_MOTOR_INVERTED = true;
            public static final int CAN_CODER_ID    = 8;
            public static final double ANGLE_OFFSET = 88.0;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, DRIVE_MOTOR_INVERTED, ANGLE_MOTOR_INVERTED, CAN_CODER_ID, ANGLE_OFFSET);
        }
    }

    public static final class Auton {
        public static final double MAX_SPEED_MPS            = 3;    // meters per second
        public static final double MAX_ACCELERATION_MPSS    = 3;    // meters per second squared

        public static final double MAX_ANGULAR_SPEED_RPS    = Math.PI;      // radians per second
        public static final double MAX_ANGULAR_SPEED_RPSS   = Math.PI;      // radians per second squared

        public static final double PX_CONTROLLER        = 1.0;
        public static final double PY_CONTROLLER        = 1.0;
        public static final double PTHETA_CONTROLLER    = 1.0;

        public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RPS, MAX_ANGULAR_SPEED_RPSS
        );
    }

    public static final class Playstation {
        
        // Driver Controls
        public static final Integer USBID = 0;

        // Axis
        public static final Integer LeftXAxis = 0;
        public static final Integer LeftYAxis = 1;
        public static final Integer RightXAxis = 2;
        public static final Integer RightYAxis = 5;

        // Trigger
        public static final Integer LeftTrigger = 3;
        public static final Integer RightTrigger = 4;

        // Bumper
        public static final Integer LeftBumper = 5;
        public static final Integer RightBumper = 6;

        // Buttons
        public static final Integer SquareButton = 1;
        public static final Integer XButton = 2;
        public static final Integer CircleButton = 3;
        public static final Integer TriangleButton = 4;

        public static final Integer LeftTriggerButton = 7;
        public static final Integer RightTriggerButton = 8;

        public static final Integer LeftButton = 9;
        public static final Integer RightButton = 10;

        public static final Integer LeftJoystickButton = 11;
        public static final Integer RightJoystickButton = 12;
        public static final Integer MiddleButton = 13;
        public static final Integer BigButton = 14;

        // POV Button
        public static final Integer NorthPOVButton = 0;
        public static final Integer NorthEastPOVButton = 45;
        public static final Integer EastPOVButton = 90;
        public static final Integer SouthEastPOVButton = 135;
        public static final Integer SouthPOVButton = 180;
        public static final Integer SouthWestPOVButton = 225;
        public static final Integer WestPOVButton = 270;
        public static final Integer NorthWestPOVButton = 315;
    }

}
