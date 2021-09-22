package frc.lib.util;

public class SwerveModuleConstants {
    
    public final int driveMotorID;
    public final int angleMotorID;

    public final boolean driveMotorInverted;
    public final boolean angleMotorInverted;

    public final int cancoderID;
    public final double angleOffset;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, boolean driveMotorInverted, boolean angleMotorInverted, int canCoderID, double angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;

        this.driveMotorInverted = driveMotorInverted;
        this.angleMotorInverted = angleMotorInverted;

        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
