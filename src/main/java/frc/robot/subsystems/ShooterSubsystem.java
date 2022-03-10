package frc.robot.subsystems;

import java.util.Map;

import frc.lib.math.Conversions;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private static final double kP = 0.17;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.05; // 0.188 0.673

    private static final double GEAR_RATIO = 1.5;
    private static final double VELOCITY_DELTA = 50.0;

    private static final double IDLE_RPM = 800.0; // 2000.0 1000.0
    private static final double MAX_RPM = 2450.0;
    private static final double MAX_SENSOR_VELOCITY = Conversions.RPMToFalcon(MAX_RPM, GEAR_RATIO);

    private TalonFX leftMotor = new TalonFX(Constants.Shooter.LEFT_MOTOR_ID, "canivore");
    private TalonFX rightMotor = new TalonFX(Constants.Shooter.RIGHT_MOTOR_ID, "canivore");
    
    // TODO: Remove after testing
    private NetworkTableEntry velocity_dash;
    private double targetVelocityRPM = IDLE_RPM;

    public ShooterSubsystem (PneumaticHub hub) {
        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        leftMotor.setNeutralMode(NeutralMode.Coast);
        rightMotor.setNeutralMode(NeutralMode.Coast);

        leftMotor.setInverted(TalonFXInvertType.Clockwise);
        rightMotor.setInverted(TalonFXInvertType.CounterClockwise);

        leftMotor.follow(rightMotor);

        rightMotor.config_kP(0, kP);
        rightMotor.config_kI(0, kI);
        rightMotor.config_kD(0, kD);
        rightMotor.config_kF(0, kF);
        rightMotor.configClosedloopRamp(0.5);

        dashboard();
    }

    private void setVelocity (double velocity_rpm) {
        rightMotor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(velocity_rpm, GEAR_RATIO));
    }

    private double getVelocity () {
        return Conversions.falconToRPM(rightMotor.getSelectedSensorVelocity(), GEAR_RATIO);
    }

    public void shoot (double velocity_rpm) {
        this.targetVelocityRPM = velocity_rpm;

        if (velocity_rpm >= 0 && velocity_rpm <= MAX_RPM) {
            setVelocity(this.targetVelocityRPM);
        } else if (velocity_rpm < 0) {
            setVelocity(0.0);
        } else if (velocity_rpm > MAX_RPM) {
            setVelocity(MAX_RPM);
        }

        // setVelocity(this.velocity_dash.getDouble(this.targetVelocityRPM));
    }

    public void idle () { setVelocity(IDLE_RPM); }

    public boolean isShooterReady () {
        double actual_velocity_rpm = getVelocity();

        //? target-delta <= actual <= target+delta
        if ( actual_velocity_rpm >= this.targetVelocityRPM - VELOCITY_DELTA && actual_velocity_rpm <= this.targetVelocityRPM + VELOCITY_DELTA ) {
            return true;
        } else { return false; }
    }

    public boolean isShooterReady2 () {
        double actual_velocity_rpm = getVelocity();
        
        if (velocity_dash != null) {
            //? target-delta <= actual <= target+delta
            if ( actual_velocity_rpm >= this.velocity_dash.getDouble(0.0) - VELOCITY_DELTA && actual_velocity_rpm <= this.velocity_dash.getDouble(0.0) + VELOCITY_DELTA ) {
                return true;
            } else { return false; }
        } else {
            return false;
        }
    }

    

    @Override
    public void periodic() {
        // System.out.println(getVelocity());
    }

    public void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.add(this);
        tab.addNumber("Velocity (RPM)", this::getVelocity);
        tab.addNumber("Velocity Graph (RPM)", this::getVelocity).withWidget(BuiltInWidgets.kGraph);
        tab.addBoolean("Shooter Ready", this::isShooterReady);
        
        this.velocity_dash = tab.add("Set Velocity (RPM)", this.targetVelocityRPM)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 500, "max", 2500, "blockIncrement", 50))
            .getEntry();

        // low goal: 760.0
        // high goal: 1750.0
    }
}
