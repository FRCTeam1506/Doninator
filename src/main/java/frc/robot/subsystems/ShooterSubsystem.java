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

    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.188;

    private static final double IDLE_RPM = 0.0; // 2000.0 1000.0

    private TalonFX leftMotor = new TalonFX(Constants.Shooter.LEFT_MOTOR_ID, "canivore");
    private TalonFX rightMotor = new TalonFX(Constants.Shooter.RIGHT_MOTOR_ID, "canivore");
    
    // TODO: Remove after testing
    private NetworkTableEntry velocity_dash;
    private double targetVelocity = 500.0;

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

    private void setShooterVelocity (double velocity_rpm) {
        leftMotor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(velocity_rpm, 1.5));
        rightMotor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(velocity_rpm, 1.5));
    }

    public void shoot (double velocity_rpm) {
        // setShooterVelocity(this.velocity_dash.getDouble(velocity_rpm));
        this.targetVelocity = velocity_rpm;
        setShooterVelocity(this.targetVelocity);
    }

    public boolean isShooterReady () {
        if ( rightMotor.getSelectedSensorVelocity() >= this.targetVelocity - 200.0 && rightMotor.getSelectedSensorVelocity() <= this.targetVelocity + 200.0 ) {
            return true;
        }
        return false;
    }

    public void idle () {
        setShooterVelocity(IDLE_RPM);
    }

    public double getVelocity () {
        return Conversions.falconToRPM(rightMotor.getSelectedSensorVelocity(), 1.5);
    }

    public void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.add(this);
        tab.addNumber("Velocity (RPM)", this::getVelocity);
        tab.addNumber("Velocity Graph (RPM)", this::getVelocity).withWidget(BuiltInWidgets.kGraph);
        
        this.velocity_dash = tab.add("Set Velocity (RPM)", this.targetVelocity)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 200, "max", 1000, "blockIncrement", 500))
            .getEntry();
    }
}
