package frc.robot.subsystems;

import java.util.Map;

import frc.lib.math.Conversions;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private TalonFX shooter = new TalonFX(Constants.Shooter.SHOOTER_ID);
    
    // TODO: Remove after testing
    private NetworkTableEntry velocity_dash;
    private double velocity_rpm = 1000.0;

    public ShooterSubsystem () {
        shooter.configFactoryDefault();
        shooter.setInverted(TalonFXInvertType.CounterClockwise);

        shooter.config_kP(0, 0.2);
        shooter.config_kI(0, 0.0);
        shooter.config_kD(0, 0.0);
        shooter.config_kF(0, 0.05);

        dashboard();
    }

    public void shoot (double velocity_rpm) {
        // shooter.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(this.velocity_dash.getDouble(this.velocity_rpm), 1.0));
    }

    public double getVelocity () {
        return Conversions.falconToRPM(shooter.getSelectedSensorVelocity(), 1.0);
    }

    public void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        tab.addNumber("Velocity (RPM)", this::getVelocity);
        tab.addNumber("Velocity Graph (RPM)", this::getVelocity).withWidget(BuiltInWidgets.kGraph);
    

    }
}
