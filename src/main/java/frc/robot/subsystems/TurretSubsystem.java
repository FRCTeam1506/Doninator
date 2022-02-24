package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    private TalonFX motor = new TalonFX(Constants.Turret.MOTOR_ID);
    
    public TurretSubsystem () {
        this.motor.configFactoryDefault();

        // this.motor.setNeutralMode(NeutralMode.Brake);
        this.motor.setNeutralMode(NeutralMode.Coast);

        //? limit switch "hard stops"
        this.motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        this.motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        //? pid for motion magic
        this.motor.config_kP(0, 0.01);
        this.motor.config_kI(0, 0.0);
        this.motor.config_kD(0, 0.0);
        this.motor.config_kF(0, 0.1); 

        //? velocity and acceleration for motion magic
        this.motor.configMotionCruiseVelocity(1000.0);
        this.motor.configMotionAcceleration(500.0);

        this.reset();
        dashboard();
    }

    private void reset () {
        this.motor.setSelectedSensorPosition(0.0);
    }

    private double getSensorPosition () {
        return this.motor.getSelectedSensorPosition(); // 25000
    }

    public void stop () {
        this.motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void setPosition (double pos) {
        if (Math.abs(pos) > 25000.0) this.motor.set(TalonFXControlMode.MotionMagic, 25000.0);
        else this.motor.set(TalonFXControlMode.MotionMagic, pos);
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Turret");
        tab.add(this);
        tab.addNumber("Sensor Data", this::getSensorPosition);
    }

}
