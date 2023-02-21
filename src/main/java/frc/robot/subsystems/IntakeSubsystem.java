package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.math.Conversions;


import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final double DEFAULT_SPEED = 0.65; // 0.33
    Conversions conversions = new Conversions();

    private TalonFX motor = new TalonFX(Constants.IntakeSubsystem.MOTOR_ID);
    double encoderCount = motor.getSelectedSensorPosition();


    public IntakeSubsystem () {
        motor.configFactoryDefault();
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        //set motor to brake
        this.motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        dashboard();
    }

    public void intake (double x) {
        //previously x was defaultSpeed
        motor.set(TalonFXControlMode.PercentOutput, x);
        System.out.println("Encoder: " + encoderCount);
    }

    public void intakeDefSpeed(){
        motor.set(TalonFXControlMode.PercentOutput, DEFAULT_SPEED);
    }

    public void outtakeDefSpeed(){
        motor.set(TalonFXControlMode.PercentOutput, -DEFAULT_SPEED);
    }

    public void intakeRPM(double rpm){        
        //gearratio is second number, set it equal to 1 for now
        motor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(rpm/2, 1));
    }

    public void outtake () {
        motor.set(TalonFXControlMode.PercentOutput, -0.45);
    }

    public void stop () {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.add(this);
        // tab.addString("XFactor State",this::getXFactorStateName);
    }

    
}