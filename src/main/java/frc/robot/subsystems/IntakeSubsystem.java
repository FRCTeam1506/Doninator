package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.math.Conversions;


import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    Conversions conversions = new Conversions();

    private DoubleSolenoid solenoid;
    private TalonFX motor = new TalonFX(Constants.IntakeSubsystem.MOTOR_ID);
    double encoderCount = motor.getSelectedSensorPosition();


    public IntakeSubsystem (PneumaticHub hub) {
        solenoid = hub.makeDoubleSolenoid(Constants.IntakeSubsystem.SolenoidId1, Constants.IntakeSubsystem.SolenoidId2);
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
        // System.out.println("Encoder: " + encoderCount);
    }

    public void intakeDefSpeed(){
        if(Constants.CandleSubsystem.cone == true){
            motor.set(TalonFXControlMode.PercentOutput, -Constants.IntakeSubsystem.CONE_DEFAULT_INTAKE_SPEED);
        }
        else{
            motor.set(TalonFXControlMode.PercentOutput, Constants.IntakeSubsystem.CUBE_DEFAULT_INTAKE_SPEED);
        }

    }

    public void intakeSlowSpeed(){
        if(Constants.CandleSubsystem.cone == true){
            motor.set(TalonFXControlMode.PercentOutput, 0.2);
        }
        else{
            motor.set(TalonFXControlMode.PercentOutput, -0.2);
        }

    }

    public void outtakeDefSpeed(){
        if(Constants.CandleSubsystem.cone == true){
            motor.set(TalonFXControlMode.PercentOutput, Constants.IntakeSubsystem.CONE_DEFAULT_OUTTAKE_SPEED);
        }
        else{
            motor.set(TalonFXControlMode.PercentOutput, -Constants.IntakeSubsystem.CUBE_DEFAULT_OUTTAKE_SPEED);
        }
    }

    public void pneumaticRetract(){
        solenoid.set(Value.kReverse);
    }

    public void pneumaticExtract(){
        solenoid.set(Value.kForward);
    }

    public void intakeRPM(double rpm){        
        //gearratio is second number, set it equal to 1 for now
        motor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(rpm/2, 1));
    }

    public void outtake () {
        motor.set(TalonFXControlMode.PercentOutput, -0.2); //-0.45
    }

    public void stop () {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    private void dashboard () {
        // ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        // tab.add(this);
        // tab.addString("XFactor State",this::getXFactorStateName);
    }

    
}