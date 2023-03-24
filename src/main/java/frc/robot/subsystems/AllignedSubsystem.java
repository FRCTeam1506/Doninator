package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.subsystems.*;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AllignedSubsystem extends SubsystemBase {

    // private final double DEFAULT_SPEED = 0.65; // 0.33

    // private TalonFX motor = new TalonFX(Constants.Intake.MOTOR_ID, "canivore");

    // private static final PneumaticState DEFAULT_PNEUMATIC_STATE = PneumaticState.High;

    ArmSubsystem arm;
    TelescopingSubsystem telescope;
    IntakeSubsystem intake;

    public AllignedSubsystem(ArmSubsystem arm, TelescopingSubsystem telescope, IntakeSubsystem intake){
        this.arm = arm;
        this.telescope = telescope;
        this.intake = intake;
    }

    public void ground(){
        intake.pneumaticRetract();
        arm.setLow();
        telescope.runZero();
    }

    public void mid(){
        if(Constants.CandleSubsystem.cone){
            intake.pneumaticExtract();
        }
        else{
            intake.pneumaticRetract();
        }

        arm.setMid();
        telescope.runMid();
    }

    public void high(){
        if(Constants.CandleSubsystem.cone){
            intake.pneumaticExtract();
        }
        else{
            intake.pneumaticRetract();
        }

        arm.setMid();
        telescope.runHigh();
    }

    public void transport(){
        intake.pneumaticRetract();
        arm.setHigh();
        telescope.runZero();
    }

    public void HP(){
        intake.pneumaticRetract();
        arm.setMid();
        telescope.runZero();
    }

    public void stop(){
        telescope.stop();
    }

    @Override
    public void periodic() {
        // System.out.println("hello");

    }

    private void dashboard () {
        //ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        //tab.add(this);
        
        
    }

}