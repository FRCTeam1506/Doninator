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

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    // private final double DEFAULT_SPEED = 0.65; // 0.33

    private DoubleSolenoid stage1;
    private DoubleSolenoid stage2;
    // private TalonFX motor = new TalonFX(Constants.Intake.MOTOR_ID, "canivore");

    // private static final PneumaticState DEFAULT_PNEUMATIC_STATE = PneumaticState.High;

    public ArmSubsystem (PneumaticHub hub) {
        stage1 = hub.makeDoubleSolenoid(Constants.ArmSubsystem.SolenoidId1, Constants.ArmSubsystem.SolenoidId2);
        stage2 = hub.makeDoubleSolenoid(Constants.ArmSubsystem.SolenoidId3, Constants.ArmSubsystem.SolenoidId4);

        dashboard();
    }

    // private enum PneumaticState { Low, Mid, High }
    // private PneumaticState currentPneumaticState = DEFAULT_PNEUMATIC_STATE;

    // //setState not in use
    // private void setState (PneumaticState state) {
    //     if (currentPneumaticState != state) {
    //         currentPneumaticState = state;
    //         switch (state) {
    //             case Low:
    //                 stage1.set(Value.kReverse);
    //                 stage2.set(Value.kReverse);
    //                 break;

    //             case Mid:
    //                 stage1.set(Value.kForward);
    //                 stage2.set(Value.kReverse);
    //                 break;

    //             case High:
    //                 stage1.set(Value.kForward);
    //                 stage2.set(Value.kForward);
    //         }
    //     }
    // }
    // private String getStateName () { return currentPneumaticState.name(); }

    // public void extendStateMAX () { setState(PneumaticState.High); }
    // public void retractStateMAX () { setState(PneumaticState.Low); }
    // public void extendStateMID() { setState(PneumaticState.Mid); }

    public void setHigh () {      
        
        System.out.println("hello1");
        stage1.set(Value.kForward);
        stage2.set(Value.kForward);
        // stage1.set(Value.kOff);
        // stage2.set(Value.kOff);
    }
    public void setMid () {         
        System.out.println("hello2");
            
        stage1.set(Value.kForward);
        stage2.set(Value.kReverse);
        // stage1.set(Value.kOff);
        // stage2.set(Value.kOff);
    }
    public void setLow() {         
        System.out.println("hello3");
            
        stage1.set(Value.kReverse);
        stage2.set(Value.kReverse);
        // stage1.set(Value.kOff);
        // stage2.set(Value.kOff);
    }


    // public void progress () {

    //     if(getStateName().equals("Low")){
    //         setState(PneumaticState.Mid);
    //     }
    //     if(getStateName().equals("Mid")){
    //         setState(PneumaticState.High);
    //     }

    // }

    // public void regress () {

    //     if(getStateName().equals("High")){
    //         setState(PneumaticState.Mid);
    //     }
    //     if(getStateName().equals("Mid")){
    //         setState(PneumaticState.Low);
    //     }

    // }

    @Override
    public void periodic() {
        // System.out.println("hello");
        // refreshExtendoState();
        // refreshTriggerState();
        // refreshLeanboiState();
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.add(this);
        // tab.addString("Arm State", this::getStateName);

        // tab.addNumber("Left Motor Pos", leftMotor::getSelectedSensorPosition);
        // tab.addNumber("Right Motor Pos", rightMotor::getSelectedSensorPosition);
        // tab.addNumber("Pneumatic State", () -> currentPneumaticState);

    }

}