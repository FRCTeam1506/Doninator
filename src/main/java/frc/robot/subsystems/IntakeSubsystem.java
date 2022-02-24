package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final double DEFAULT_SPEED = 0.33;

    private enum XFactorState {
        RETRACTED,
        EXTENDED,
    }

    private XFactorState currentXFactorState;

    private TalonFX motor = new TalonFX(Constants.Intake.MOTOR_ID);
    private Solenoid xFactor = new Solenoid(PneumaticsModuleType.REVPH, Constants.Intake.XFACTOR_ID);

    public IntakeSubsystem () {
        motor.configFactoryDefault();
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        motor.setInverted(TalonFXInvertType.Clockwise);
        currentXFactorState = XFactorState.RETRACTED;
        dashboard();
    }

    public void intake () {
        motor.set(TalonFXControlMode.PercentOutput, DEFAULT_SPEED);
    }

    public void outtake () {
        motor.set(TalonFXControlMode.PercentOutput, -DEFAULT_SPEED);
    }

    public void stop () {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    private void setXFactorState (XFactorState state) {
        this.currentXFactorState = state;
        switch (state) {
            case RETRACTED:
                this.xFactor.set(true);
                break;

            case EXTENDED:
                this.xFactor.set(false);
                break;

            default:
                this.setXFactorState(XFactorState.RETRACTED);
                break;
        }
    }

    public void extend () {
        this.setXFactorState(XFactorState.EXTENDED);
    }

    public void retract () {
        this.setXFactorState(XFactorState.RETRACTED);
    }

    public XFactorState getXFactorState () {
        return this.currentXFactorState;
    }

    private String getXFactorStateName () {
        return this.currentXFactorState.name();
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.add(this);
        tab.addString("XFactor State",this::getXFactorStateName);
    }
    
}
