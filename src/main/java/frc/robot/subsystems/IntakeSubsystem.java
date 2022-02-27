package frc.robot.subsystems;

import frc.robot.Constants;

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

    private final double DEFAULT_SPEED = 0.33;

    private enum XFactorState { RETRACTED, EXTENDED }
    private XFactorState currentXFactorState = XFactorState.RETRACTED;

    private Solenoid xFactor;
    private TalonFX motor = new TalonFX(Constants.Intake.MOTOR_ID, "canivore");

    public IntakeSubsystem (PneumaticHub hub) {
        xFactor = hub.makeSolenoid(Constants.Intake.XFACTOR_ID);
        motor.configFactoryDefault();
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        motor.setInverted(TalonFXInvertType.Clockwise);
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
                this.xFactor.set(false);
                break;

            case EXTENDED:
                this.xFactor.set(true);
                break;
        }
    }
    private String getXFactorStateName () { return currentXFactorState.name(); }

    public void extend () { this.setXFactorState(XFactorState.EXTENDED); }
    public void retract () { this.setXFactorState(XFactorState.RETRACTED); }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.add(this);
        tab.addString("XFactor State",this::getXFactorStateName);
    }
    
}
