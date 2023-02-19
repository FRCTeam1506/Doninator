package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopingSubsystem extends SubsystemBase {

    private final double DEFAULT_SPEED = 0.3; // 0.33 //0.65

    private TalonFX motor = new TalonFX(Constants.TelescopingSubsystem.MOTOR_ID);
    double encoderCount = motor.getSelectedSensorPosition();
    double startingEncoderCount = encoderCount;

    public TelescopingSubsystem () {
        motor.configFactoryDefault();
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        resetMotors();
        //set motor to brake
        this.motor.setNeutralMode(Constants.SwerveDrivetrain.DRIVE_NEUTRAL_MODE);
        dashboard();
    }

    public void forward () {
        motor.set(TalonFXControlMode.PercentOutput, DEFAULT_SPEED);
        encoderCount = motor.getSelectedSensorPosition();
    }

    public void backward () {
        motor.set(TalonFXControlMode.PercentOutput, -DEFAULT_SPEED);
        encoderCount = motor.getSelectedSensorPosition();
    }

    public void stop () {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
        encoderCount = motor.getSelectedSensorPosition();
    }

    public void setMid(){
        // motor.set(TalonFXControlMode.MotionMagic, -21719);
        motor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(rpm/2, 1));

        System.out.println("hello hello hello");
    }
    public void setHigh(){
        motor.set(TalonFXControlMode.Position, -50000);
    }

    public void printStuff(){
        System.out.println("Starting encoder count: " + startingEncoderCount);
        System.out.println("Current encoder count: " + encoderCount);
    }

    public void resetMotors(){
        motor.setSelectedSensorPosition(0.00);
        System.out.println("reset" + motor.getSelectedSensorPosition());
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.add(this);
        // tab.addString("XFactor State",this::getXFactorStateName);
    }
    
}
