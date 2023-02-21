package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TelescopingSubsystem extends SubsystemBase {

    private TalonFX motor = new TalonFX(Constants.TelescopingSubsystem.MOTOR_ID);
    double encoderCount = motor.getSelectedSensorPosition();
    double startingEncoderCount = encoderCount;
    double speed = 0.3;


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
        motor.set(TalonFXControlMode.PercentOutput, -Constants.TelescopingSubsystem.DEFAULT_SPEED);
        encoderCount = motor.getSelectedSensorPosition();
    }

    public void backward () {
        motor.set(TalonFXControlMode.PercentOutput, Constants.TelescopingSubsystem.DEFAULT_SPEED);
        encoderCount = motor.getSelectedSensorPosition();
    }

    public void stop () {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
        encoderCount = motor.getSelectedSensorPosition();
    }

    public void setMid(){
        // motor.set(TalonFXControlMode.MotionMagic, -21719);
        // motor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(rpm/2, 1));

        System.out.println("hello hello hello");
    }
    public void setHigh(){
        motor.set(TalonFXControlMode.Position, -50000);
    }

    public void printStuff(){
        System.out.println("Starting encoder count: " + startingEncoderCount);
        System.out.println("Current encoder count: " + motor.getSelectedSensorPosition());
    }

    public void resetMotors(){
        motor.setSelectedSensorPosition(0.00);
        System.out.println("reset" + motor.getSelectedSensorPosition());
    }

    public void runHigh(){
        double c = motor.getSelectedSensorPosition();
        while(c<190000){
            motor.set(TalonFXControlMode.PercentOutput, 0.3);
            c = motor.getSelectedSensorPosition();
        }
        stop();
    }

    public void runMid(){
        double c = motor.getSelectedSensorPosition();
        int num = 87191;
        if(c>num){
            while(c>num){
                motor.set(TalonFXControlMode.PercentOutput, -0.3);
                c = motor.getSelectedSensorPosition();
            }
        }
        else if(c<num){
            while(c<num){
                motor.set(TalonFXControlMode.PercentOutput, 0.3);
                c = motor.getSelectedSensorPosition();
            }
        }
        stop();
    }

    public void runHP(){
        double c = motor.getSelectedSensorPosition();
        int num = 166587;
        if(c>num){
            while(c>num){
                motor.set(TalonFXControlMode.PercentOutput, -0.3);
                c = motor.getSelectedSensorPosition();
            }
        }
        else if(c<num){
            while(c<num){
                motor.set(TalonFXControlMode.PercentOutput, 0.3);
                c = motor.getSelectedSensorPosition();
            }
        }
        stop();
    }


    public void runZero(){
        double c = motor.getSelectedSensorPosition();
        while(c>0){
            motor.set(TalonFXControlMode.PercentOutput, -speed);
            c = motor.getSelectedSensorPosition();
        }
        stop();
    }



    public void testRun() {
        // runUntil(0.2);
        Timer.delay(0.50);            
        stop();
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.add(this);
        // tab.addString("XFactor State",this::getXFactorStateName);
    }
    
}
