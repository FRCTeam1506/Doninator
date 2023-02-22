package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    private static final double kP = 0.61; // 0.84
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.4;  // 0.4

    private static final double kVelocity = 40_000.0;       // 62_000.0
    private static final double kAcceleration = 30_000.0;   // 44_000.0

    private static final double MIN_POSITION = -1_500.0;
    private static final double MAX_POSITION = 230_000.0; // 200_000
    public static final double FIRST_RUNG_HEIGHT = 130_000.0; // 90_000
    public static final double ABOVE_RUNG_HEIGHT = 25_000.0; // 25_000
    public static final double FULL_EXTEND = MAX_POSITION;


    public TelescopingSubsystem () {
        motor.configFactoryDefault();
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        resetMotors();
        //set motor to brake
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);

        motor.setInverted(TalonFXInvertType.CounterClockwise);


        motor.config_kP(0, kP);
        motor.config_kI(0, kI);
        motor.config_kD(0, kD);
        motor.config_kF(0, kF);

        motor.configMotionCruiseVelocity(kVelocity);
        motor.configMotionAcceleration(kAcceleration);

        resetMotors();
        dashboard();
    }

        /* 
    * ####################### 
    *   Motor Functions
    * #######################
    */
    public void resetMotors () {
        motor.setSelectedSensorPosition(0.0);
        System.out.println("reset" + motor.getSelectedSensorPosition());
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


   //     while(c<190000){
    public void runHigh(){
        motor.set(TalonFXControlMode.MotionMagic, 190000);
    }

    public void runMid(){
        motor.set(TalonFXControlMode.MotionMagic, 87191);
    }

    public void runHP(){
        motor.set(TalonFXControlMode.MotionMagic, 166587);
    }


    public void runZero(){
        motor.set(TalonFXControlMode.MotionMagic, 0);
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
