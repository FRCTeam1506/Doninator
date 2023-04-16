package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.drivetrain.SwerveTeleop;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TelescopingSubsystem extends SubsystemBase {

    private TalonFX motor = new TalonFX(Constants.TelescopingSubsystem.MOTOR_ID);
    DigitalInput input = new DigitalInput(Constants.TelescopingSubsystem.DIO_PORT);
    // Counter counter = new Counter(input);
    boolean click;

    double encoderCount = motor.getSelectedSensorPosition();
    double startingEncoderCount = encoderCount;
    double speed = 0.3;
    private static final double kP = 0.61; // 0.84
    private static final double kI = 0.00025;
    private static final double kD = 0;
    private static final double kF = 0.4;  // 0.4

    private static final double kVelocity = 40_000.0;       // 62_000.0
    private static final double kAcceleration = 30_000.0;   // 44_000.0

    private static final double MIN_POSITION = -1_500.0;
    private static final double MAX_POSITION = 230_000.0; // 200_000
    public static final double FIRST_RUNG_HEIGHT = 130_000.0; // 90_000
    public static final double ABOVE_RUNG_HEIGHT = 25_000.0; // 25_000
    public static final double FULL_EXTEND = MAX_POSITION;

    private double targetPosition = 0;
    

    public TelescopingSubsystem () {
        motor.configFactoryDefault();
        motor.setControlFramePeriod(ControlFrame.Control_3_General, 100);
        motor.setInverted(TalonFXInvertType.CounterClockwise);
        // resetMotors();
        //set motor to brake
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);

        motor.setInverted(TalonFXInvertType.CounterClockwise);
        encoderCount = motor.getSelectedSensorPosition();


        motor.config_kP(0, kP);
        motor.config_kI(0, kI);
        motor.config_kD(0, kD);
        motor.config_kF(0, kF);

        motor.configMotionCruiseVelocity(kVelocity);
        motor.configMotionAcceleration(kAcceleration);

        //current limit burn out the motor at states
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveDrivetrain.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.SwerveDrivetrain.DRIVE_CONTINUOUS_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CL, 
            Constants.SwerveDrivetrain.DRIVE_PEAK_CURRENT_DURATION);

        motor.configSupplyCurrentLimit(driveSupplyLimit);

        // click = input.get();

        // resetMotors();
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

    public void printStuff(){
        System.out.println("Starting encoder count: " + startingEncoderCount);
        System.out.println("Current encoder count: " + motor.getSelectedSensorPosition());
    }

    public void runHigh(){
        // targetPosition = 190000;
        motor.set(TalonFXControlMode.MotionMagic, 180000); //145k too low standish, 193k
    }

    public void runHighAuto(){
        targetPosition = 161290;
        motor.set(TalonFXControlMode.MotionMagic, 193000); // 142500 143800
    }

    public void runMid(){
        targetPosition = 87191;
        motor.set(TalonFXControlMode.MotionMagic, 80000); // 88332
    }

    // public void runHumanPlayer(){
    //     motor.set(TalonFXControlMode.MotionMagic, 30000);
    // }

    public void runHP(){
        motor.set(TalonFXControlMode.MotionMagic, 40000); //35000 post-states
    }


    public void runZero(){
        targetPosition = 1;
        motor.set(TalonFXControlMode.PercentOutput, -Constants.TelescopingSubsystem.DEFAULT_SPEED);
        System.out.println("000");
    }

    public void antiLukeFeature(){
        if(encoderCount<0){
            stop();
        }
        else if(encoderCount>235000){
            stop();
        }
    }

    public void testSwitch(){
        click = input.get();
        if(click == false){
            resetMotors();
            // System.out.println("CLICKED");
            motor.set(TalonFXControlMode.PercentOutput, 0);
            motor.set(TalonFXControlMode.MotionMagic, 2500);
            // stop();
        }
    }

    public void testRunZero(){
        motor.set(TalonFXControlMode.MotionMagic, 2500);
    }

    private void dashboard () {
        // ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        // tab.add(this);
        encoderCount = motor.getSelectedSensorPosition();
        SmartDashboard.putNumber("Telescope motor", encoderCount);
        SmartDashboard.putNumber("Telescope target", targetPosition);

        // tab.addString("XFactor State",this::getXFactorStateName);
    }

    public void periodic(){
        testSwitch();
        // antiLukeFeature();
        click = input.get();
        dashboard();

    }
    
}
