package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private static final double kP = 0.61; // 0.84
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.4;  // 0.4

    private static final double kVelocity = 40_000.0;       // 62_000.0
    private static final double kAcceleration = 30_000.0;   // 44_000.0

    private static final double MIN_POSITION = -1_500.0;
    private static final double MAX_POSITION = 200_000.0; // 110_000
    public static final double FIRST_RUNG_HEIGHT = 130_000.0; // 90_000
    public static final double ABOVE_RUNG_HEIGHT = 25_000.0; // 25_000
    public static final double FULL_EXTEND = MAX_POSITION;

    private static final LeanboiState DEFAULT_LEANBOI_STATE = LeanboiState.EXTENDED;

    private TalonFX leftMotor = new TalonFX(Constants.Climber.LEFT_MOTOR_ID, "canivore");
    private TalonFX rightMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR_ID, "canivore");

    private DoubleSolenoid leanboi;

    private enum LeanboiState { EXTENDED, RETRACTED }
    private LeanboiState currentLeanboiState = DEFAULT_LEANBOI_STATE;

    public int currentClimbState = 0;

    public ClimberSubsystem (PneumaticHub hub) {
        leanboi = hub.makeDoubleSolenoid(Constants.Climber.LEFT_LEANBOI_ID, Constants.Climber.RIGHT_LEANBOI_ID);

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        leftMotor.setInverted(TalonFXInvertType.CounterClockwise);
        rightMotor.setInverted(TalonFXInvertType.Clockwise);

        leftMotor.config_kP(0, kP);
        leftMotor.config_kI(0, kI);
        leftMotor.config_kD(0, kD);
        leftMotor.config_kF(0, kF);

        rightMotor.config_kP(0, kP);
        rightMotor.config_kI(0, kI);
        rightMotor.config_kD(0, kD);
        rightMotor.config_kF(0, kF);

        leftMotor.configMotionCruiseVelocity(kVelocity);
        leftMotor.configMotionAcceleration(kAcceleration);
        rightMotor.configMotionCruiseVelocity(kVelocity);
        rightMotor.configMotionAcceleration(kAcceleration);

        setLeanboiState(LeanboiState.EXTENDED);

        resetMotors();
        dashboard();
    }


    /* 
    * ####################### 
    *   Motor Functions
    * #######################
    */
    private void resetMotors () {
        leftMotor.setSelectedSensorPosition(0.0);
        rightMotor.setSelectedSensorPosition(0.0);
    }

    public void stopMotors () {
        leftMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        rightMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void setMotorPosition (double pos) {
        if (Math.abs(pos) <= MAX_POSITION && pos >= MIN_POSITION) {
            leftMotor.set(TalonFXControlMode.MotionMagic, -pos);
            rightMotor.set(TalonFXControlMode.MotionMagic, -pos);
        } else {
            stopMotors();
        }
    }

    public void setMotorPosition (double pos, boolean fast) {
        double FAST_kP = 0.84;
        double FAST_vel = 70_000.0; // 62_000.0
        double FAST_acc = 44_000.0;

        double SLOW_kP = 0.61;
        double SLOW_vel = 46_000.0; // 40_000.0
        double SLOW_acc = 30_000.0;

        if (Math.abs(pos) <= MAX_POSITION && pos >= MIN_POSITION) {
            if (fast) {
                leftMotor.config_kP(0, FAST_kP);
                rightMotor.config_kP(0, FAST_kP);
                leftMotor.configMotionCruiseVelocity(FAST_vel);
                rightMotor.configMotionCruiseVelocity(FAST_vel);
                leftMotor.configMotionAcceleration(FAST_acc);
                rightMotor.configMotionAcceleration(FAST_acc);
            } else {
                leftMotor.config_kP(0, SLOW_kP);
                rightMotor.config_kP(0, SLOW_kP);
                leftMotor.configMotionCruiseVelocity(SLOW_vel);
                rightMotor.configMotionCruiseVelocity(SLOW_vel);
                leftMotor.configMotionAcceleration(SLOW_acc);
                rightMotor.configMotionAcceleration(SLOW_acc);
            }

            leftMotor.set(TalonFXControlMode.MotionMagic, -pos);
            rightMotor.set(TalonFXControlMode.MotionMagic, -pos);
        } else {
            stopMotors();
        }
    }

    public double getLeftMotorPosition () { return -leftMotor.getSelectedSensorPosition(); }
    public double getRightMotorPosition () { return -rightMotor.getSelectedSensorPosition(); }

    public double getMotorPosition () {
        return leftMotor.getSelectedSensorPosition();
    }

    public void setPower (double leftPower, double rightPower) {
        double leftPosition = leftMotor.getSelectedSensorPosition();
        if (Math.abs(leftPosition) <= MAX_POSITION && leftPosition >= 0) {
            leftMotor.set(TalonFXControlMode.PercentOutput, leftPower);
        } else {
            leftMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }

        double rightPosition = rightMotor.getSelectedSensorPosition();
        if (Math.abs(rightPosition) <= MAX_POSITION && rightPosition >= 0) {
            rightMotor.set(TalonFXControlMode.PercentOutput, rightPower);
        } else {
            rightMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }


    /*
    * ####################### 
    *   Leanboi Functions
    * #######################
    */
    private void setLeanboiState (LeanboiState state) {
        if (currentLeanboiState != state) {
            currentLeanboiState = state;
            switch (state) {
                case RETRACTED:
                    leanboi.set(Value.kReverse);
                    break;

                case EXTENDED:
                    leanboi.set(Value.kForward);
                    break;
            }
        }
    }
    private String getLeanboiStateName () { return currentLeanboiState.name(); }

    public void extendLeanboi () { setLeanboiState(LeanboiState.EXTENDED); }
    public void retractLeanboi () { setLeanboiState(LeanboiState.RETRACTED); }

    public void progressClimb () {
        currentClimbState++;
    }

    public void regressClimb () {
        currentClimbState--;
    }

    @Override
    public void periodic() {
        // refreshExtendoState();
        // refreshTriggerState();
        // refreshLeanboiState();
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.add(this);
        tab.addString("Leanboi State", this::getLeanboiStateName);

        tab.addNumber("Left Motor Pos", leftMotor::getSelectedSensorPosition);
        tab.addNumber("Right Motor Pos", rightMotor::getSelectedSensorPosition);
        tab.addNumber("Climb State", () -> currentClimbState);
    }

}
