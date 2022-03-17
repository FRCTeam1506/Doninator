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

    private static final double kP = 0.84;   // 0.84
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.4;  // 0.4

    private static final double kVelocity = 62_000.0;       // 70_000.0
    private static final double kAcceleration = 44_000.0;   // 50_000.0

    private static final double MIN_POSITION = -1_000.0;
    private static final double MAX_POSITION = 150_000.0; // 110_000
    public static final double FIRST_RUNG_HEIGHT = 130_000.0; // 90_000
    public static final double ABOVE_RUNG_HEIGHT = 25_000.0; // 25_000
    public static final double FULL_EXTEND = MAX_POSITION;

    private static final ExtendoState DEFAULT_EXTENDO_STATE = ExtendoState.RETRACTED;
    private static final LeanboiState DEFAULT_LEANBOI_STATE = LeanboiState.EXTENDED;
    private static final TriggerState DEFAULT_TRIGGER_STATE = TriggerState.RETRACTED;

    private TalonFX leftMotor = new TalonFX(Constants.Climber.LEFT_MOTOR_ID, "canivore");
    private TalonFX rightMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR_ID, "canivore");

    private DoubleSolenoid extendo;
    private DoubleSolenoid leanboi;
    private DoubleSolenoid trigger;

    private enum ExtendoState { EXTENDED, RETRACTED }
    private ExtendoState currentExtendoState = DEFAULT_EXTENDO_STATE;

    private enum LeanboiState { EXTENDED, RETRACTED }
    private LeanboiState currentLeanboiState = DEFAULT_LEANBOI_STATE;

    private enum TriggerState { EXTENDED, RETRACTED }
    private TriggerState currentTriggerState = DEFAULT_TRIGGER_STATE;

    public int currentClimbState = 0;

    public ClimberSubsystem (PneumaticHub hub) {
        extendo = hub.makeDoubleSolenoid(Constants.Climber.LEFT_EXTENDO_ID, Constants.Climber.RIGHT_EXTENDO_ID);
        leanboi = hub.makeDoubleSolenoid(Constants.Climber.LEFT_LEANBOI_ID, Constants.Climber.RIGHT_LEANBOI_ID);
        trigger = hub.makeDoubleSolenoid(Constants.Climber.LEFT_TRIGGER_ID, Constants.Climber.RIGHT_TRIGGER_ID);

        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();

        leftMotor.setNeutralMode(NeutralMode.Brake);
        rightMotor.setNeutralMode(NeutralMode.Brake);

        leftMotor.setInverted(TalonFXInvertType.Clockwise);
        rightMotor.setInverted(TalonFXInvertType.CounterClockwise);

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
        setTriggerState(TriggerState.EXTENDED);
        setExtendoState(ExtendoState.RETRACTED);

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
            // double leftPos = leftMotor.getSelectedSensorPosition();
            // double rightPos = rightMotor.getSelectedSensorPosition();
            // while (!(leftPos <= pos - 50 && leftPos >= pos + 50 && rightPos <= pos - 50 && rightPos >= pos + 50)) {
                leftMotor.set(TalonFXControlMode.MotionMagic, pos);
                rightMotor.set(TalonFXControlMode.MotionMagic, pos);
            // }
        } else {
            stopMotors();
        }
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
    *   Extendo Functions
    * #######################
    */
    private void setExtendoState (ExtendoState state) {
        currentExtendoState = state;
        switch (state) {
            case RETRACTED:
                extendo.set(Value.kForward);
                break;

            case EXTENDED:
                extendo.set(Value.kReverse);
                break;
        }
    }
    private String getExtendoStateName () { return currentExtendoState.name(); }

    public void extendExtendo () { setExtendoState(ExtendoState.EXTENDED); }
    public void retractExtendo () { setExtendoState(ExtendoState.RETRACTED); }


    /*
    * ####################### 
    *   Leanboi Functions
    * #######################
    */
    private void setLeanboiState (LeanboiState state) {
        currentLeanboiState = state;
        switch (state) {
            case RETRACTED:
                leanboi.set(Value.kForward);
                break;

            case EXTENDED:
                leanboi.set(Value.kReverse);
                break;
        }
    }
    private String getLeanboiStateName () { return currentLeanboiState.name(); }

    public void extendLeanboi () { setLeanboiState(LeanboiState.EXTENDED); }
    public void retractLeanboi () { setLeanboiState(LeanboiState.RETRACTED); }


    /* 
    * ####################### 
    *   Trigger Functions
    * #######################
    */
    private void setTriggerState (TriggerState state) {
        currentTriggerState = state;
        switch (state) {
            case RETRACTED:
                trigger.set(Value.kReverse);
                break;

            case EXTENDED:
                trigger.set(Value.kForward);
                break;
        }
    }
    private String getTriggerStateName () { return currentTriggerState.name(); }

    public void extendTrigger () { setTriggerState(TriggerState.EXTENDED); }
    public void retractTrigger () { setTriggerState(TriggerState.RETRACTED); }


    private void setClimbState (int state) {
        currentClimbState = state;
        switch (state) {
            case 0:
                break;

            case 1:
                extendExtendo();
                retractLeanboi();
                retractTrigger();
                break;

            case 2:
                setMotorPosition(FIRST_RUNG_HEIGHT);
                break;

            case 3:
                setMotorPosition(0.0);
                break;

            case 4:
                setMotorPosition(ABOVE_RUNG_HEIGHT);
                break;

            case 5:
                setLeanboiState(LeanboiState.RETRACTED);
                break;

            case 6:
                setMotorPosition(FULL_EXTEND);
                break;

            case 7:
                setLeanboiState(LeanboiState.EXTENDED);
                break;

            case 8:
                setMotorPosition(ABOVE_RUNG_HEIGHT);
                break;

            case 9:
                setMotorPosition(0.0);
                break;

            case 10:
                setMotorPosition(ABOVE_RUNG_HEIGHT);
                break;

            case 11:
                setLeanboiState(LeanboiState.RETRACTED);
                break;

            case 12:
                setMotorPosition(FULL_EXTEND);
                break;

            case 13:
                setLeanboiState(LeanboiState.EXTENDED);
                break;
        }
    }

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
        tab.addString("Extendo State", this::getExtendoStateName);
        tab.addString("Leanboi State", this::getLeanboiStateName);
        tab.addString("Trigger State", this::getTriggerStateName);

        tab.addNumber("Left Motor Pos", leftMotor::getSelectedSensorPosition);
        tab.addNumber("Right Motor Pos", rightMotor::getSelectedSensorPosition);
        tab.addNumber("Climb State", () -> currentClimbState);
    }

}
