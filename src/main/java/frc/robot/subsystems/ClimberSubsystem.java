package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ClimberSubsystem extends SubsystemBase {

    private static final double kP = 0.2;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kF = 0.05;

    private static final double kVelocity = 70_000.0;
    private static final double kAcceleration = 50_000.0;

    private static final double MAX_POSITION = 150_000.0;
    private static final double FIRST_RUNG_HEIGHT = 75_000.0;
    private static final double ABOVE_RUNG_HEIGHT = 25_000.0;
    private static final double FULL_EXTEND = MAX_POSITION;

    private TalonFX leftMotor = new TalonFX(Constants.Climber.LEFT_MOTOR_ID);
    private TalonFX rightMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR_ID);

    private DoubleSolenoid extendo = RobotContainer.hub.makeDoubleSolenoid(Constants.Climber.LEFT_EXTENDO_ID, Constants.Climber.RIGHT_EXTENDO_ID);
    private DoubleSolenoid leanboi = RobotContainer.hub.makeDoubleSolenoid(Constants.Climber.LEFT_LEANBOI_ID, Constants.Climber.RIGHT_LEANBOI_ID);
    private DoubleSolenoid trigger = RobotContainer.hub.makeDoubleSolenoid(Constants.Climber.LEFT_TRIGGER_ID, Constants.Climber.RIGHT_TRIGGER_ID);

    private enum ExtendoState { EXTENDED, RETRACTED }
    private ExtendoState currentExtendoState = ExtendoState.RETRACTED;

    private enum LeanboiState { EXTENDED, RETRACTED }
    private LeanboiState currentLeanboiState = LeanboiState.RETRACTED;

    private enum TriggerState { EXTENDED, RETRACTED }
    private TriggerState currentTriggerState = TriggerState.RETRACTED;

    private enum ClimbState { DEFAULT, FIRST, SECOND, THIRD, FOURTH, FIFTH, SIXTH, SEVENTH, EIGHTH, NINTH, TENTH, AAAA, BBBB, CCCC }
    private ClimbState currentClimbState = ClimbState.DEFAULT;

    public ClimberSubsystem () {
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
        if ((Math.abs(pos) <= MAX_POSITION)) {
            leftMotor.set(TalonFXControlMode.MotionMagic, pos);
            rightMotor.set(TalonFXControlMode.MotionMagic, pos);
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


    private void setClimbState (ClimbState state) {
        currentClimbState = state;
        switch (state) {
            case DEFAULT:
                setLeanboiState(LeanboiState.RETRACTED);
                setTriggerState(TriggerState.EXTENDED);
                break;

            case FIRST:
                setExtendoState(ExtendoState.EXTENDED);
                setLeanboiState(LeanboiState.EXTENDED);
                setTriggerState(TriggerState.RETRACTED);
                break;

            case SECOND:
                setMotorPosition(FIRST_RUNG_HEIGHT);
                break;

            case THIRD:
                setMotorPosition(0.0);
                break;

            case FOURTH:
                setMotorPosition(ABOVE_RUNG_HEIGHT);
                break;

            case FIFTH:
                setLeanboiState(LeanboiState.RETRACTED);
                break;

            case SIXTH:
                setMotorPosition(FULL_EXTEND);
                break;

            case SEVENTH:
                setLeanboiState(LeanboiState.EXTENDED);
                break;

            case EIGHTH:
                setMotorPosition(ABOVE_RUNG_HEIGHT);
                break;

            case NINTH:
                setMotorPosition(0.0);
                break;

            case TENTH:
                setMotorPosition(ABOVE_RUNG_HEIGHT);
                break;

            case AAAA:
                setLeanboiState(LeanboiState.RETRACTED);
                break;

            case BBBB:
                setMotorPosition(FULL_EXTEND);
                break;

            case CCCC:
                setLeanboiState(LeanboiState.EXTENDED);
                break;
        }
    }

    public void progressClimb () {
        switch (currentClimbState) {
            case DEFAULT:
                setClimbState(ClimbState.FIRST);
                break;
            
            case FIRST:
                setClimbState(ClimbState.SECOND);
                break;

            case SECOND:
                setClimbState(ClimbState.THIRD);
                break;

            case THIRD:
                setClimbState(ClimbState.FOURTH);
                break;

            case FOURTH:
                setClimbState(ClimbState.FIFTH);
                break;

            case FIFTH:
                setClimbState(ClimbState.SIXTH);
                break;

            case SIXTH:
                setClimbState(ClimbState.SEVENTH);
                break;

            case SEVENTH:
                setClimbState(ClimbState.EIGHTH);
                break;

            case EIGHTH:
                setClimbState(ClimbState.NINTH);
                break;

            case NINTH:
                setClimbState(ClimbState.TENTH);
                break;

            case TENTH:
                setClimbState(ClimbState.AAAA);
                break;

            case AAAA:
                setClimbState(ClimbState.BBBB);
                break;

            case BBBB:
                setClimbState(ClimbState.CCCC);
                break;

            case CCCC:
                System.out.println("stfu, you are down climbing");
                break;
        }
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Climber");
        tab.add(this);
        tab.addString("Extendo State", this::getExtendoStateName);
        tab.addString("Leanboi State", this::getLeanboiStateName);
        tab.addString("Trigger State", this::getTriggerStateName);
    }

}
