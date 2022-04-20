package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private static final double TOP_SPEED_INDEXING      = 0.295; // 0.30
    private static final double BOTTOM_SPEED_INDEXING   = 0.50;

    private static final double TOP_SPEED_SHOOTING      = 0.20; // 0.15

    public enum IndexerState { EMPTY, LOW, HIGH, FULL }
    private IndexerState currentIndexerState = IndexerState.EMPTY;

    private enum ShootingState { NO_SHOOT, SHOOT }
    private ShootingState currentShootingState = ShootingState.NO_SHOOT;

    private TalonFX topMotor    = new TalonFX(Constants.Indexer.TOP_MOTOR_ID, "canivore");
    private TalonFX bottomMotor = new TalonFX(Constants.Indexer.BOTTOM_MOTOR_ID, "canivore");

    private DigitalInput topSensor      = new DigitalInput(2);
    private DigitalInput bottomSensor   = new DigitalInput(5); // 1

    private boolean isRunning = false;

    public IndexerSubsystem () {
        topMotor.setInverted(TalonFXInvertType.CounterClockwise);
        bottomMotor.setInverted(TalonFXInvertType.CounterClockwise);

        topMotor.setNeutralMode(NeutralMode.Brake);
        bottomMotor.setNeutralMode(NeutralMode.Brake);

        dashboard();
    }

    private void setIndexerState (IndexerState state) {
        this.currentIndexerState = state;
        if (isRunning) {
            switch (state) {
                case EMPTY:
                case LOW:
                    //? low and high index until high sensor detects it
                    fullIndex();
                    break;

                case HIGH:
                    //? low index until low sensor detects it
                    if (this.currentShootingState == ShootingState.SHOOT) {
                        highIndexShooting();
                    } else {
                        lowIndex();
                    }
                    break;

                case FULL:
                    if (this.currentShootingState == ShootingState.SHOOT) {
                        highIndexShooting();
                    } else {
                        stop();
                    }
                    break;
            }
        } else {
            stop();
        }
    }
    public IndexerState getIndexerState () { return this.currentIndexerState; }

    public void enableIndexing () { isRunning = true; }
    public void disableIndexing () { isRunning = false; }

    private void lowIndex () {
        topMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        bottomMotor.set(TalonFXControlMode.PercentOutput, BOTTOM_SPEED_INDEXING);
    }

    public void highIndexShooting () {
        topMotor.set(TalonFXControlMode.PercentOutput, TOP_SPEED_SHOOTING);
        bottomMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    private void fullIndex () {
        topMotor.set(TalonFXControlMode.PercentOutput, TOP_SPEED_INDEXING);
        bottomMotor.set(TalonFXControlMode.PercentOutput, BOTTOM_SPEED_INDEXING);
    }

    public void reverse () {
        topMotor.set(TalonFXControlMode.PercentOutput, -0.35);
        bottomMotor.set(TalonFXControlMode.PercentOutput, -0.35);
    }

    public void stop () {
        topMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        bottomMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    private void setShootingState (ShootingState state) { this.currentShootingState = state; }
    public void enableShooting () { setShootingState(ShootingState.SHOOT); }
    public void disableShooting () { setShootingState(ShootingState.NO_SHOOT); }

    @Override
    public void periodic() {
        //? use sensor data to determine indexer state
        boolean isLowSensorBlocked  = bottomSensor.get();
        boolean isHighSensorBlocked = topSensor.get();

        if (isLowSensorBlocked && isHighSensorBlocked) setIndexerState(IndexerState.FULL);
        else if (isLowSensorBlocked) setIndexerState(IndexerState.LOW);
        else if (isHighSensorBlocked) setIndexerState(IndexerState.HIGH);
        else setIndexerState(IndexerState.EMPTY);

        // setIndexerState(IndexerState.EMPTY);
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.add(this);
        tab.addString("Indexer State", () -> this.currentIndexerState.name());
        tab.addString("Shooting State", () -> this.currentShootingState.name());
        tab.addBoolean("Bottom Sensor", bottomSensor::get);
        tab.addBoolean("Top Sensor", topSensor::get);
    }
    
}
