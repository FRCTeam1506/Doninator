package frc.robot.subsystems;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {

    private static final double DEFAULT_BOTTOM_SPEED = 0.25;  // 0.77 0.10
    private static final double DEFAULT_TOP_SPEED = 0.21;     // 0.48 0.13 0.35 0.21

    private enum IndexerState { EMPTY, LOW, HIGH, FULL}
    private IndexerState currentIndexerState = IndexerState.EMPTY;

    private enum ShootingState { NO_SHOOT, SHOOT }
    private ShootingState currentShootingState = ShootingState.NO_SHOOT;

    private TalonFX bottomMotor = new TalonFX(Constants.Indexer.BOTTOM_MOTOR_ID, "canivore");
    private TalonFX topMotor = new TalonFX(Constants.Indexer.TOP_MOTOR_ID, "canivore");

    private DigitalInput lowSensor = new DigitalInput(2);
    private DigitalInput highSensor = new DigitalInput(1);

    private boolean isRunning = false;

    public IndexerSubsystem () {
        bottomMotor.setInverted(TalonFXInvertType.Clockwise);
        topMotor.setInverted(TalonFXInvertType.CounterClockwise);
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
                        highIndex();
                    } else {
                        lowIndex();
                    }
                    break;

                case FULL:
                    if (this.currentShootingState == ShootingState.SHOOT) {
                        highIndex();
                    } else {
                        stop();
                    }
                    break;
            }
        } else {
            stop();
        }
    }

    public void enableIndexing () { isRunning = true; }
    public void disableIndexing () { isRunning = false; }

    private void lowIndex () { bottomMotor.set(TalonFXControlMode.PercentOutput, DEFAULT_BOTTOM_SPEED); }
    private void highIndex () { topMotor.set(TalonFXControlMode.PercentOutput, DEFAULT_TOP_SPEED); }

    private void fullIndex () {
        bottomMotor.set(TalonFXControlMode.PercentOutput, DEFAULT_BOTTOM_SPEED);
        topMotor.set(TalonFXControlMode.PercentOutput, DEFAULT_TOP_SPEED);
    }

    public void stop () {
        bottomMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        topMotor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    private void setShootingState (ShootingState state) { this.currentShootingState = state; }
    public void enableShooting () { setShootingState(ShootingState.SHOOT); }
    public void disableShooting () { setShootingState(ShootingState.NO_SHOOT); }

    @Override
    public void periodic() {
        //? use sensor data to determine indexer state
        boolean isLowSensorBlocked  = !lowSensor.get();
        boolean isHighSensorBlocked = !highSensor.get();

        if (isLowSensorBlocked && isHighSensorBlocked) this.setIndexerState(IndexerState.FULL);
        else if (isLowSensorBlocked) this.setIndexerState(IndexerState.LOW);
        else if (isHighSensorBlocked) this.setIndexerState(IndexerState.HIGH);
        else this.setIndexerState(IndexerState.EMPTY);
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Indexer");
        tab.add(this);
        tab.addString("Indexer State", () -> this.currentIndexerState.name());
        tab.addString("Shooting State", () -> this.currentShootingState.name());
        tab.addBoolean("Low Sensor", lowSensor::get);
        tab.addBoolean("High Sensor", highSensor::get);
    }
    
}
