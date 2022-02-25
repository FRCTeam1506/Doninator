package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

class LimelightData {

    public double x, y, area = 0.0;
    public boolean isAligned, isTargeting = false;
    public int currentPipeline = 0;

    public LimelightData () {}
}

public class TurretSubsystem extends SubsystemBase {

    private TalonFX motor = new TalonFX(Constants.Turret.MOTOR_ID);

    private LimelightData limelightData = new LimelightData();
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public TurretSubsystem () {
        motor.configFactoryDefault();

        // motor.setNeutralMode(NeutralMode.Brake);
        motor.setNeutralMode(NeutralMode.Coast);

        //? limit switch "hard stops"
        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

        //? pid for motion magic
        motor.config_kP(0, 1.0);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.0);
        motor.config_kF(0, 0.2);

        //? velocity and acceleration for motion magic
        motor.configMotionCruiseVelocity(10_000.0);
        motor.configMotionAcceleration(7_000.0);

        this.reset();
        dashboard();
    }

    private void reset () {
        motor.setSelectedSensorPosition(0.0);
    }

    public double getSensorPosition () {
        return motor.getSelectedSensorPosition(); // 25000
    }

    public void stop () {
        motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }

    public void setPosition (double pos) {
        if ((Math.abs(pos) <= 25000.0)) {
            motor.set(TalonFXControlMode.MotionMagic, pos);
        }
    }


    private void limelightPeriodic () {
        limelightData.x                 = table.getEntry("tx").getDouble(0.0);
        limelightData.y                 = table.getEntry("ty").getDouble(0.0);
        limelightData.area              = table.getEntry("ta").getDouble(0.0);
        limelightData.isTargeting       = table.getEntry("tv").getNumber(0).intValue() == 1 ? true : false;
        limelightData.currentPipeline   = table.getEntry("pipeline").getNumber(0).intValue();

        limelightData.isAligned = Math.abs(limelightData.x) < 0.1 ? true : false;
    }

    @Override
    public void periodic() {
        limelightPeriodic();
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Turret");
        tab.add(this);
        tab.addNumber("Sensor Data", this::getSensorPosition);
    }

}
