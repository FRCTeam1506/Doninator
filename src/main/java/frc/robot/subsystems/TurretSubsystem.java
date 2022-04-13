package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

    enum ControlState { MANUAL, AUTO }
    private ControlState currentControlState = ControlState.AUTO;

    enum HoodState { UP, DOWN }

    class LimelightData {

        public double x, y, area = 0.0;
        public boolean isAligned, isTargeting = false;
        public int currentPipeline = 0;
    
        public LimelightData () {}
    
        public double getDistance (HoodState state) {
            if (!isTargeting) return 0.0;
            else {
                double H1_LIMELIGHT_HEIGHT_HUP      = Units.metersToInches(1.1176); // 44 in
                double H1_LIMELIGHT_HEIGHT_HDOWN    = Units.metersToInches(1.0668); // 42 in
                double H2_GOAL_HEIGHT   = Units.metersToInches(2.591); // 8.5 ft
                double A1_HOOD_UP       = Units.degreesToRadians(16.0);
                double A1_HOOD_DOWN     = Units.degreesToRadians(32.0);
                double A2_Y_DIFF = Units.degreesToRadians(this.y);
    
                double A1;
                switch (state) {
                    case UP:
                        A1 = A1_HOOD_UP;
                        break;
    
                    case DOWN:
                        A1 = A1_HOOD_DOWN;
                        break;
    
                    default:
                        A1 = A1_HOOD_UP;
                        break;
                }

                double H1;
                switch (state) {
                    case UP:
                        H1 = H1_LIMELIGHT_HEIGHT_HUP;
                        break;

                    case DOWN:
                        H1 = H1_LIMELIGHT_HEIGHT_HDOWN;
                        break;

                    default:
                        H1 = H1_LIMELIGHT_HEIGHT_HUP;
                        break;
                }
    
                // (h2 - h1) / Math.tan(a1 + a2)
                double distance = (H2_GOAL_HEIGHT - H1) / Math.tan(A1 + A2_Y_DIFF);
                return Math.abs(distance);
            }
        }
    }

    private static final double TURRET_MAX_TICKS = 75_000.0;

    private TalonFX motor = new TalonFX(Constants.Turret.MOTOR_ID, "canivore");

    private HoodState currentHoodState = HoodState.UP;
    private Solenoid hood;

    private LimelightData limelightData = new LimelightData();
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    public TurretSubsystem (PneumaticHub hub) {
        hood = hub.makeSolenoid(Constants.Turret.HOOD_ID);

        motor.configFactoryDefault();

        // motor.setNeutralMode(NeutralMode.Brake);
        motor.setNeutralMode(NeutralMode.Coast);

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

    /* 
    * ####################### 
    *   Motor Functions
    * #######################
    */
    private void reset () { motor.setSelectedSensorPosition(0.0); }

    public void stop () { motor.set(TalonFXControlMode.PercentOutput, 0.0); }

    public void setPosition (double pos) {
        if (Math.abs(pos) <= TURRET_MAX_TICKS) {
            motor.set(TalonFXControlMode.MotionMagic, pos);
        } else {
            if (pos < 0) motor.set(TalonFXControlMode.MotionMagic, -TURRET_MAX_TICKS);
            else motor.set(TalonFXControlMode.MotionMagic, TURRET_MAX_TICKS);
        }
    }

    public void setPower (double power) {
        double pos = motor.getSelectedSensorPosition();
        if (Math.abs(pos) <= TURRET_MAX_TICKS) {
            motor.set(TalonFXControlMode.PercentOutput, power);
        } else {
            motor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    public void setVelocity (double velocity_ticks) {
        double pos = motor.getSelectedSensorPosition();
        if (Math.abs(pos) <= TURRET_MAX_TICKS) {
            motor.set(TalonFXControlMode.Velocity, velocity_ticks);
        } else {
            motor.set(TalonFXControlMode.PercentOutput, 0.0);
        }
    }

    public double getVelocity () {
        return motor.getSelectedSensorVelocity();
    }


    /* 
    * ####################### 
    *   Hood Functions
    * #######################
    */
    private void setHoodState (HoodState state) {
        currentHoodState = state;
        switch (state) {
            case UP:
                hood.set(false);
                break;

            case DOWN:
                hood.set(true);
                break;
        }
    }
    public void putHoodUp () { if (currentHoodState != HoodState.UP) setHoodState(HoodState.UP); }
    public void putHoodDown () { if (currentHoodState != HoodState.DOWN) setHoodState(HoodState.DOWN); }

    private void hoodPeriodic () {
        switch (currentHoodState) {
            case UP:
                if (limelightData.y >= 20.0) { putHoodDown(); }
                break;
            
            case DOWN:
                if (limelightData.y <= 0.0) { putHoodUp(); }
                break;
        }
    }


    /* 
    * ####################### 
    *   Limelight Functions
    * #######################
    */
    private void limelightPeriodic () {
        limelightData.x                 = table.getEntry("tx").getDouble(0.0);
        limelightData.y                 = table.getEntry("ty").getDouble(0.0);
        limelightData.area              = table.getEntry("ta").getDouble(0.0);
        limelightData.isTargeting       = table.getEntry("tv").getNumber(0).intValue() == 1 ? true : false;
        limelightData.currentPipeline   = table.getEntry("pipeline").getNumber(0).intValue();

        limelightData.isAligned = Math.abs(limelightData.x) < 0.1 ? true : false;
    }

    public double getXError () {
        return limelightData.x;
    }

    public double calculateShooterRPM () {
        double distance = limelightData.getDistance(currentHoodState); // 60 - 200

        int zone;
        if (distance <= 60) { zone = 0; }
        else if (distance > 60 && distance <= 65) { zone = 1; }
        else if (distance > 65 && distance <= 70) { zone = 2; }
        else if (distance > 70 && distance <= 75) { zone = 3; }
        else if (distance > 75 && distance <= 80) { zone = 4; }
        else if (distance > 80 && distance <= 85) { zone = 5; }
        else if (distance > 85 && distance <= 90) { zone = 6; }
        else if (distance > 90 && distance <= 95) { zone = 7; }
        else if (distance > 95 && distance <= 100) { zone = 8; }
        else if (distance > 100 && distance <= 105) { zone = 9; }
        else if (distance > 105 && distance <= 110) { zone = 10; }
        else if (distance > 110 && distance <= 115) { zone = 11; }
        else if (distance > 115 && distance <= 120) { zone = 12; }
        else if (distance > 120 && distance <= 125) { zone = 13; }
        else if (distance > 125 && distance <= 130) { zone = 14; }
        else if (distance > 130 && distance <= 135) { zone = 15; }
        else if (distance > 135 && distance <= 140) { zone = 16; }
        else if (distance > 140 && distance <= 145) { zone = 17; }
        else if (distance > 145 && distance <= 150) { zone = 18; }
        else if (distance > 150 && distance <= 155) { zone = 19; }
        else if (distance > 155 && distance <= 160) { zone = 20; }
        else if (distance > 160 && distance <= 165) { zone = 21; }
        else if (distance > 165 && distance <= 170) { zone = 22; }
        else if (distance > 170 && distance <= 175) { zone = 23; }
        else if (distance > 175 && distance <= 180) { zone = 24; }
        else if (distance > 180 && distance <= 185) { zone = 25; }
        else if (distance > 185 && distance <= 190) { zone = 26; }
        else if (distance > 190 && distance <= 195) { zone = 27; }
        else { zone = 28; }

        // return zone;

        switch (zone) {
            case 0:
                return 1800.0;

            case 1:
                return 1750.0;

            case 2:
                return 1700.0;

            case 3:
                return 1720.0;

            case 4:
                return 1730.0;

            case 5:
                return 1740.0;

            case 6:
                return 1750.0;

            case 7:
                return 1760.0;

            case 8:
                return 1780.0;

            case 9:
                return 1790.0;

            case 10:
                return 1820.0;

            case 11:
                return 1840.0;

            case 12:
                return 1900.0;

            case 13:
                return 1960.0;

            case 14:
                return 2030.0;

            case 15:
            case 16:
            case 17:
                return 2040.0;

            case 18:
                return 2060.0;

            case 19:
                return 2070.0;

            case 20:
                return 2080.0;

            case 21:
                return 2090.0;

            case 22:
                return 2100.0;

            case 23:
                return 2130.0;

            case 24:
                return 2150.0;

            case 25:
                return 2240.0;

            case 26:
                return 2300.0;

            case 27:
                return 2330.0;
        
            default:
                return 1800.0;
        }
    }

    public boolean isTracking () { return limelightData.isTargeting; }


    /* 
    * ####################### 
    *   Control Functions
    * #######################
    */
    private void setControlState (ControlState state) { currentControlState = state; }

    public void setManual () { setControlState(ControlState.MANUAL); }
    public void setAuto () { setControlState(ControlState.AUTO); }
    public boolean isManual () { return currentControlState == ControlState.MANUAL; }
    public boolean isAuto () { return currentControlState == ControlState.AUTO; }

    public void toggleControlState () {
        if (currentControlState == ControlState.MANUAL) setControlState(ControlState.AUTO);
        else setControlState(ControlState.MANUAL);
    }


    @Override
    public void periodic() {
        hoodPeriodic();
        limelightPeriodic();
    }

    private void dashboard () {
        ShuffleboardTab tab = Shuffleboard.getTab("Turret");
        tab.add(this);
        tab.addNumber("Turret Sensor Data", motor::getSelectedSensorPosition);
        tab.addNumber("Limelight Horiz Error ", this::getXError);
        tab.addNumber("Calculated Shooter RPM", this::calculateShooterRPM);
        tab.addNumber("Limelight Distance", () -> limelightData.getDistance(currentHoodState));
        tab.addBoolean("Control State", () -> this.currentControlState == ControlState.AUTO);
    }

}
