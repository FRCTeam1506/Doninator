package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OurBeautifulGlowingCANdleSubsystem extends SubsystemBase {

    CANdle candle = new CANdle(Constants.CandleSubsystem.CANDLE_ID); // creates a new CANdle with ID 0
    String color;

    public OurBeautifulGlowingCANdleSubsystem () {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1; // dim the LEDs to half brightness
        candle.configAllSettings(config);
        dashboard();
    }

    public void purple () {
        candle.setLEDs(212, 66, 245); // set the CANdle LEDs to white
        color = "purple";
    }

    public void yellow () {
        color = "yellow";
        candle.setLEDs(199, 179, 2); // set the CANdle LEDs to white

    }

    public void incrementOne(){
        // candle
    }

    // public Supplier<String> getColor(){
    //     return (Supplier<String>) candle;
    // }


    private void dashboard () {
        // ShuffleboardTab tab = Shuffleboard.getTab("Candle Color");
        // tab.add(this);
        // tab.addString("Candle Color ",getColor());
    }
    
}
