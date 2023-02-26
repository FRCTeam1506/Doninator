package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.function.Supplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
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
    int rY = 199;
    int gY = 179;
    int bY = 2;
    
    int rP = 212;
    int gP = 66;
    int britishPetroleum = 245;

    public OurBeautifulGlowingCANdleSubsystem () {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // set the strip type to RGB
        config.brightnessScalar = 1; // dim the LEDs to half brightness
        candle.configAllSettings(config);
        dashboard();
    }

    public void purple () {
        stopGSA();
        candle.setLEDs(rP, gP, britishPetroleum); // set the CANdle LEDs to white
        color = "purple";
        Constants.CandleSubsystem.cone = false;
    }

    public void yellow () {
        stopGSA();
        color = "yellow";
        candle.setLEDs(rY, gY, bY); // set the CANdle LEDs to white
        Constants.CandleSubsystem.cone = true;

    }

    public void incrementColor(int x){
        rY += x;
        gY += x;
        bY += x;
        
        rP += x;
        gP += x;
        britishPetroleum += x;
    
    }

    public void gsa(){
        RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
        candle.animate(rainbowAnim);
    }

    public void stopGSA(){
        candle.animate(null);
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
